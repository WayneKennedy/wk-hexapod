#!/usr/bin/env python3
"""
Hexapod Controller Node

Body-centric control with IK, gait generation, and IMU stabilization.
Based on fn-hexapod/Code/Server/control.py architecture.

Following ROS hexapod best practices (KevinOchs pattern):
- Monolithic controller for low-latency real-time control
- IK, gait, and servo output in single node
- Standard ROS interfaces (cmd_vel, Pose, JointState)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, Pose, Vector3, TransformStamped
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, Float64MultiArray
from std_srvs.srv import Trigger, SetBool
from tf2_ros import TransformBroadcaster
from hexapod_interfaces.action import MoveDistance
import copy
import math
import numpy as np
import time
import threading

# Hardware imports
try:
    import smbus
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False

try:
    from gpiozero import OutputDevice
    GPIOZERO_AVAILABLE = True
except ImportError:
    GPIOZERO_AVAILABLE = False


class PCA9685:
    """PCA9685 PWM driver - from fn-hexapod pca9685.py"""
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, bus, address=0x40):
        self.bus = smbus.SMBus(bus)
        self.address = address
        self.bus.write_byte_data(self.address, self.MODE1, 0x00)

    def set_pwm_freq(self, freq):
        prescale = int(25000000.0 / (4096.0 * freq) - 1 + 0.5)
        old_mode = self.bus.read_byte_data(self.address, self.MODE1)
        self.bus.write_byte_data(self.address, self.MODE1, (old_mode & 0x7F) | 0x10)
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode | 0x80)

    def set_pwm(self, channel, on, off):
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel, on & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 1, on >> 8)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 2, off & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 3, off >> 8)

    def set_pwm_off(self, channel):
        self.set_pwm(channel, 4096, 4096)


class IncrementalPID:
    """Incremental PID controller from fn-hexapod pid.py"""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0.0
        self.prev_error = 0.0
        self.output = 0.0

    def calculate(self, error):
        delta = (self.kp * (error - self.last_error) +
                 self.ki * error +
                 self.kd * (error - 2 * self.last_error + self.prev_error))
        self.output += delta
        self.prev_error = self.last_error
        self.last_error = error
        return self.output

    def reset(self):
        self.last_error = 0.0
        self.prev_error = 0.0
        self.output = 0.0


class HexapodController(Node):
    """
    Main hexapod controller following body-centric design.

    Coordinate frames:
    - World frame: X forward, Y left, Z up (REP 103)
    - Body frame: Origin at body center
    - Leg frame: Origin at coxa pivot, X along leg
    """

    # Servo channel mapping (from fn-hexapod)
    LEG_CHANNELS = [
        [15, 14, 13],  # Leg 0 (RF): coxa, femur, tibia
        [12, 11, 10],  # Leg 1 (RM)
        [9, 8, 31],    # Leg 2 (RR)
        [22, 23, 27],  # Leg 3 (LR)
        [19, 20, 21],  # Leg 4 (LM)
        [16, 17, 18],  # Leg 5 (LF)
    ]
    HEAD_CHANNELS = [0, 1]  # pan, tilt

    # Link lengths (mm)
    L1 = 33.0   # coxa
    L2 = 90.0   # femur
    L3 = 110.0  # tibia

    def __init__(self):
        super().__init__('hexapod_controller')

        # ===== Parameters =====
        self.declare_parameters(namespace='', parameters=[
            ('body.leg_angles', [54.0, 0.0, -54.0, -126.0, 180.0, 126.0]),
            ('body.leg_offsets', [94.0, 85.0, 94.0, 94.0, 85.0, 94.0]),
            ('body.default_height', 30.0),
            ('body.height_offset', 14.0),
            ('balance.enabled', False),
            ('balance.kp', 0.5),
            ('balance.ki', 0.0),
            ('balance.kd', 0.0025),
            ('balance.update_rate', 50.0),
            ('gait.default', 'tripod'),
            ('gait.step_height', 40.0),
            ('gait.cycle_time', 1.0),
            ('odometry.imu_fusion', True),
            ('odometry.imu_yaw_weight', 0.98),
        ])

        self.leg_angles = self.get_parameter('body.leg_angles').value
        self.leg_offsets = self.get_parameter('body.leg_offsets').value
        self.default_height = self.get_parameter('body.default_height').value
        self.height_offset = self.get_parameter('body.height_offset').value
        self.balance_enabled = self.get_parameter('balance.enabled').value
        self.balance_rate = self.get_parameter('balance.update_rate').value

        # ===== Body State =====
        # Position (mm) and orientation (degrees)
        self.body_position = np.array([0.0, 0.0, -self.default_height])
        self.body_orientation = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw

        # Default foot positions in world frame (mm)
        self.foot_positions = np.array([
            [137.1, 189.4, self.body_position[2]],
            [225.0, 0.0, self.body_position[2]],
            [137.1, -189.4, self.body_position[2]],
            [-137.1, -189.4, self.body_position[2]],
            [-225.0, 0.0, self.body_position[2]],
            [-137.1, 189.4, self.body_position[2]],
        ])

        # Leg positions in leg-local frame (mm)
        self.leg_positions = [[140.0, 0.0, 0.0] for _ in range(6)]

        # Calibration
        self.calibration_positions = self._load_calibration()
        self.calibration_angles = [[0.0, 0.0, 0.0] for _ in range(6)]
        self._calculate_calibration()

        # Current servo angles (degrees)
        self.current_angles = [[90.0, 90.0, 90.0] for _ in range(6)]

        # ===== Hardware =====
        self.pwm_40 = None
        self.pwm_41 = None
        self.servo_power = None
        self._init_hardware()

        # ===== PID Controllers =====
        kp = self.get_parameter('balance.kp').value
        ki = self.get_parameter('balance.ki').value
        kd = self.get_parameter('balance.kd').value
        self.pid_roll = IncrementalPID(kp, ki, kd)
        self.pid_pitch = IncrementalPID(kp, ki, kd)

        # IMU state
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_yaw = 0.0
        self.imu_yaw_rad = 0.0  # Raw yaw in radians for odometry sync

        # ===== Odometry State =====
        # Position in world frame (meters)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0  # radians
        # Velocity (m/s, rad/s)
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_vyaw = 0.0
        # Last update time for velocity calculation
        self.last_odom_time = None

        # ===== State Machine =====
        self.is_initialized = False
        self.is_walking = False
        self.is_relaxed = True

        # ===== ROS Interfaces =====
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.body_pose_sub = self.create_subscription(
            Pose, 'body_pose', self.body_pose_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)
        self.pose_cmd_sub = self.create_subscription(
            String, 'pose_command', self.pose_command_callback, 10)

        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(
            Odometry, 'odom', 10)
        # Servo commands to servo_driver (18 leg angles + 2 head angles)
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, 'joint_commands', 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Services
        self.init_srv = self.create_service(
            Trigger, 'hexapod/initialize', self.initialize_callback)
        self.balance_srv = self.create_service(
            SetBool, 'hexapod/enable_balance', self.enable_balance_callback)
        self.reset_odom_srv = self.create_service(
            Trigger, 'hexapod/reset_odometry', self.reset_odometry_callback)

        # Action servers (use reentrant callback group for concurrent execution)
        self.action_cb_group = ReentrantCallbackGroup()
        self.move_distance_action = ActionServer(
            self,
            MoveDistance,
            'hexapod/move_distance',
            execute_callback=self.move_distance_execute,
            goal_callback=self.move_distance_goal_callback,
            cancel_callback=self.move_distance_cancel_callback,
            callback_group=self.action_cb_group
        )

        # Action state
        self.action_in_progress = False

        # Balance loop timer (only runs when enabled)
        self.balance_timer = None

        # Odometry publish timer (20 Hz)
        self.odom_timer = self.create_timer(0.05, self._publish_odometry)

        self.get_logger().info('Hexapod controller started (NOT initialized)')
        self.get_logger().info('Call /hexapod/initialize service when robot is safe')

    def _init_hardware(self):
        """Initialize PCA9685 servo drivers"""
        if not HARDWARE_AVAILABLE:
            self.get_logger().warn('smbus not available, running in simulation')
            return

        try:
            # Servo power control (GPIO 4, low = enabled)
            if GPIOZERO_AVAILABLE:
                self.servo_power = OutputDevice(4)
                self.servo_power.off()

            # PCA9685 drivers
            self.pwm_41 = PCA9685(1, 0x41)  # Channels 0-15
            self.pwm_41.set_pwm_freq(50)
            self.pwm_40 = PCA9685(1, 0x40)  # Channels 16-31
            self.pwm_40.set_pwm_freq(50)
            time.sleep(0.01)
            self.get_logger().info('Hardware initialized')
        except Exception as e:
            self.get_logger().error(f'Hardware init failed: {e}')

    def _load_calibration(self):
        """Load servo calibration from file"""
        paths = [
            '/home/wkenn/Code/wk-hexapod/ros2_ws/src/hexapod_hardware/config/servo_calibration.txt',
            '/home/wkenn/Code/fn-hexapod/Code/Server/point.txt',
        ]
        for path in paths:
            try:
                with open(path, 'r') as f:
                    data = [list(map(int, line.strip().split('\t')))
                            for line in f if line.strip()]
                    self.get_logger().info(f'Loaded calibration from {path}')
                    return data
            except:
                pass
        self.get_logger().warn('No calibration file, using defaults')
        return [[140, 0, 0] for _ in range(6)]

    def _calculate_calibration(self):
        """Calculate calibration angle offsets"""
        for i in range(6):
            cal = self._coordinate_to_angle(
                -self.calibration_positions[i][2],
                self.calibration_positions[i][0],
                self.calibration_positions[i][1])
            default = self._coordinate_to_angle(0, 140, 0)
            self.calibration_angles[i] = [cal[j] - default[j] for j in range(3)]

    # ===== Inverse Kinematics =====

    def _coordinate_to_angle(self, x, y, z):
        """IK: Convert leg endpoint (x,y,z) to servo angles (a,b,c)"""
        a = math.pi / 2 - math.atan2(z, y)
        x_4 = self.L1 * math.sin(a)
        x_5 = self.L1 * math.cos(a)
        l23 = math.sqrt((z - x_5)**2 + (y - x_4)**2 + x**2)

        w = self._clamp(x / l23, -1, 1)
        v = self._clamp((self.L2**2 + l23**2 - self.L3**2) / (2 * self.L2 * l23), -1, 1)
        u = self._clamp((self.L2**2 + self.L3**2 - l23**2) / (2 * self.L3 * self.L2), -1, 1)

        b = math.asin(round(w, 2)) - math.acos(round(v, 2))
        c = math.pi - math.acos(round(u, 2))

        return [round(math.degrees(a)), round(math.degrees(b)), round(math.degrees(c))]

    def _clamp(self, val, lo, hi):
        return max(lo, min(hi, val))

    # ===== Coordinate Transforms =====

    def _world_to_leg(self, foot_pos, leg_idx):
        """Transform foot position from world frame to leg-local frame"""
        angle_rad = math.radians(self.leg_angles[leg_idx])
        offset = self.leg_offsets[leg_idx]

        # Rotate and translate
        x = (foot_pos[0] * math.cos(angle_rad) +
             foot_pos[1] * math.sin(angle_rad) - offset)
        y = (-foot_pos[0] * math.sin(angle_rad) +
             foot_pos[1] * math.cos(angle_rad))
        z = foot_pos[2] - self.height_offset

        return [x, y, z]

    def _calculate_rotation_matrix(self, roll, pitch, yaw):
        """Calculate 3D rotation matrix from Euler angles (degrees)"""
        r = math.radians(roll)
        p = math.radians(pitch)
        y = math.radians(yaw)

        Rx = np.array([[1, 0, 0],
                       [0, math.cos(p), -math.sin(p)],
                       [0, math.sin(p), math.cos(p)]])
        Ry = np.array([[math.cos(r), 0, -math.sin(r)],
                       [0, 1, 0],
                       [math.sin(r), 0, math.cos(r)]])
        Rz = np.array([[math.cos(y), -math.sin(y), 0],
                       [math.sin(y), math.cos(y), 0],
                       [0, 0, 1]])

        return Rx @ Ry @ Rz

    def _apply_body_transform(self):
        """Calculate leg positions from body pose and foot positions"""
        R = self._calculate_rotation_matrix(*self.body_orientation)

        for i in range(6):
            # Apply body rotation and translation to foot position
            foot = np.array(self.foot_positions[i])
            transformed = R @ foot + self.body_position

            # Convert to leg-local coordinates
            self.leg_positions[i] = self._world_to_leg(transformed, i)

    # ===== Servo Output =====

    def _set_servo_angle(self, channel, angle):
        """Set servo angle (0-180 degrees)"""
        angle = self._clamp(angle, 0, 180)
        duty = (angle / 180.0) * (2500 - 500) + 500  # pulse width
        pwm_val = int(duty / 20000.0 * 4095)

        if channel < 16 and self.pwm_41:
            self.pwm_41.set_pwm(channel, 0, pwm_val)
        elif channel < 32 and self.pwm_40:
            self.pwm_40.set_pwm(channel - 16, 0, pwm_val)

    def _update_servos(self):
        """Calculate and send all servo angles"""
        # Apply body transform to get leg positions
        self._apply_body_transform()

        # Calculate servo angles for each leg
        for i in range(6):
            raw = self._coordinate_to_angle(
                -self.leg_positions[i][2],
                self.leg_positions[i][0],
                self.leg_positions[i][1])

            # Apply calibration and side correction
            if i < 3:  # Right side
                self.current_angles[i][0] = self._clamp(
                    raw[0] + self.calibration_angles[i][0], 0, 180)
                self.current_angles[i][1] = self._clamp(
                    90 - (raw[1] + self.calibration_angles[i][1]), 0, 180)
                self.current_angles[i][2] = self._clamp(
                    raw[2] + self.calibration_angles[i][2], 0, 180)
            else:  # Left side
                self.current_angles[i][0] = self._clamp(
                    raw[0] + self.calibration_angles[i][0], 0, 180)
                self.current_angles[i][1] = self._clamp(
                    90 + raw[1] + self.calibration_angles[i][1], 0, 180)
                self.current_angles[i][2] = self._clamp(
                    180 - (raw[2] + self.calibration_angles[i][2]), 0, 180)

        # Send to servo_driver via topic
        self._publish_joint_commands()

        # Publish joint states for visualization
        self._publish_joint_states()

    def _relax_servos(self):
        """Turn off all servo PWM"""
        if self.pwm_40:
            for i in range(16):
                self.pwm_40.set_pwm_off(i)
        if self.pwm_41:
            for i in range(16):
                self.pwm_41.set_pwm_off(i)
        self.is_relaxed = True
        self.get_logger().info('Servos relaxed')

    def _publish_joint_commands(self):
        """Publish joint angles to servo_driver"""
        msg = Float64MultiArray()
        # Format: [leg0_coxa, leg0_femur, leg0_tibia, leg1_coxa, ..., pan, tilt]
        data = []
        for i in range(6):
            for j in range(3):
                data.append(float(self.current_angles[i][j]))
        # Add head angles (default centered for now)
        data.append(90.0)  # pan
        data.append(90.0)  # tilt
        msg.data = data
        self.joint_cmd_pub.publish(msg)

    def _publish_joint_states(self):
        """Publish joint states for rviz visualization"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(6):
            for j, joint in enumerate(['coxa', 'femur', 'tibia']):
                msg.name.append(f'leg{i+1}_{joint}')
                msg.position.append(math.radians(self.current_angles[i][j]))

        self.joint_state_pub.publish(msg)

    def _update_odometry(self, dx_mm, dy_mm, dyaw_deg, dt):
        """
        Update odometry from gait cycle displacement.

        Args:
            dx_mm: Forward displacement in body frame (mm)
            dy_mm: Left displacement in body frame (mm)
            dyaw_deg: Rotation (degrees)
            dt: Time delta (seconds)
        """
        # Convert mm to meters
        dx = dx_mm / 1000.0
        dy = dy_mm / 1000.0
        dyaw = math.radians(dyaw_deg)

        # Transform body-frame displacement to world frame
        cos_yaw = math.cos(self.odom_yaw)
        sin_yaw = math.sin(self.odom_yaw)

        # Integrate position (body dx is forward, which maps to world based on yaw)
        self.odom_x += dx * cos_yaw - dy * sin_yaw
        self.odom_y += dx * sin_yaw + dy * cos_yaw
        self.odom_yaw += dyaw

        # Normalize yaw to [-pi, pi]
        while self.odom_yaw > math.pi:
            self.odom_yaw -= 2 * math.pi
        while self.odom_yaw < -math.pi:
            self.odom_yaw += 2 * math.pi

        # Calculate velocities
        if dt > 0:
            self.odom_vx = dx / dt
            self.odom_vy = dy / dt
            self.odom_vyaw = dyaw / dt

    def _publish_odometry(self):
        """Publish odometry message and TF transform."""
        now = self.get_clock().now()

        # Create quaternion from yaw
        # For 2D: qz = sin(yaw/2), qw = cos(yaw/2)
        qz = math.sin(self.odom_yaw / 2.0)
        qw = math.cos(self.odom_yaw / 2.0)

        # Publish TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Velocity (in body frame as per REP 105)
        odom.twist.twist.linear.x = self.odom_vx
        odom.twist.twist.linear.y = self.odom_vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.odom_vyaw

        # Covariance (diagonal, moderate uncertainty)
        # [x, y, z, roll, pitch, yaw]
        pose_cov = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.02]
        odom.pose.covariance = pose_cov

        twist_cov = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.02]
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

    def _reset_odometry(self, sync_imu_yaw=True):
        """Reset odometry to origin, optionally syncing yaw with IMU."""
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_vyaw = 0.0

        # Sync yaw with IMU if enabled and IMU data available
        if sync_imu_yaw and self.get_parameter('odometry.imu_fusion').value:
            self.odom_yaw = self.imu_yaw_rad
            self.get_logger().info(f'Odometry reset, yaw synced to IMU: {math.degrees(self.odom_yaw):.1f}°')
        else:
            self.odom_yaw = 0.0
            self.get_logger().info('Odometry reset to origin')

    # ===== Body Pose Commands =====

    def home(self, reset_odom=True):
        """Move to calibrated home position (flat on ground)"""
        self.get_logger().info('Moving to HOME position...')
        self.body_position = np.array([0.0, 0.0, 0.0])
        self.body_orientation = np.array([0.0, 0.0, 0.0])

        # Reset foot positions to neutral stance at ground level
        self.foot_positions = np.array([
            [137.1, 189.4, 0.0],
            [225.0, 0.0, 0.0],
            [137.1, -189.4, 0.0],
            [-137.1, -189.4, 0.0],
            [-225.0, 0.0, 0.0],
            [-137.1, 189.4, 0.0],
        ])

        self._update_servos()
        self.is_relaxed = False

        # Reset odometry when homing (robot is at known position)
        if reset_odom:
            self._reset_odometry()

        self.get_logger().info('HOME position set')

    def stand(self, height=None, duration=1.0):
        """Smoothly raise body to standing height"""
        if height is None:
            height = self.default_height

        self.get_logger().info(f'Standing (raising body {height}mm)...')

        steps = 50
        start_z = self.body_position[2]
        end_z = -height  # Negative Z raises body

        for step in range(steps + 1):
            t = step / steps
            t = t * t * (3 - 2 * t)  # Ease in-out

            self.body_position[2] = start_z + t * (end_z - start_z)
            self._update_servos()
            time.sleep(duration / steps)

        self.get_logger().info('STAND position set')

    def set_body_pose(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None):
        """Set body pose (position in mm, orientation in degrees)"""
        if x is not None:
            self.body_position[0] = self._clamp(x, -40, 40)
        if y is not None:
            self.body_position[1] = self._clamp(y, -40, 40)
        if z is not None:
            self.body_position[2] = self._clamp(z, -50, 0)
        if roll is not None:
            self.body_orientation[0] = self._clamp(roll, -15, 15)
        if pitch is not None:
            self.body_orientation[1] = self._clamp(pitch, -15, 15)
        if yaw is not None:
            self.body_orientation[2] = self._clamp(yaw, -15, 15)

        self._update_servos()

    # ===== IMU Balance =====

    def _balance_loop(self):
        """IMU balance feedback loop"""
        if not self.balance_enabled or self.is_relaxed:
            return

        # Apply PID correction to counteract measured tilt
        roll_correction = self.pid_roll.calculate(self.imu_roll)
        pitch_correction = self.pid_pitch.calculate(self.imu_pitch)

        # Update body orientation to compensate
        self.body_orientation[0] = self._clamp(roll_correction, -15, 15)
        self.body_orientation[1] = self._clamp(pitch_correction, -15, 15)

        self._update_servos()

    # ===== Gait Generation =====

    def _reset_to_stand(self):
        """Reset foot positions to neutral standing stance"""
        self.foot_positions = np.array([
            [137.1, 189.4, self.body_position[2]],
            [225.0, 0.0, self.body_position[2]],
            [137.1, -189.4, self.body_position[2]],
            [-137.1, -189.4, self.body_position[2]],
            [-225.0, 0.0, self.body_position[2]],
            [-137.1, 189.4, self.body_position[2]],
        ])
        self._update_servos()

    # Neutral foot positions (X, Y) for rotation calculations
    # These define where each foot sits relative to body center
    NEUTRAL_FEET = [
        [137.1, 189.4],    # leg 0 - right front
        [225.0, 0.0],      # leg 1 - right middle
        [137.1, -189.4],   # leg 2 - right rear
        [-137.1, -189.4],  # leg 3 - left rear
        [-225.0, 0.0],     # leg 4 - left middle
        [-137.1, 189.4],   # leg 5 - left front
    ]

    def _tripod_gait_cycle(self, y_move, x_move, turn):
        """
        One tripod gait cycle with full omni-directional movement.
        Based on fn-hexapod control.py rotation math.

        y_move: forward/back (mm per cycle)
        x_move: strafe left/right (mm per cycle)
        turn: rotation in degrees per cycle
        """
        step_height = self.get_parameter('gait.step_height').value
        cycle_time = self.get_parameter('gait.cycle_time').value

        frames = 64
        delay = cycle_time / frames

        # Calculate per-leg movement including rotation
        # Rotation: each foot moves based on rotating around body center
        # Negate turn: positive angular.z = CCW, but our math gives CCW for positive angle
        angle_rad = math.radians(-turn)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        leg_moves = []
        for i in range(6):
            nx, ny = self.NEUTRAL_FEET[i]
            # Rotation contribution: delta from rotating foot position around origin
            rot_x = nx * cos_a + ny * sin_a - nx
            rot_y = -nx * sin_a + ny * cos_a - ny
            # Total movement = rotation + translation
            leg_moves.append([rot_x + x_move, rot_y + y_move])

        # Store starting positions
        start_feet = [list(f) for f in self.foot_positions]
        cycle_start_time = time.time()

        for frame in range(frames):
            phase = frame / frames

            for i in range(6):
                is_odd = (i % 2 == 1)
                move_x, move_y = leg_moves[i]

                # Odd legs move first half, even legs move second half
                if is_odd:
                    leg_phase = phase * 2 if phase < 0.5 else 1.0
                else:
                    leg_phase = 0.0 if phase < 0.5 else (phase - 0.5) * 2

                # Calculate foot position
                if leg_phase < 1.0:
                    # Swing phase - leg in air moving forward
                    swing = math.sin(leg_phase * math.pi)
                    self.foot_positions[i][0] = start_feet[i][0] + move_x * (leg_phase - 0.5)
                    self.foot_positions[i][1] = start_feet[i][1] + move_y * (leg_phase - 0.5)
                    self.foot_positions[i][2] = self.body_position[2] + step_height * swing
                else:
                    # Stance complete
                    self.foot_positions[i][0] = start_feet[i][0] + move_x * 0.5
                    self.foot_positions[i][1] = start_feet[i][1] + move_y * 0.5
                    self.foot_positions[i][2] = self.body_position[2]

                # Push phase for grounded legs
                if (is_odd and phase >= 0.5) or (not is_odd and phase < 0.5):
                    push_phase = (phase - 0.5) if is_odd else phase
                    self.foot_positions[i][0] = start_feet[i][0] - move_x * push_phase * 2
                    self.foot_positions[i][1] = start_feet[i][1] - move_y * push_phase * 2

            self._update_servos()
            time.sleep(delay)

        # Update odometry after cycle completes
        cycle_time_actual = time.time() - cycle_start_time
        self._update_odometry(y_move, x_move, turn, cycle_time_actual)

        # Reset to neutral for next cycle
        self._reset_to_stand()

    def _run_gait_step(self, x, y, turn):
        """
        Execute one gait cycle step.

        Tripod gait: legs 0,2,4 (even) and 1,3,5 (odd) alternate
        Phase 0-49: Even legs swing forward while odd legs push back
        Phase 50-99: Odd legs swing forward while even legs push back

        Args:
            x: Forward/back motion (mm per step)
            y: Left/right motion (mm per step)
            turn: Rotation (degrees per step)
        """
        gait_type = self.get_parameter('gait.default').value
        step_height = self.get_parameter('gait.step_height').value
        cycle_time = self.get_parameter('gait.cycle_time').value

        if gait_type == 'tripod':
            self._tripod_gait_step(x, y, turn, step_height, cycle_time)
        else:
            self._wave_gait_step(x, y, turn, step_height, cycle_time)

    def _tripod_gait_step(self, x, y, turn, step_height, cycle_time):
        """
        Tripod gait: 3 legs move at a time (even/odd alternating).
        Ported from fn-hexapod control.py run_gait() gait="1"
        """
        # Calculate movement per foot based on position and rotation
        angle_rad = math.radians(turn)
        base_points = copy.deepcopy(self.foot_positions)

        # Calculate movement delta for each leg
        xy_delta = []
        for i in range(6):
            # Rotation contribution
            rot_x = (base_points[i][0] * math.cos(angle_rad) +
                     base_points[i][1] * math.sin(angle_rad) - base_points[i][0])
            rot_y = (-base_points[i][0] * math.sin(angle_rad) +
                     base_points[i][1] * math.cos(angle_rad) - base_points[i][1])
            xy_delta.append([rot_x + x, rot_y + y])

        frames = 64  # Number of sub-steps per cycle
        z_delta = step_height / frames
        delay = cycle_time / frames

        for j in range(frames):
            for i in range(3):  # For each tripod pair
                even = 2 * i      # Legs 0, 2, 4
                odd = 2 * i + 1   # Legs 1, 3, 5

                if j < frames // 8:
                    # Even legs push back, odd legs lift and swing
                    base_points[even][0] -= 4 * xy_delta[even][0] / frames
                    base_points[even][1] -= 4 * xy_delta[even][1] / frames
                    base_points[odd][0] += 8 * xy_delta[odd][0] / frames
                    base_points[odd][1] += 8 * xy_delta[odd][1] / frames
                    base_points[odd][2] = self.body_position[2] + step_height

                elif j < frames // 4:
                    # Even push, odd lower
                    base_points[even][0] -= 4 * xy_delta[even][0] / frames
                    base_points[even][1] -= 4 * xy_delta[even][1] / frames
                    base_points[odd][2] -= z_delta * 8

                elif j < 3 * frames // 8:
                    # Even lift, odd push
                    base_points[even][2] += z_delta * 8
                    base_points[odd][0] -= 4 * xy_delta[odd][0] / frames
                    base_points[odd][1] -= 4 * xy_delta[odd][1] / frames

                elif j < 5 * frames // 8:
                    # Even swing forward, odd push
                    base_points[even][0] += 8 * xy_delta[even][0] / frames
                    base_points[even][1] += 8 * xy_delta[even][1] / frames
                    base_points[odd][0] -= 4 * xy_delta[odd][0] / frames
                    base_points[odd][1] -= 4 * xy_delta[odd][1] / frames

                elif j < 3 * frames // 4:
                    # Even lower, odd push
                    base_points[even][2] -= z_delta * 8
                    base_points[odd][0] -= 4 * xy_delta[odd][0] / frames
                    base_points[odd][1] -= 4 * xy_delta[odd][1] / frames

                elif j < 7 * frames // 8:
                    # Even push, odd lift
                    base_points[even][0] -= 4 * xy_delta[even][0] / frames
                    base_points[even][1] -= 4 * xy_delta[even][1] / frames
                    base_points[odd][2] += z_delta * 8

                else:
                    # Even push, odd swing forward
                    base_points[even][0] -= 4 * xy_delta[even][0] / frames
                    base_points[even][1] -= 4 * xy_delta[even][1] / frames
                    base_points[odd][0] += 8 * xy_delta[odd][0] / frames
                    base_points[odd][1] += 8 * xy_delta[odd][1] / frames

            # Update foot positions and servos
            self.foot_positions = np.array(base_points)
            self._apply_body_transform()

            for leg_idx in range(6):
                raw = self._coordinate_to_angle(
                    -self.leg_positions[leg_idx][2],
                    self.leg_positions[leg_idx][0],
                    self.leg_positions[leg_idx][1])

                if leg_idx < 3:
                    self.current_angles[leg_idx][0] = self._clamp(
                        raw[0] + self.calibration_angles[leg_idx][0], 0, 180)
                    self.current_angles[leg_idx][1] = self._clamp(
                        90 - (raw[1] + self.calibration_angles[leg_idx][1]), 0, 180)
                    self.current_angles[leg_idx][2] = self._clamp(
                        raw[2] + self.calibration_angles[leg_idx][2], 0, 180)
                else:
                    self.current_angles[leg_idx][0] = self._clamp(
                        raw[0] + self.calibration_angles[leg_idx][0], 0, 180)
                    self.current_angles[leg_idx][1] = self._clamp(
                        90 + raw[1] + self.calibration_angles[leg_idx][1], 0, 180)
                    self.current_angles[leg_idx][2] = self._clamp(
                        180 - (raw[2] + self.calibration_angles[leg_idx][2]), 0, 180)

            for leg_idx in range(6):
                channels = self.LEG_CHANNELS[leg_idx]
                for k in range(3):
                    self._set_servo_angle(channels[k], self.current_angles[leg_idx][k])

            time.sleep(delay)

    def _wave_gait_step(self, x, y, turn, step_height, cycle_time):
        """
        Wave gait: one leg moves at a time in sequence.
        Ported from fn-hexapod control.py run_gait() gait="2"
        """
        angle_rad = math.radians(turn)
        base_points = copy.deepcopy(self.foot_positions)

        xy_delta = []
        for i in range(6):
            rot_x = (base_points[i][0] * math.cos(angle_rad) +
                     base_points[i][1] * math.sin(angle_rad) - base_points[i][0])
            rot_y = (-base_points[i][0] * math.sin(angle_rad) +
                     base_points[i][1] * math.cos(angle_rad) - base_points[i][1])
            xy_delta.append([rot_x + x, rot_y + y])

        frames = 171
        z_delta = step_height / frames * 18
        delay = cycle_time / frames

        # Leg sequence for wave gait
        leg_order = [5, 2, 1, 0, 3, 4]

        for leg_num in leg_order:
            frames_per_leg = frames // 6

            for j in range(frames_per_leg):
                for k in range(6):
                    if leg_order.index(k) == leg_order.index(leg_num):
                        # This leg is stepping
                        if j < frames_per_leg // 3:
                            base_points[k][2] += z_delta
                        elif j < 2 * frames_per_leg // 3:
                            base_points[k][0] += 30 * xy_delta[k][0] / frames_per_leg
                            base_points[k][1] += 30 * xy_delta[k][1] / frames_per_leg
                        else:
                            base_points[k][2] -= z_delta
                    else:
                        # Other legs push
                        base_points[k][0] -= 2 * xy_delta[k][0] / frames_per_leg
                        base_points[k][1] -= 2 * xy_delta[k][1] / frames_per_leg

                # Update servos
                self.foot_positions = np.array(base_points)
                self._apply_body_transform()

                for leg_idx in range(6):
                    raw = self._coordinate_to_angle(
                        -self.leg_positions[leg_idx][2],
                        self.leg_positions[leg_idx][0],
                        self.leg_positions[leg_idx][1])

                    if leg_idx < 3:
                        self.current_angles[leg_idx][0] = self._clamp(
                            raw[0] + self.calibration_angles[leg_idx][0], 0, 180)
                        self.current_angles[leg_idx][1] = self._clamp(
                            90 - (raw[1] + self.calibration_angles[leg_idx][1]), 0, 180)
                        self.current_angles[leg_idx][2] = self._clamp(
                            raw[2] + self.calibration_angles[leg_idx][2], 0, 180)
                    else:
                        self.current_angles[leg_idx][0] = self._clamp(
                            raw[0] + self.calibration_angles[leg_idx][0], 0, 180)
                        self.current_angles[leg_idx][1] = self._clamp(
                            90 + raw[1] + self.calibration_angles[leg_idx][1], 0, 180)
                        self.current_angles[leg_idx][2] = self._clamp(
                            180 - (raw[2] + self.calibration_angles[leg_idx][2]), 0, 180)

                for leg_idx in range(6):
                    channels = self.LEG_CHANNELS[leg_idx]
                    for m in range(3):
                        self._set_servo_angle(channels[m], self.current_angles[leg_idx][m])

                time.sleep(delay)

    # ===== ROS Callbacks =====

    def cmd_vel_callback(self, msg):
        """Handle velocity commands for walking"""
        if not self.is_initialized or self.is_relaxed:
            return

        # Map Twist to gait parameters
        # ROS linear.x (forward) -> world Y axis
        # ROS linear.y (left) -> world X axis
        # angular.z = rotation

        forward = self._clamp(msg.linear.x * 25, -25, 25)  # mm per step
        strafe = self._clamp(-msg.linear.y * 25, -25, 25)  # negate for correct direction
        turn = self._clamp(msg.angular.z * 12, -15, 15)  # degrees per step

        if abs(forward) < 1 and abs(strafe) < 1 and abs(turn) < 1:
            # Stop moving - return to neutral
            if self.is_walking:
                self.is_walking = False
                self._reset_to_stand()
            return

        self.is_walking = True
        self._tripod_gait_cycle(forward, strafe, turn)

    def body_pose_callback(self, msg):
        """Handle body pose commands"""
        if not self.is_initialized:
            return

        # Convert from ROS Pose (meters) to internal (mm)
        self.set_body_pose(
            x=msg.position.x * 1000,
            y=msg.position.y * 1000,
            z=msg.position.z * 1000,
        )
        # TODO: Convert quaternion to euler for orientation

    def imu_callback(self, msg):
        """Handle IMU data for balance and odometry yaw fusion"""
        # Extract quaternion components
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Convert quaternion to euler angles (proper conversion)
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        self.imu_roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        # Pitch (y-axis rotation)
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            self.imu_pitch = math.degrees(math.copysign(math.pi / 2, sinp))
        else:
            self.imu_pitch = math.degrees(math.asin(sinp))

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        imu_yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self.imu_yaw = math.degrees(imu_yaw_rad)

        # Store raw IMU yaw for reset sync
        self.imu_yaw_rad = imu_yaw_rad

        # Fuse IMU yaw with odometry using complementary filter
        if self.get_parameter('odometry.imu_fusion').value and self.is_walking:
            # During walking, blend IMU yaw with gait-integrated yaw
            alpha = self.get_parameter('odometry.imu_yaw_weight').value
            # Compute difference and apply weighted correction
            yaw_diff = imu_yaw_rad - self.odom_yaw
            # Normalize difference to [-pi, pi]
            while yaw_diff > math.pi:
                yaw_diff -= 2 * math.pi
            while yaw_diff < -math.pi:
                yaw_diff += 2 * math.pi
            # Apply correction
            self.odom_yaw += alpha * yaw_diff
            # Normalize result
            while self.odom_yaw > math.pi:
                self.odom_yaw -= 2 * math.pi
            while self.odom_yaw < -math.pi:
                self.odom_yaw += 2 * math.pi

    def pose_command_callback(self, msg):
        """Handle string pose commands"""
        cmd = msg.data.lower().strip()

        if cmd == 'home':
            # Don't reset odometry when homing via topic (might be during nav)
            self.home(reset_odom=False)
            self.is_initialized = True  # Ready for stand/walk after home
        elif cmd == 'stand':
            if self.is_initialized:
                self.stand()
            else:
                self.get_logger().warn('Not initialized - send home first')
        elif cmd == 'relax':
            self._relax_servos()
        else:
            self.get_logger().warn(f'Unknown pose: {cmd}')

    def initialize_callback(self, request, response):
        """Service: Safe initialization sequence"""
        if self.is_initialized:
            response.success = True
            response.message = 'Already initialized'
            return response

        try:
            self.get_logger().info('=== INITIALIZING ===')
            self.home()
            time.sleep(0.5)
            self.stand()

            self.is_initialized = True
            response.success = True
            response.message = 'Initialized: home -> stand'
            self.get_logger().info('=== INIT COMPLETE ===')
        except Exception as e:
            response.success = False
            response.message = f'Failed: {e}'

        return response

    def enable_balance_callback(self, request, response):
        """Service: Enable/disable IMU balance"""
        self.balance_enabled = request.data

        if self.balance_enabled:
            self.pid_roll.reset()
            self.pid_pitch.reset()
            if self.balance_timer is None:
                period = 1.0 / self.balance_rate
                self.balance_timer = self.create_timer(period, self._balance_loop)
            self.get_logger().info('Balance ENABLED')
        else:
            if self.balance_timer:
                self.balance_timer.cancel()
                self.balance_timer = None
            self.get_logger().info('Balance DISABLED')

        response.success = True
        response.message = f'Balance: {self.balance_enabled}'
        return response

    def reset_odometry_callback(self, request, response):
        """Service: Reset odometry to origin"""
        self._reset_odometry()
        response.success = True
        response.message = 'Odometry reset'
        return response

    # ===== MoveDistance Action Server =====

    def move_distance_goal_callback(self, goal_request):
        """Accept or reject a goal request"""
        if not self.is_initialized:
            self.get_logger().warn('MoveDistance rejected: not initialized')
            return GoalResponse.REJECT
        if self.action_in_progress:
            self.get_logger().warn('MoveDistance rejected: action in progress')
            return GoalResponse.REJECT
        if self.is_relaxed:
            self.get_logger().warn('MoveDistance rejected: servos relaxed')
            return GoalResponse.REJECT

        self.get_logger().info(f'MoveDistance accepted: {goal_request.distance:.3f}m')
        return GoalResponse.ACCEPT

    def move_distance_cancel_callback(self, goal_handle):
        """Accept or reject a cancel request"""
        self.get_logger().info('MoveDistance cancel requested')
        return CancelResponse.ACCEPT

    async def move_distance_execute(self, goal_handle):
        """Execute the MoveDistance action"""
        self.action_in_progress = True
        self.get_logger().info('MoveDistance executing...')

        goal = goal_handle.request
        distance = goal.distance  # meters
        max_speed = goal.max_speed if goal.max_speed > 0 else 0.1  # m/s

        # Record starting position
        start_x = self.odom_x
        start_y = self.odom_y
        start_yaw = self.odom_yaw

        # Calculate direction (forward or backward)
        direction = 1.0 if distance >= 0 else -1.0
        target_distance = abs(distance)

        # Feedback message
        feedback = MoveDistance.Feedback()
        result = MoveDistance.Result()

        # Gait parameters
        step_size = 0.025  # 25mm per cycle
        cycle_time = self.get_parameter('gait.cycle_time').value

        traveled = 0.0
        last_feedback_time = time.time()

        try:
            while traveled < target_distance:
                # Check for cancel
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.actual_distance = traveled
                    result.message = 'Canceled'
                    self.action_in_progress = False
                    self._reset_to_stand()
                    return result

                # Execute one gait cycle
                y_move = step_size * 1000 * direction  # Convert to mm
                self._tripod_gait_cycle(y_move, 0.0, 0.0)

                # Calculate distance traveled from odometry
                dx = self.odom_x - start_x
                dy = self.odom_y - start_y
                traveled = math.sqrt(dx*dx + dy*dy)

                # Publish feedback (max 10Hz)
                now = time.time()
                if now - last_feedback_time >= 0.1:
                    feedback.distance_remaining = max(0.0, target_distance - traveled)
                    feedback.current_speed = step_size / cycle_time
                    goal_handle.publish_feedback(feedback)
                    last_feedback_time = now

            # Goal reached
            goal_handle.succeed()
            result.success = True
            result.actual_distance = traveled
            result.message = f'Traveled {traveled:.3f}m'
            self.get_logger().info(f'MoveDistance complete: {traveled:.3f}m')

        except Exception as e:
            goal_handle.abort()
            result.success = False
            result.actual_distance = traveled
            result.message = f'Error: {e}'
            self.get_logger().error(f'MoveDistance failed: {e}')

        finally:
            self.action_in_progress = False
            self._reset_to_stand()

        return result

    def destroy_node(self):
        self._relax_servos()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HexapodController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
