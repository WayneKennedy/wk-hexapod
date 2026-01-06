#!/usr/bin/env python3
"""
Servo Driver Node for Hexapod Robot
Controls PCA9685 PWM servo drivers for leg and head servos

Based on working reference implementation in ../fn-hexapod/Code/Server/:
  - servo.py - PCA9685 servo control
  - pca9685.py - Low-level PWM driver
  - control.py - Gait control and IK
  - home.py, stand.py - Confirmed working position routines
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, String
from std_srvs.srv import Trigger
import math
import os
import time

# Hardware imports
try:
    import smbus
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False

try:
    from gpiozero import OutputDevice
    GPIOZERO_AVAILABLE = True
except ImportError:
    GPIOZERO_AVAILABLE = False


class PCA9685:
    """PCA9685 PWM driver - from fn-hexapod/Code/Server/pca9685.py"""

    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, bus, address=0x40):
        self.bus = smbus.SMBus(bus)
        self.address = address
        self.bus.write_byte_data(self.address, self.MODE1, 0x00)

    def set_pwm_freq(self, freq):
        """Set PWM frequency"""
        prescale = int(25000000.0 / (4096.0 * freq) - 1 + 0.5)
        old_mode = self.bus.read_byte_data(self.address, self.MODE1)
        new_mode = (old_mode & 0x7F) | 0x10  # Sleep mode
        self.bus.write_byte_data(self.address, self.MODE1, new_mode)
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode)
        import time
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode | 0x80)

    def set_pwm(self, channel, on, off):
        """Set PWM on/off values for a channel"""
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel, on & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 1, on >> 8)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 2, off & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 3, off >> 8)

    def set_pwm_off(self, channel):
        """Turn off PWM for a channel (relax servo)"""
        self.set_pwm(channel, 4096, 4096)


class ServoDriver(Node):
    """
    Servo driver following fn-hexapod/Code/Server/servo.py and control.py patterns.

    Channel mapping (from fn-hexapod control.py):
      Channels 0-15  -> PCA9685 at 0x41
      Channels 16-31 -> PCA9685 at 0x40

    Leg channel mapping:
      Leg 1: coxa=15, femur=14, tibia=13
      Leg 2: coxa=12, femur=11, tibia=10
      Leg 3: coxa=9,  femur=8,  tibia=31
      Leg 4: coxa=22, femur=23, tibia=27
      Leg 5: coxa=19, femur=20, tibia=21
      Leg 6: coxa=16, femur=17, tibia=18

    Head (from fn-hexapod server.py CMD_CAMERA): pan=0, tilt=1
    """

    # Link lengths in mm (from fn-hexapod control.py)
    L1 = 33   # coxa
    L2 = 90   # femur
    L3 = 110  # tibia

    # Leg channel mapping from fn-hexapod control.py
    LEG_CHANNELS = [
        [15, 14, 13],  # Leg 1 (index 0): coxa, femur, tibia
        [12, 11, 10],  # Leg 2 (index 1)
        [9, 8, 31],    # Leg 3 (index 2) - tibia on 31!
        [22, 23, 27],  # Leg 4 (index 3) - tibia on 27!
        [19, 20, 21],  # Leg 5 (index 4)
        [16, 17, 18],  # Leg 6 (index 5)
    ]

    def __init__(self):
        super().__init__('servo_driver')

        # Declare parameters
        self.declare_parameter('i2c.bus', 1)
        self.declare_parameter('i2c.pca9685_addr_1', 0x40)
        self.declare_parameter('i2c.pca9685_addr_2', 0x41)
        self.declare_parameter('servos.pwm_frequency', 50)
        self.declare_parameter('servos.min_pulse_us', 500)
        self.declare_parameter('servos.max_pulse_us', 2500)
        self.declare_parameter('servos.power_gpio', 4)
        # Head servo channels (from fn-hexapod server.py CMD_CAMERA handler: channels 0,1)
        self.declare_parameter('servos.head_channels.pan', 0)
        self.declare_parameter('servos.head_channels.tilt', 1)

        # Get parameters
        bus = self.get_parameter('i2c.bus').value
        self.addr_40 = self.get_parameter('i2c.pca9685_addr_1').value  # 0x40
        self.addr_41 = self.get_parameter('i2c.pca9685_addr_2').value  # 0x41
        pwm_freq = self.get_parameter('servos.pwm_frequency').value
        self.min_pulse = self.get_parameter('servos.min_pulse_us').value
        self.max_pulse = self.get_parameter('servos.max_pulse_us').value
        power_gpio = self.get_parameter('servos.power_gpio').value
        self.head_pan_channel = self.get_parameter('servos.head_channels.pan').value
        self.head_tilt_channel = self.get_parameter('servos.head_channels.tilt').value

        # Initialize servo power control (GPIO 4)
        # Reference: control.py:21-22 - OutputDevice(4), off() to enable
        self.servo_power = None
        if GPIOZERO_AVAILABLE:
            try:
                self.servo_power = OutputDevice(power_gpio)
                self.servo_power.off()  # off = servos enabled
                self.get_logger().info(f'Servo power control initialized on GPIO {power_gpio}')
            except Exception as e:
                self.get_logger().warn(f'Failed to initialize servo power GPIO: {e}')

        # Initialize PCA9685 drivers
        # IMPORTANT: Reference servo.py uses swapped addressing:
        #   pwm_41 (0x41) handles channels 0-15
        #   pwm_40 (0x40) handles channels 16-31
        self.pwm_40 = None  # For channels 16-31
        self.pwm_41 = None  # For channels 0-15

        if SMBUS_AVAILABLE:
            try:
                self.pwm_41 = PCA9685(bus, self.addr_41)
                self.pwm_41.set_pwm_freq(pwm_freq)
                self.get_logger().info(f'PCA9685 at 0x{self.addr_41:02x} initialized (channels 0-15)')

                self.pwm_40 = PCA9685(bus, self.addr_40)
                self.pwm_40.set_pwm_freq(pwm_freq)
                self.get_logger().info(f'PCA9685 at 0x{self.addr_40:02x} initialized (channels 16-31)')

            except Exception as e:
                self.get_logger().error(f'Failed to initialize PCA9685: {e}')
        else:
            self.get_logger().warn('smbus not available, running in simulation mode')

        # Load calibration offsets
        self.calibration_positions = self.load_calibration()
        self.calibration_angles = [[0, 0, 0] for _ in range(6)]
        self.current_angles = [[90, 0, 0] for _ in range(6)]
        self.leg_positions = [[140, 0, 0] for _ in range(6)]

        # Calculate calibration angle offsets
        self.calibrate()

        # Create subscribers
        self.joint_sub = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_callback,
            10
        )

        # Leg position commands (x, y, z per leg)
        self.leg_pos_sub = self.create_subscription(
            Float64MultiArray,
            'leg_positions',
            self.leg_positions_callback,
            10
        )

        # Single leg command [leg_id, coxa, femur, tibia] in degrees
        self.leg_sub = self.create_subscription(
            Float64MultiArray,
            'leg_command',
            self.leg_callback,
            10
        )

        # Head pan/tilt command [pan, tilt] in degrees
        self.head_sub = self.create_subscription(
            Float64MultiArray,
            'head_command',
            self.head_callback,
            10
        )

        # Relax servos command
        self.relax_sub = self.create_subscription(
            Bool,
            'servo_relax',
            self.relax_callback,
            10
        )

        # Pose command (home, stand, relax)
        self.pose_sub = self.create_subscription(
            String,
            'pose_command',
            self.pose_callback,
            10
        )

        # Safe initialization service - operator must call this explicitly
        # IMPORTANT: Servos have no position feedback, so operator must ensure
        # robot is in a safe position before calling this service
        self.init_service = self.create_service(
            Trigger,
            'servo_driver/initialize',
            self.initialize_callback
        )

        # Track initialization state
        self.is_initialized = False

        self.get_logger().info('Servo driver started (NOT initialized - call /servo_driver/initialize when safe)')

    def load_calibration(self):
        """Load calibration data from point.txt (fn-hexapod control.py)"""
        # Try multiple locations
        search_paths = [
            '/ros2_ws/config/servo_calibration.txt',
            '/ros2_ws/src/hexapod_hardware/config/servo_calibration.txt',
            'point.txt',
        ]

        for path in search_paths:
            if os.path.exists(path):
                try:
                    with open(path, 'r') as f:
                        lines = f.readlines()
                        data = [list(map(int, line.strip().split('\t'))) for line in lines if line.strip()]
                    self.get_logger().info(f'Loaded calibration from {path}')
                    return data
                except Exception as e:
                    self.get_logger().warn(f'Failed to load calibration from {path}: {e}')

        # Default: no offsets
        self.get_logger().warn('No calibration file found, using defaults')
        return [[140, 0, 0] for _ in range(6)]

    def calibrate(self):
        """Calculate calibration angle offsets (fn-hexapod control.py)"""
        default_pos = [[140, 0, 0] for _ in range(6)]

        for i in range(6):
            # Convert calibration positions to angles
            cal_angles = self.coordinate_to_angle(
                -self.calibration_positions[i][2],
                self.calibration_positions[i][0],
                self.calibration_positions[i][1]
            )
            # Convert default positions to angles
            def_angles = self.coordinate_to_angle(
                -default_pos[i][2],
                default_pos[i][0],
                default_pos[i][1]
            )
            # Calculate offsets
            self.calibration_angles[i][0] = cal_angles[0] - def_angles[0]
            self.calibration_angles[i][1] = cal_angles[1] - def_angles[1]
            self.calibration_angles[i][2] = cal_angles[2] - def_angles[2]

        self.get_logger().info(f'Calibration angles: {self.calibration_angles}')

    def coordinate_to_angle(self, x, y, z):
        """
        Inverse kinematics: convert foot position to servo angles.
        From fn-hexapod control.py (coordinate_to_angle)

        Args:
            x, y, z: foot position relative to leg origin (mm)

        Returns:
            (a, b, c): coxa, femur, tibia angles in degrees
        """
        l1, l2, l3 = self.L1, self.L2, self.L3

        a = math.pi / 2 - math.atan2(z, y)

        x_3 = 0
        x_4 = l1 * math.sin(a)
        x_5 = l1 * math.cos(a)

        l23 = math.sqrt((z - x_5) ** 2 + (y - x_4) ** 2 + (x - x_3) ** 2)

        w = self._clamp((x - x_3) / l23, -1, 1)
        v = self._clamp((l2 * l2 + l23 * l23 - l3 * l3) / (2 * l2 * l23), -1, 1)
        u = self._clamp((l2 ** 2 + l3 ** 2 - l23 ** 2) / (2 * l3 * l2), -1, 1)

        b = math.asin(round(w, 2)) - math.acos(round(v, 2))
        c = math.pi - math.acos(round(u, 2))

        return (round(math.degrees(a)), round(math.degrees(b)), round(math.degrees(c)))

    def _clamp(self, value, min_val, max_val):
        """Clamp value to range"""
        return max(min_val, min(max_val, value))

    def set_servo_angle(self, channel, angle):
        """
        Set servo angle on a channel (0-31).
        From fn-hexapod servo.py (set_servo_angle)

        Channels 0-15 go to pwm_41 (0x41)
        Channels 16-31 go to pwm_40 (0x40)
        """
        angle = self._clamp(angle, 0, 180)

        # Map angle to pulse width, then to PWM value
        duty_cycle = self._map_value(angle, 0, 180, self.min_pulse, self.max_pulse)
        duty_cycle = self._map_value(duty_cycle, 0, 20000, 0, 4095)
        pwm_value = int(duty_cycle)

        if channel < 16:
            if self.pwm_41:
                self.pwm_41.set_pwm(channel, 0, pwm_value)
        elif channel < 32:
            if self.pwm_40:
                self.pwm_40.set_pwm(channel - 16, 0, pwm_value)

    def _map_value(self, value, from_low, from_high, to_low, to_high):
        """Map value from one range to another"""
        return (to_high - to_low) * (value - from_low) / (from_high - from_low) + to_low

    def set_leg_angles(self):
        """
        Apply current leg positions to servos with calibration.
        From fn-hexapod control.py (set_angle method)
        """
        for i in range(6):
            # Convert position to raw angles
            raw = self.coordinate_to_angle(
                -self.leg_positions[i][2],
                self.leg_positions[i][0],
                self.leg_positions[i][1]
            )
            self.current_angles[i] = list(raw)

        # Apply calibration and servo direction corrections
        # Legs 0-2 (right side) and legs 3-5 (left side) have different corrections
        for i in range(3):
            # Right side legs (0, 1, 2)
            a0 = self._clamp(self.current_angles[i][0] + self.calibration_angles[i][0], 0, 180)
            a1 = self._clamp(90 - (self.current_angles[i][1] + self.calibration_angles[i][1]), 0, 180)
            a2 = self._clamp(self.current_angles[i][2] + self.calibration_angles[i][2], 0, 180)

            # Left side legs (3, 4, 5)
            b0 = self._clamp(self.current_angles[i + 3][0] + self.calibration_angles[i + 3][0], 0, 180)
            b1 = self._clamp(90 + self.current_angles[i + 3][1] + self.calibration_angles[i + 3][1], 0, 180)
            b2 = self._clamp(180 - (self.current_angles[i + 3][2] + self.calibration_angles[i + 3][2]), 0, 180)

            self.current_angles[i] = [a0, a1, a2]
            self.current_angles[i + 3] = [b0, b1, b2]

        # Send to servos using correct channel mapping
        for leg_idx in range(6):
            channels = self.LEG_CHANNELS[leg_idx]
            angles = self.current_angles[leg_idx]
            for joint_idx in range(3):
                self.set_servo_angle(channels[joint_idx], angles[joint_idx])

    def joint_callback(self, msg):
        """Handle raw joint position commands (20 angles in degrees)"""
        if len(msg.data) < 20:
            self.get_logger().warn(f'Expected 20 joint values, got {len(msg.data)}')
            return

        # Map to leg channels
        # msg.data layout: [leg0_coxa, leg0_femur, leg0_tibia, leg1_coxa, ...]
        for leg_idx in range(6):
            channels = self.LEG_CHANNELS[leg_idx]
            for joint_idx in range(3):
                angle = msg.data[leg_idx * 3 + joint_idx]
                self.set_servo_angle(channels[joint_idx], angle)

        # Head servos (last 2 values)
        if len(msg.data) >= 20:
            self.set_servo_angle(self.head_pan_channel, msg.data[18])
            self.set_servo_angle(self.head_tilt_channel, msg.data[19])

    def leg_positions_callback(self, msg):
        """Handle leg position commands (18 values: x,y,z per leg)"""
        if len(msg.data) < 18:
            self.get_logger().warn(f'Expected 18 position values, got {len(msg.data)}')
            return

        for i in range(6):
            self.leg_positions[i][0] = msg.data[i * 3]
            self.leg_positions[i][1] = msg.data[i * 3 + 1]
            self.leg_positions[i][2] = msg.data[i * 3 + 2]

        self.set_leg_angles()

    def leg_callback(self, msg):
        """Handle single leg command [leg_id, coxa, femur, tibia] in degrees"""
        if len(msg.data) < 4:
            return

        leg_id = int(msg.data[0])
        if leg_id < 0 or leg_id >= 6:
            self.get_logger().warn(f'Invalid leg_id: {leg_id}')
            return

        channels = self.LEG_CHANNELS[leg_id]
        for joint_idx in range(3):
            self.set_servo_angle(channels[joint_idx], msg.data[1 + joint_idx])

    def head_callback(self, msg):
        """Handle head pan/tilt command [pan, tilt] in degrees"""
        if len(msg.data) < 2:
            return

        self.set_servo_angle(self.head_pan_channel, msg.data[0])
        self.set_servo_angle(self.head_tilt_channel, msg.data[1])

    def relax_callback(self, msg):
        """Relax all servos (disable PWM)"""
        if msg.data:
            self.relax_servos()
        else:
            # Re-enable by setting current positions
            self.set_leg_angles()

    def relax_servos(self):
        """Turn off all servo PWM signals (fn-hexapod servo.py relax method)"""
        if self.pwm_40:
            for i in range(16):
                self.pwm_40.set_pwm_off(i)
        if self.pwm_41:
            for i in range(16):
                self.pwm_41.set_pwm_off(i)
        self.get_logger().info('Servos relaxed')

    def home(self):
        """
        Move all legs to calibrated home position.
        From fn-hexapod home.py - leg_position = [140, 0, 0]
        """
        self.get_logger().info('Moving to HOME position...')
        for i in range(6):
            self.leg_positions[i] = [140, 0, 0]
        self.set_leg_angles()
        self.is_initialized = True  # Ready for stand after home
        self.get_logger().info('HOME position set')

    def stand(self, height=30, duration=1.0, steps=50):
        """
        Smoothly raise body to standing height.
        From fn-hexapod stand.py - raises body by lowering z coordinate.

        Args:
            height: How high to raise body in mm (default 30)
            duration: Time for smooth transition in seconds
            steps: Number of interpolation steps
        """
        self.get_logger().info(f'Standing up (raising body {height}mm)...')

        # Start position (home)
        start_z = 0
        # End position (standing - negative z raises the body)
        end_z = -height

        for step in range(steps + 1):
            t = step / steps
            # Ease in-out for smoother motion
            t = t * t * (3 - 2 * t)

            current_z = start_z + t * (end_z - start_z)

            for i in range(6):
                self.leg_positions[i] = [140, 0, current_z]

            self.set_leg_angles()
            time.sleep(duration / steps)

        self.get_logger().info('STAND position set')

    def pose_callback(self, msg):
        """Handle pose commands - only relax (home/stand via controller's joint_commands)"""
        command = msg.data.lower().strip()

        if command == 'relax':
            self.relax_servos()
        elif command in ['home', 'stand']:
            # Handled by hexapod_controller which publishes to joint_commands
            pass
        else:
            self.get_logger().warn(f'Unknown pose command: {command}')

    def initialize_callback(self, request, response):
        """
        Service callback for safe initialization.
        IMPORTANT: Operator must ensure robot is in safe position before calling.
        Servos have no position feedback - this will move all legs!
        """
        if self.is_initialized:
            response.success = True
            response.message = 'Already initialized'
            return response

        try:
            self.get_logger().info('=== INITIALIZING - Moving to home then stand ===')

            # Step 1: Move to home position
            self.home()
            time.sleep(0.5)

            # Step 2: Stand up
            self.stand()

            self.is_initialized = True
            response.success = True
            response.message = 'Initialized: home -> stand complete'
            self.get_logger().info('=== INITIALIZATION COMPLETE ===')

        except Exception as e:
            response.success = False
            response.message = f'Initialization failed: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def destroy_node(self):
        # Relax servos on shutdown
        self.relax_servos()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()

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
