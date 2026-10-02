#!/usr/bin/env python3
"""
IMU Driver Node for Hexapod Robot
Publishes IMU data from MPU6050 sensor

Based on working implementation in ../fn-hexapod/Code/Server/imu.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
import math
import statistics

# Hardware imports
try:
    from mpu6050 import mpu6050
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False


class ImuDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')

        # Declare parameters
        self.declare_parameter('i2c.bus', 1)
        self.declare_parameter('i2c.mpu6050_addr', 0x68)
        self.declare_parameter('imu.publish_rate', 100.0)
        self.declare_parameter('imu.accel_range', 2)
        self.declare_parameter('imu.gyro_range', 250)
        # Rotation about z, degrees, that turns a vector in the chip's axes into
        # the body's (x forward, y left). -90 since 2026-10-01: tilted by hand,
        # the chip's y axis pointed forward and its x axis to the robot's right
        # (test-log.md, OQ-13). Everything downstream reads body axes.
        self.declare_parameter('imu.mounting_yaw_deg', -90.0)
        # Gyro bias: the mean of this many readings taken while the robot is
        # still, at start-up and on /imu/calibrate_gyro, is subtracted from
        # every rate. Uncorrected the yaw drifted 0.2 deg/s at rest (OQ-36).
        # Readings are not published while the mean is being taken.
        self.declare_parameter('imu.gyro_bias_samples', 200)
        # A spread above this (deg/s, standard deviation on any axis) means
        # the robot was moving: the bias is left as it was.
        self.declare_parameter('imu.gyro_bias_max_std_dps', 1.0)

        # Get parameters
        bus = self.get_parameter('i2c.bus').value
        addr = self.get_parameter('i2c.mpu6050_addr').value
        publish_rate = self.get_parameter('imu.publish_rate').value
        yaw = math.radians(self.get_parameter('imu.mounting_yaw_deg').value)
        self.cos_yaw, self.sin_yaw = math.cos(yaw), math.sin(yaw)

        # Initialize sensor
        if HARDWARE_AVAILABLE:
            try:
                self.sensor = mpu6050(address=addr, bus=bus)
                accel_range = self.get_parameter('imu.accel_range').value
                gyro_range = self.get_parameter('imu.gyro_range').value

                # Set ranges
                if accel_range == 2:
                    self.sensor.set_accel_range(mpu6050.ACCEL_RANGE_2G)
                elif accel_range == 4:
                    self.sensor.set_accel_range(mpu6050.ACCEL_RANGE_4G)

                if gyro_range == 250:
                    self.sensor.set_gyro_range(mpu6050.GYRO_RANGE_250DEG)
                elif gyro_range == 500:
                    self.sensor.set_gyro_range(mpu6050.GYRO_RANGE_500DEG)

                self.get_logger().info(f'MPU6050 initialized at 0x{addr:02x}')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize MPU6050: {e}')
                self.sensor = None
        else:
            self.get_logger().warn('Hardware not available, running in simulation mode')
            self.sensor = None

        self.gyro_bias = [0.0, 0.0, 0.0]   # deg/s, chip axes
        self._bias_samples = []
        self._collecting = self.sensor is not None
        self.calibrate_srv = self.create_service(
            Trigger, 'imu/calibrate_gyro', self.calibrate_callback)

        # Create publisher
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        # Create timer for publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_imu_data)

        self.get_logger().info(f'IMU driver started at {publish_rate} Hz')

    def _to_body(self, x, y):
        return (self.cos_yaw * x - self.sin_yaw * y,
                self.sin_yaw * x + self.cos_yaw * y)

    def calibrate_callback(self, request, response):
        """Take the gyro bias again; the robot must be still."""
        self._bias_samples = []
        self._collecting = self.sensor is not None
        response.success = self._collecting
        response.message = ('Taking the gyro bias' if self._collecting
                            else 'No sensor')
        return response

    def _collect_bias(self, gyro):
        """Accumulate one reading; set the bias once enough are in."""
        self._bias_samples.append((gyro['x'], gyro['y'], gyro['z']))
        n = self.get_parameter('imu.gyro_bias_samples').value
        if len(self._bias_samples) < n:
            return
        self._collecting = False
        axes = list(zip(*self._bias_samples))
        std = [statistics.pstdev(a) for a in axes]
        limit = self.get_parameter('imu.gyro_bias_max_std_dps').value
        if max(std) > limit:
            self.get_logger().warn(
                f'Gyro bias not taken: the robot moved (std {std[0]:.2f}, {std[1]:.2f}, '
                f'{std[2]:.2f} deg/s over {n} readings); keeping '
                f'{self.gyro_bias[0]:+.2f}, {self.gyro_bias[1]:+.2f}, {self.gyro_bias[2]:+.2f}')
            return
        self.gyro_bias = [statistics.fmean(a) for a in axes]
        self.get_logger().info(
            f'Gyro bias {self.gyro_bias[0]:+.2f}, {self.gyro_bias[1]:+.2f}, '
            f'{self.gyro_bias[2]:+.2f} deg/s from {n} readings (std up to {max(std):.2f})')

    def publish_imu_data(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        if self.sensor:
            try:
                accel = self.sensor.get_accel_data()
                gyro = self.sensor.get_gyro_data()

                if self._collecting:
                    self._collect_bias(gyro)
                    return

                # Chip axes to body axes (imu.mounting_yaw_deg), gyro bias off
                ax, ay = self._to_body(accel['x'], accel['y'])
                gx, gy = self._to_body(gyro['x'] - self.gyro_bias[0],
                                       gyro['y'] - self.gyro_bias[1])
                gz = gyro['z'] - self.gyro_bias[2]

                # Linear acceleration (m/s^2)
                msg.linear_acceleration.x = ax
                msg.linear_acceleration.y = ay
                msg.linear_acceleration.z = accel['z']

                # Angular velocity (rad/s) - convert from deg/s
                msg.angular_velocity.x = math.radians(gx)
                msg.angular_velocity.y = math.radians(gy)
                msg.angular_velocity.z = math.radians(gz)

                # Orientation not provided by raw sensor
                msg.orientation_covariance[0] = -1  # Indicates no orientation data

            except Exception as e:
                self.get_logger().warn(f'Failed to read IMU: {e}')
                return
        else:
            # Simulation mode - publish zeros
            msg.orientation_covariance[0] = -1

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
