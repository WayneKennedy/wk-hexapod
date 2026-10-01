#!/usr/bin/env python3
"""
IMU Driver Node for Hexapod Robot
Publishes IMU data from MPU6050 sensor

Based on working implementation in ../fn-hexapod/Code/Server/imu.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math

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

        # Create publisher
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        # Create timer for publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_imu_data)

        self.get_logger().info(f'IMU driver started at {publish_rate} Hz')

    def _to_body(self, x, y):
        return (self.cos_yaw * x - self.sin_yaw * y,
                self.sin_yaw * x + self.cos_yaw * y)

    def publish_imu_data(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        if self.sensor:
            try:
                accel = self.sensor.get_accel_data()
                gyro = self.sensor.get_gyro_data()

                # Chip axes to body axes (imu.mounting_yaw_deg)
                ax, ay = self._to_body(accel['x'], accel['y'])
                gx, gy = self._to_body(gyro['x'], gyro['y'])

                # Linear acceleration (m/s^2)
                msg.linear_acceleration.x = ax
                msg.linear_acceleration.y = ay
                msg.linear_acceleration.z = accel['z']

                # Angular velocity (rad/s) - convert from deg/s
                msg.angular_velocity.x = math.radians(gx)
                msg.angular_velocity.y = math.radians(gy)
                msg.angular_velocity.z = math.radians(gyro['z'])

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
