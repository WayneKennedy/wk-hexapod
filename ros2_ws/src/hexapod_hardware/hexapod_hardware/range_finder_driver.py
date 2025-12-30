#!/usr/bin/env python3
"""
Range Finder Driver Node for Hexapod Robot
Publishes distance data from HC-SR04 ultrasonic sensor

Based on reference implementation in reference/ultrasonic.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import warnings
import math

# Hardware imports
try:
    # Suppress gpiozero pin factory fallback warnings (noisy but not errors)
    warnings.filterwarnings("ignore", message=".*Falling back.*")
    from gpiozero import DistanceSensor, PWMSoftwareFallback, DistanceSensorNoEcho
    from gpiozero.exc import BadPinFactory
    warnings.filterwarnings("ignore", category=DistanceSensorNoEcho)
    warnings.filterwarnings("ignore", category=PWMSoftwareFallback)
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    BadPinFactory = Exception  # Placeholder for non-GPIO environments


class RangeFinderDriver(Node):
    def __init__(self):
        super().__init__('range_finder_driver')

        # Declare parameters
        self.declare_parameter('range_finder.publish_rate', 10.0)
        self.declare_parameter('range_finder.trigger_pin', 27)
        self.declare_parameter('range_finder.echo_pin', 22)
        self.declare_parameter('range_finder.max_range', 3.0)
        self.declare_parameter('range_finder.min_range', 0.02)
        self.declare_parameter('range_finder.field_of_view', 0.2617)  # ~15 degrees

        # Get parameters
        publish_rate = self.get_parameter('range_finder.publish_rate').value
        trigger_pin = self.get_parameter('range_finder.trigger_pin').value
        echo_pin = self.get_parameter('range_finder.echo_pin').value
        self.max_range = self.get_parameter('range_finder.max_range').value
        self.min_range = self.get_parameter('range_finder.min_range').value
        self.field_of_view = self.get_parameter('range_finder.field_of_view').value

        # Initialize sensor
        self.sensor = None
        if HARDWARE_AVAILABLE:
            try:
                self.sensor = DistanceSensor(
                    echo=echo_pin,
                    trigger=trigger_pin,
                    max_distance=self.max_range
                )
                self.get_logger().info(
                    f'Ultrasonic sensor initialized (trigger={trigger_pin}, echo={echo_pin})'
                )
            except BadPinFactory:
                self.get_logger().warn('No GPIO pin factory available, running in simulation mode')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize ultrasonic sensor: {e}')
        else:
            self.get_logger().warn('gpiozero not installed, running in simulation mode')

        # Simulation state
        self.sim_distance = 1.0
        self.sim_direction = 0.1

        # Create publisher
        self.range_pub = self.create_publisher(Range, 'range_finder/range', 10)

        # Create timer for publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_range_data)

        self.get_logger().info(f'Range finder driver started at {publish_rate} Hz')

    def publish_range_data(self):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_link'

        # Sensor metadata
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.field_of_view
        msg.min_range = self.min_range
        msg.max_range = self.max_range

        if self.sensor:
            try:
                # gpiozero returns distance in meters
                distance = self.sensor.distance
                msg.range = float(distance)
            except Exception as e:
                self.get_logger().warn(f'Failed to read range: {e}')
                return
        else:
            # Simulation mode - oscillate between min and max
            self.sim_distance += self.sim_direction
            if self.sim_distance >= self.max_range or self.sim_distance <= self.min_range:
                self.sim_direction *= -1
            msg.range = self.sim_distance

        self.range_pub.publish(msg)

    def destroy_node(self):
        if self.sensor:
            self.sensor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RangeFinderDriver()

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
