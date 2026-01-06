#!/usr/bin/env python3
"""
Buzzer Controller Node for Hexapod Robot
Controls the onboard buzzer via GPIO

Based on working implementation in ../fn-hexapod/Code/Server/buzzer.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# Hardware imports
try:
    from gpiozero import OutputDevice
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False


class BuzzerController(Node):
    def __init__(self):
        super().__init__('buzzer_controller')

        # Declare parameters
        self.declare_parameter('buzzer.gpio_pin', 17)
        self.declare_parameter('buzzer.default_beep_duration', 0.2)

        # Get parameters
        gpio_pin = self.get_parameter('buzzer.gpio_pin').value
        self.default_beep_duration = self.get_parameter('buzzer.default_beep_duration').value

        # Initialize buzzer
        self.buzzer = None
        if HARDWARE_AVAILABLE:
            try:
                self.buzzer = OutputDevice(gpio_pin)
                self.get_logger().info(f'Buzzer initialized on GPIO {gpio_pin}')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize buzzer: {e}')
        else:
            self.get_logger().warn('gpiozero not available - buzzer disabled')

        # Timer for auto-off after beep
        self.beep_timer = None

        # Create subscriber for on/off control
        self.state_sub = self.create_subscription(
            Bool,
            'buzzer/state',
            self.state_callback,
            10
        )

        # Create service for single beep
        self.beep_srv = self.create_service(
            Trigger,
            'buzzer/beep',
            self.beep_callback
        )

        self.get_logger().info('Buzzer controller started')

    def state_callback(self, msg: Bool):
        """Set buzzer on or off"""
        if not self.buzzer:
            return

        if msg.data:
            self.buzzer.on()
            self.get_logger().debug('Buzzer ON')
        else:
            self.buzzer.off()
            self.get_logger().debug('Buzzer OFF')

    def beep_callback(self, request, response):
        """Single beep service - turns buzzer on then off after default duration"""
        if not self.buzzer:
            response.success = False
            response.message = 'Buzzer hardware not available'
            return response

        # Cancel any existing beep timer
        if self.beep_timer:
            self.beep_timer.cancel()

        # Turn on buzzer
        self.buzzer.on()

        # Schedule turn off
        self.beep_timer = self.create_timer(
            self.default_beep_duration,
            self._beep_off_callback
        )

        response.success = True
        response.message = f'Beep for {self.default_beep_duration}s'
        return response

    def _beep_off_callback(self):
        """Turn off buzzer after beep duration"""
        if self.buzzer:
            self.buzzer.off()
        if self.beep_timer:
            self.beep_timer.cancel()
            self.beep_timer = None

    def destroy_node(self):
        if self.buzzer:
            self.buzzer.off()
            self.buzzer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BuzzerController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
