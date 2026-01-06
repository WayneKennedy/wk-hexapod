#!/usr/bin/env python3
"""
Power Indicator Node for Hexapod Robot
Monitors battery voltages and updates LED indicators

LED Mapping:
- LOAD battery (servos) -> Left LEDs (L1, L2, L3)
- CTRL battery (Pi)     -> Right LEDs (R1, R2, R3)
- REAR LED              -> Off by default

Color Coding:
- Green:  Good power (>= voltage_good)
- Yellow: Warning (>= voltage_warn and < voltage_good)
- Red:    Critical (< voltage_warn)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


class PowerIndicator(Node):
    def __init__(self):
        super().__init__('power_indicator')

        # Declare parameters
        self.declare_parameter('power_indicator.update_interval', 10.0)  # seconds
        self.declare_parameter('power_indicator.voltage_good', 7.0)      # green threshold
        self.declare_parameter('power_indicator.voltage_warn', 6.5)      # yellow->red threshold
        self.declare_parameter('power_indicator.voltage_usb', 0.5)       # below = USB/no battery

        # Get parameters
        self.update_interval = self.get_parameter('power_indicator.update_interval').value
        self.voltage_good = self.get_parameter('power_indicator.voltage_good').value
        self.voltage_warn = self.get_parameter('power_indicator.voltage_warn').value
        self.voltage_usb = self.get_parameter('power_indicator.voltage_usb').value

        # Color definitions (R, G, B)
        self.COLOR_GREEN = (0, 255, 0)
        self.COLOR_YELLOW = (255, 255, 0)
        self.COLOR_RED = (255, 0, 0)
        self.COLOR_BLUE = (0, 0, 255)    # USB power mode
        self.COLOR_OFF = (0, 0, 0)

        # Latest battery readings
        self.load_voltage = None
        self.ctrl_voltage = None

        # Track last published colors to avoid redundant updates
        self.last_load_color = None
        self.last_ctrl_color = None

        # Subscribe to battery voltages
        self.battery_sub = self.create_subscription(
            Float32MultiArray,
            'battery/voltages',
            self.battery_callback,
            10
        )

        # Publisher for LED zone commands
        self.led_pub = self.create_publisher(String, 'leds/zone', 10)

        # Timer for periodic LED updates
        self.update_timer = self.create_timer(self.update_interval, self.update_leds)

        # Initial state: turn off rear LED
        self.publish_zone('rear', self.COLOR_OFF)

        self.get_logger().info(
            f'Power indicator started (update every {self.update_interval}s, '
            f'blue < {self.voltage_usb}V, green >= {self.voltage_good}V, '
            f'yellow >= {self.voltage_warn}V, red < {self.voltage_warn}V)'
        )

    def battery_callback(self, msg):
        """Store latest battery readings [LOAD, CTRL]"""
        if len(msg.data) >= 2:
            self.load_voltage = msg.data[0]
            self.ctrl_voltage = msg.data[1]

    def voltage_to_color(self, voltage):
        """Convert voltage to indicator color"""
        if voltage is None:
            return self.COLOR_OFF
        elif voltage < self.voltage_usb:
            return self.COLOR_BLUE      # USB power / no battery
        elif voltage >= self.voltage_good:
            return self.COLOR_GREEN
        elif voltage >= self.voltage_warn:
            return self.COLOR_YELLOW
        else:
            return self.COLOR_RED

    def publish_zone(self, zone, color):
        """Publish LED zone command"""
        r, g, b = color
        msg = String()
        msg.data = f'{zone}:{r},{g},{b}'
        self.led_pub.publish(msg)

    def update_leds(self):
        """Update LED indicators based on current battery state"""
        # Determine colors
        load_color = self.voltage_to_color(self.load_voltage)
        ctrl_color = self.voltage_to_color(self.ctrl_voltage)

        # Update LOAD indicator (left LEDs) if changed
        if load_color != self.last_load_color:
            self.publish_zone('load', load_color)
            self.last_load_color = load_color
            if self.load_voltage is not None:
                self.get_logger().info(
                    f'LOAD: {self.load_voltage:.2f}V -> {self._color_name(load_color)}'
                )

        # Update CTRL indicator (right LEDs) if changed
        if ctrl_color != self.last_ctrl_color:
            self.publish_zone('ctrl', ctrl_color)
            self.last_ctrl_color = ctrl_color
            if self.ctrl_voltage is not None:
                self.get_logger().info(
                    f'CTRL: {self.ctrl_voltage:.2f}V -> {self._color_name(ctrl_color)}'
                )

    def _color_name(self, color):
        """Get human-readable color name"""
        if color == self.COLOR_GREEN:
            return 'GREEN'
        elif color == self.COLOR_YELLOW:
            return 'YELLOW'
        elif color == self.COLOR_RED:
            return 'RED'
        elif color == self.COLOR_BLUE:
            return 'BLUE (USB)'
        else:
            return 'OFF'


def main(args=None):
    rclpy.init(args=args)
    node = PowerIndicator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
