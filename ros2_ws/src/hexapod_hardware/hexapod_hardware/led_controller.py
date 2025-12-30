#!/usr/bin/env python3
"""
LED Controller Node for Hexapod Robot
Controls WS2812 LEDs via SPI (PCB V2.0 / Pi 5)

Based on reference implementation in reference/spi_ledpixel.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from std_msgs.msg import UInt8MultiArray

# Hardware imports
try:
    import spidev
    import numpy as np
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False


class SPILedPixel:
    """WS2812 LED control via SPI - adapted from reference/spi_ledpixel.py"""

    def __init__(self, led_count, brightness=255, color_order='GRB'):
        self.led_count = led_count
        self.brightness = brightness
        self.color_order = color_order

        # Each LED bit needs 3 SPI bytes (for WS2812 timing)
        # Each LED has 24 bits = 72 SPI bytes
        self.buffer = np.zeros(led_count * 24 * 3, dtype=np.uint8)

        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 2400000  # ~2.4MHz for WS2812 timing

        self.led_data = [[0, 0, 0] for _ in range(led_count)]

    def _encode_byte(self, byte):
        """Encode a byte to WS2812 SPI format"""
        result = []
        for i in range(8):
            if byte & (0x80 >> i):
                result.extend([0b11111100])  # 1 bit
            else:
                result.extend([0b11000000])  # 0 bit
        return result

    def set_led_rgb(self, index, r, g, b):
        """Set LED color"""
        if 0 <= index < self.led_count:
            # Apply brightness
            r = int(r * self.brightness / 255)
            g = int(g * self.brightness / 255)
            b = int(b * self.brightness / 255)

            if self.color_order == 'GRB':
                self.led_data[index] = [g, r, b]
            else:  # RGB
                self.led_data[index] = [r, g, b]

    def show(self):
        """Update LEDs"""
        buffer = []
        for led in self.led_data:
            for byte in led:
                buffer.extend(self._encode_byte(byte))
        self.spi.xfer2(buffer)

    def clear(self):
        """Turn off all LEDs"""
        for i in range(self.led_count):
            self.led_data[i] = [0, 0, 0]
        self.show()

    def close(self):
        """Clean up"""
        self.clear()
        self.spi.close()


class LedController(Node):
    def __init__(self):
        super().__init__('led_controller')

        # Declare parameters
        self.declare_parameter('leds.count', 7)
        self.declare_parameter('leds.brightness', 255)
        self.declare_parameter('leds.type', 'GRB')
        self.declare_parameter('pcb_version', 2)

        # Get parameters
        led_count = self.get_parameter('leds.count').value
        brightness = self.get_parameter('leds.brightness').value
        color_order = self.get_parameter('leds.type').value
        pcb_version = self.get_parameter('pcb_version').value

        # Initialize LED strip
        self.strip = None
        if HARDWARE_AVAILABLE and pcb_version == 2:
            try:
                self.strip = SPILedPixel(led_count, brightness, color_order)
                self.get_logger().info(f'LED strip initialized ({led_count} LEDs via SPI)')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize LED strip: {e}')
        else:
            self.get_logger().warn('Hardware not available or PCB V1 (unsupported on Pi 5)')

        # Create subscribers
        self.color_sub = self.create_subscription(
            ColorRGBA,
            'leds/color',
            self.color_callback,
            10
        )

        self.pattern_sub = self.create_subscription(
            UInt8MultiArray,
            'leds/pattern',
            self.pattern_callback,
            10
        )

        self.get_logger().info('LED controller started')

    def color_callback(self, msg):
        """Set all LEDs to a single color"""
        if not self.strip:
            return

        r = int(msg.r * 255)
        g = int(msg.g * 255)
        b = int(msg.b * 255)

        for i in range(self.strip.led_count):
            self.strip.set_led_rgb(i, r, g, b)
        self.strip.show()

    def pattern_callback(self, msg):
        """Set individual LED colors [r0,g0,b0, r1,g1,b1, ...]"""
        if not self.strip:
            return

        data = list(msg.data)
        for i in range(min(len(data) // 3, self.strip.led_count)):
            r, g, b = data[i*3:(i+1)*3]
            self.strip.set_led_rgb(i, r, g, b)
        self.strip.show()

    def destroy_node(self):
        if self.strip:
            self.strip.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LedController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
