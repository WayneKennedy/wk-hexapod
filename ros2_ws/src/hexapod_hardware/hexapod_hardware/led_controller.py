#!/usr/bin/env python3
"""
LED Controller Node for Hexapod Robot
Controls WS2812 LEDs via SPI (PCB V2.0 / Pi 5)

Based on working implementation in ../fn-hexapod/Code/Server/spi_ledpixel.py

LED Layout (viewed from above, front is camera gap):

         (FRONT - camera gap)
     L1[6]               R1[0]
     L2[5]               R2[1]
     L3[4]               R3[2]
            REAR[3]

Indices are clockwise from front-right.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from std_msgs.msg import UInt8MultiArray


# =============================================================================
# LED Position Constants
# =============================================================================

# Individual LED positions (index into LED strip)
class LedPosition:
    R1 = 0      # Front right
    R2 = 1      # Mid right
    R3 = 2      # Rear right
    REAR = 3    # Center rear
    L3 = 4      # Rear left
    L2 = 5      # Mid left
    L1 = 6      # Front left


# Zone definitions (lists of LED indices)
class LedZone:
    # Individual positions as single-element lists for consistency
    R1 = [LedPosition.R1]
    R2 = [LedPosition.R2]
    R3 = [LedPosition.R3]
    REAR = [LedPosition.REAR]
    L3 = [LedPosition.L3]
    L2 = [LedPosition.L2]
    L1 = [LedPosition.L1]

    # Side zones
    RIGHT = [LedPosition.R1, LedPosition.R2, LedPosition.R3]
    LEFT = [LedPosition.L1, LedPosition.L2, LedPosition.L3]

    # Position zones
    FRONT = [LedPosition.R1, LedPosition.L1]
    BACK = [LedPosition.R3, LedPosition.REAR, LedPosition.L3]
    MID = [LedPosition.R2, LedPosition.L2]

    # All LEDs
    ALL = [0, 1, 2, 3, 4, 5, 6]


# Zone name to indices mapping
ZONE_MAP = {
    # Individual positions
    'r1': LedZone.R1,
    'r2': LedZone.R2,
    'r3': LedZone.R3,
    'rear': LedZone.REAR,
    'l3': LedZone.L3,
    'l2': LedZone.L2,
    'l1': LedZone.L1,
    # Side zones
    'right': LedZone.RIGHT,
    'left': LedZone.LEFT,
    # Position zones
    'front': LedZone.FRONT,
    'back': LedZone.BACK,
    'mid': LedZone.MID,
    # All LEDs
    'all': LedZone.ALL,
    # Power indicator aliases (match physical switch labels)
    'load': LedZone.LEFT,   # LOAD battery -> left LEDs
    'ctrl': LedZone.RIGHT,  # CTRL battery -> right LEDs
}

# Hardware imports
try:
    import spidev
    import numpy as np
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False


class SPILedPixel:
    """WS2812 LED control via SPI - from fn-hexapod/Code/Server/spi_ledpixel.py"""

    def __init__(self, led_count, brightness=255, color_order='GRB', bus=0, device=0):
        self.led_count = led_count
        self.brightness = brightness
        self.bus = bus

        # Set color order offsets (GRB is default for WS2812)
        self._set_led_type(color_order)

        # Color data: 3 bytes per LED
        self.led_color = [0] * (led_count * 3)
        self.led_original_color = [0] * (led_count * 3)

        # Initialize SPI
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.mode = 0

        # Clear LEDs on init
        self.clear()

    def _set_led_type(self, rgb_type):
        """Set LED color sequence offsets"""
        led_type = ['RGB', 'RBG', 'GRB', 'GBR', 'BRG', 'BGR']
        led_type_offset = [0x06, 0x09, 0x12, 0x21, 0x18, 0x24]
        try:
            index = led_type.index(rgb_type)
            self.led_red_offset = (led_type_offset[index] >> 4) & 0x03
            self.led_green_offset = (led_type_offset[index] >> 2) & 0x03
            self.led_blue_offset = (led_type_offset[index] >> 0) & 0x03
        except ValueError:
            # Default to GRB
            self.led_red_offset = 1
            self.led_green_offset = 0
            self.led_blue_offset = 2

    def set_led_rgb(self, index, r, g, b):
        """Set LED color with brightness applied"""
        if 0 <= index < self.led_count:
            # Apply brightness
            p = [0, 0, 0]
            p[self.led_red_offset] = round(r * self.brightness / 255)
            p[self.led_green_offset] = round(g * self.brightness / 255)
            p[self.led_blue_offset] = round(b * self.brightness / 255)

            # Store original colors
            self.led_original_color[index * 3 + self.led_red_offset] = r
            self.led_original_color[index * 3 + self.led_green_offset] = g
            self.led_original_color[index * 3 + self.led_blue_offset] = b

            # Store brightness-adjusted colors
            for i in range(3):
                self.led_color[index * 3 + i] = p[i]

    def show(self):
        """Update LEDs using WS2812 protocol over SPI"""
        # Convert color data to numpy array
        d = np.array(self.led_color, dtype=np.uint8).ravel()

        # Each color bit becomes 8 SPI bits for proper WS2812 timing
        tx = np.zeros(len(d) * 8, dtype=np.uint8)
        for ibit in range(8):
            # T0H=1, T0L=7, T1H=5, T1L=3
            # 0xF8 (11111000) for T1, 0x80 (10000000) for T0
            tx[7 - ibit::8] = ((d >> ibit) & 1) * 0x78 + 0x80

        # Send at 6.4MHz for bus 0 (8 bits / 1.25us per WS2812 bit)
        if self.bus == 0:
            self.spi.xfer(tx.tolist(), int(8 / 1.25e-6))
        else:
            self.spi.xfer(tx.tolist(), int(8 / 1.0e-6))

    def clear(self):
        """Turn off all LEDs"""
        self.led_color = [0] * (self.led_count * 3)
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

        # Zone-based control: "zone:r,g,b" e.g., "left:255,0,0" or "front:0,255,0"
        self.zone_sub = self.create_subscription(
            String,
            'leds/zone',
            self.zone_callback,
            10
        )

        self.get_logger().info('LED controller started')
        self.get_logger().info('Zones: r1, r2, r3, rear, l1, l2, l3, left, right, front, back, mid, all')

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

    def zone_callback(self, msg):
        """Set LEDs by zone name. Format: 'zone:r,g,b' e.g., 'left:255,0,0'"""
        if not self.strip:
            return

        try:
            # Parse message: "zone:r,g,b"
            parts = msg.data.lower().strip().split(':')
            if len(parts) != 2:
                self.get_logger().warn(f"Invalid zone format: {msg.data}. Use 'zone:r,g,b'")
                return

            zone_name = parts[0]
            rgb_parts = parts[1].split(',')

            if len(rgb_parts) != 3:
                self.get_logger().warn(f"Invalid RGB format: {parts[1]}. Use 'r,g,b'")
                return

            r, g, b = int(rgb_parts[0]), int(rgb_parts[1]), int(rgb_parts[2])

            # Clamp values
            r = max(0, min(255, r))
            g = max(0, min(255, g))
            b = max(0, min(255, b))

            # Get zone indices
            if zone_name not in ZONE_MAP:
                self.get_logger().warn(f"Unknown zone: {zone_name}. Valid: {', '.join(ZONE_MAP.keys())}")
                return

            indices = ZONE_MAP[zone_name]

            # Set LEDs in zone
            for idx in indices:
                self.strip.set_led_rgb(idx, r, g, b)
            self.strip.show()

            self.get_logger().debug(f"Set zone '{zone_name}' to RGB({r},{g},{b})")

        except ValueError as e:
            self.get_logger().warn(f"Parse error: {e}. Format: 'zone:r,g,b'")

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
