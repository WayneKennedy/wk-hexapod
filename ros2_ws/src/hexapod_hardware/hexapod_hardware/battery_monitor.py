#!/usr/bin/env python3
"""
Battery Monitor Node for Hexapod Robot
Monitors dual battery voltages via ADS7830 ADC

Based on reference implementation in reference/adc.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# Hardware imports
try:
    import smbus
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False


class ADS7830:
    """ADS7830 ADC driver - adapted from reference/adc.py"""

    def __init__(self, bus, address=0x48):
        self.bus = smbus.SMBus(bus)
        self.address = address

    def read_channel(self, channel):
        """Read single-ended ADC channel (0-7)"""
        # Command byte: single-ended, channel select
        command = 0x84 | ((channel & 0x04) << 4) | ((channel & 0x03) << 2)
        self.bus.write_byte(self.address, command)
        return self.bus.read_byte(self.address)

    def read_voltage(self, channel, vref=3.3, divider=3.0):
        """Read voltage with scaling for voltage divider"""
        raw = self.read_channel(channel)
        voltage = (raw / 255.0) * vref * divider
        return voltage


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        # Declare parameters
        self.declare_parameter('i2c.bus', 1)
        self.declare_parameter('i2c.ads7830_addr', 0x48)
        self.declare_parameter('battery.publish_rate', 1.0)
        self.declare_parameter('battery.voltage_divider', 3.0)
        self.declare_parameter('battery.low_voltage_warn', 6.5)
        self.declare_parameter('battery.channels', [0, 4])

        # Get parameters
        bus = self.get_parameter('i2c.bus').value
        addr = self.get_parameter('i2c.ads7830_addr').value
        publish_rate = self.get_parameter('battery.publish_rate').value
        self.voltage_divider = self.get_parameter('battery.voltage_divider').value
        self.low_voltage = self.get_parameter('battery.low_voltage_warn').value
        self.channels = self.get_parameter('battery.channels').value

        # Initialize ADC
        self.adc = None
        if HARDWARE_AVAILABLE:
            try:
                self.adc = ADS7830(bus, addr)
                self.get_logger().info(f'ADS7830 initialized at 0x{addr:02x}')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize ADS7830: {e}')
        else:
            self.get_logger().warn('Hardware not available, running in simulation mode')

        # Create publishers
        self.voltage_pub = self.create_publisher(
            Float32MultiArray, 'battery/voltages', 10
        )
        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )

        # Create timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_battery_status)

        self.get_logger().info(f'Battery monitor started at {publish_rate} Hz')

    def publish_battery_status(self):
        voltages = []

        if self.adc:
            try:
                for ch in self.channels:
                    v = self.adc.read_voltage(ch, divider=self.voltage_divider)
                    voltages.append(v)
            except Exception as e:
                self.get_logger().warn(f'Failed to read ADC: {e}')
                return
        else:
            # Simulation mode
            voltages = [7.4, 7.4]

        # Publish voltages
        msg = Float32MultiArray()
        msg.data = voltages
        self.voltage_pub.publish(msg)

        # Publish diagnostics
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()

        for i, voltage in enumerate(voltages):
            status = DiagnosticStatus()
            status.name = f'Battery {i+1}'
            status.hardware_id = f'battery_{i+1}'

            if voltage < self.low_voltage:
                status.level = DiagnosticStatus.WARN
                status.message = 'Low voltage'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'OK'

            status.values.append(KeyValue(key='voltage', value=f'{voltage:.2f}V'))
            diag.status.append(status)

        self.diag_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
