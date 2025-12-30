#!/usr/bin/env python3
"""
Servo Driver Node for Hexapod Robot
Controls PCA9685 PWM servo drivers for leg and head servos

Based on reference implementation in reference/servo.py and reference/pca9685.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

# Hardware imports
try:
    import smbus
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False


class PCA9685:
    """PCA9685 PWM driver - adapted from reference/pca9685.py"""

    # Register addresses
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, bus, address=0x40):
        self.bus = smbus.SMBus(bus)
        self.address = address
        self._init_device()

    def _init_device(self):
        """Initialize PCA9685"""
        self.bus.write_byte_data(self.address, self.MODE1, 0x00)
        time.sleep(0.005)

    def set_pwm_freq(self, freq):
        """Set PWM frequency"""
        prescale = int(25000000.0 / (4096 * freq) - 1)
        old_mode = self.bus.read_byte_data(self.address, self.MODE1)
        new_mode = (old_mode & 0x7F) | 0x10  # Sleep mode
        self.bus.write_byte_data(self.address, self.MODE1, new_mode)
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode | 0x80)

    def set_pwm(self, channel, on, off):
        """Set PWM on/off values for a channel"""
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel, on & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 1, on >> 8)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 2, off & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 3, off >> 8)

    def set_servo_angle(self, channel, angle, min_pulse=500, max_pulse=2500):
        """Set servo angle (0-180 degrees)"""
        angle = max(0, min(180, angle))
        pulse = min_pulse + (max_pulse - min_pulse) * angle / 180
        # Convert to PWM value (at 50Hz, 4096 steps per 20ms period)
        pwm_value = int(pulse * 4096 / 20000)
        self.set_pwm(channel, 0, pwm_value)


class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')

        # Declare parameters
        self.declare_parameter('i2c.bus', 1)
        self.declare_parameter('i2c.pca9685_addr_1', 0x40)
        self.declare_parameter('i2c.pca9685_addr_2', 0x41)
        self.declare_parameter('servos.pwm_frequency', 50)
        self.declare_parameter('servos.min_pulse_us', 500)
        self.declare_parameter('servos.max_pulse_us', 2500)

        # Get parameters
        bus = self.get_parameter('i2c.bus').value
        addr1 = self.get_parameter('i2c.pca9685_addr_1').value
        addr2 = self.get_parameter('i2c.pca9685_addr_2').value
        pwm_freq = self.get_parameter('servos.pwm_frequency').value
        self.min_pulse = self.get_parameter('servos.min_pulse_us').value
        self.max_pulse = self.get_parameter('servos.max_pulse_us').value

        # Initialize PCA9685 drivers
        self.pwm_drivers = []
        if HARDWARE_AVAILABLE:
            try:
                pwm1 = PCA9685(bus, addr1)
                pwm1.set_pwm_freq(pwm_freq)
                self.pwm_drivers.append(pwm1)
                self.get_logger().info(f'PCA9685 initialized at 0x{addr1:02x}')

                pwm2 = PCA9685(bus, addr2)
                pwm2.set_pwm_freq(pwm_freq)
                self.pwm_drivers.append(pwm2)
                self.get_logger().info(f'PCA9685 initialized at 0x{addr2:02x}')

            except Exception as e:
                self.get_logger().error(f'Failed to initialize PCA9685: {e}')
        else:
            self.get_logger().warn('Hardware not available, running in simulation mode')

        # Create subscribers
        # Joint positions: 18 leg servos + 2 head servos = 20 values
        self.joint_sub = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_callback,
            10
        )

        # Individual leg command for testing
        self.leg_sub = self.create_subscription(
            Float64MultiArray,
            'leg_command',
            self.leg_callback,
            10
        )

        self.get_logger().info('Servo driver started')

    def joint_callback(self, msg):
        """Handle joint position commands (20 angles in degrees)"""
        if len(msg.data) < 20:
            self.get_logger().warn(f'Expected 20 joint values, got {len(msg.data)}')
            return

        if not self.pwm_drivers:
            return

        # First 16 channels on driver 0, remaining on driver 1
        for i, angle in enumerate(msg.data):
            if i < 16:
                driver_idx = 0
                channel = i
            else:
                driver_idx = 1
                channel = i - 16

            if driver_idx < len(self.pwm_drivers):
                self.pwm_drivers[driver_idx].set_servo_angle(
                    channel, angle, self.min_pulse, self.max_pulse
                )

    def leg_callback(self, msg):
        """Handle single leg command [leg_id, coxa, femur, tibia]"""
        if len(msg.data) < 4:
            return

        leg_id = int(msg.data[0])
        angles = msg.data[1:4]

        # Leg channel mapping (from hardware.yaml)
        leg_channels = [
            [0, 1, 2],      # Leg 0
            [3, 4, 5],      # Leg 1
            [6, 7, 8],      # Leg 2
            [16, 17, 18],   # Leg 3
            [19, 20, 21],   # Leg 4
            [22, 23, 24],   # Leg 5
        ]

        if leg_id < 0 or leg_id >= len(leg_channels):
            return

        channels = leg_channels[leg_id]
        for i, (channel, angle) in enumerate(zip(channels, angles)):
            if channel < 16:
                driver_idx = 0
            else:
                driver_idx = 1
                channel = channel - 16

            if driver_idx < len(self.pwm_drivers):
                self.pwm_drivers[driver_idx].set_servo_angle(
                    channel, angle, self.min_pulse, self.max_pulse
                )


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
