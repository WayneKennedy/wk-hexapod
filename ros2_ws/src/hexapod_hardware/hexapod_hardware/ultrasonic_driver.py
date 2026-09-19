#!/usr/bin/env python3
"""
Ultrasonic Driver Node for Hexapod Robot
Publishes HC-SR04 ranges from the kit's head sensor as sensor_msgs/Range.

Pins are the vendor's (Code/Server/ultrasonic.py: trigger GPIO 27, echo
GPIO 22). The echo pulse is timed from kernel line-event timestamps (lgpio
alerts), not from Python wake-ups, so scheduler latency under load does not
enter the measurement; each message is stamped at the echo's falling edge.

No echo within max_range is published as range == max_range, which the Nav2
RangeSensorLayer (clear_on_max_reading) and collision monitor read as "nothing
within max_range". A missing echo pulse altogether is a sensor fault and is
not published.
"""

import threading
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Range

try:
    import lgpio
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False


class UltrasonicDriver(Node):
    def __init__(self):
        super().__init__('ultrasonic_driver')

        self.declare_parameter('ultrasonic.gpiochip', 4)
        self.declare_parameter('ultrasonic.trigger_gpio', 27)
        self.declare_parameter('ultrasonic.echo_gpio', 22)
        self.declare_parameter('ultrasonic.rate', 15.0)
        self.declare_parameter('ultrasonic.min_range', 0.03)
        self.declare_parameter('ultrasonic.max_range', 2.0)
        self.declare_parameter('ultrasonic.field_of_view', 0.26)
        self.declare_parameter('ultrasonic.speed_of_sound', 343.0)
        self.declare_parameter('ultrasonic.frame_id', 'ultrasonic_link')

        chip = self.get_parameter('ultrasonic.gpiochip').value
        self.trigger = self.get_parameter('ultrasonic.trigger_gpio').value
        self.echo = self.get_parameter('ultrasonic.echo_gpio').value
        rate = self.get_parameter('ultrasonic.rate').value
        self.min_range = self.get_parameter('ultrasonic.min_range').value
        self.max_range = self.get_parameter('ultrasonic.max_range').value
        self.fov = self.get_parameter('ultrasonic.field_of_view').value
        self.speed_of_sound = self.get_parameter('ultrasonic.speed_of_sound').value
        self.frame_id = self.get_parameter('ultrasonic.frame_id').value

        self.pub = self.create_publisher(Range, 'ultrasonic/range', 10)

        # Edges of the current ping: (level, monotonic tick ns), from lgpio's thread
        self._lock = threading.Lock()
        self._edges = []
        self._pinged = False
        self._missed = 0

        self.chip = None
        self.cb = None
        if HARDWARE_AVAILABLE:
            try:
                self.chip = lgpio.gpiochip_open(chip)
                lgpio.gpio_claim_output(self.chip, self.trigger, 0)
                lgpio.gpio_claim_alert(self.chip, self.echo, lgpio.BOTH_EDGES)
                self.cb = lgpio.callback(
                    self.chip, self.echo, lgpio.BOTH_EDGES, self._on_edge)
                self.get_logger().info(
                    f'HC-SR04 on gpiochip{chip}: trigger {self.trigger}, echo {self.echo}')
            except Exception as e:
                self.get_logger().error(f'Failed to claim ultrasonic GPIO: {e}')
                self._release()
        else:
            self.get_logger().warn('lgpio not available; no ranges will be published')

        # The datasheet asks for >= 60 ms between pings; at 15 Hz each tick
        # first reads the previous ping, then fires the next.
        period = max(1.0 / rate, 0.06)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f'Ultrasonic driver started at {1.0 / period:.1f} Hz, '
            f'{self.min_range}-{self.max_range} m')

    def _on_edge(self, chip, gpio, level, tick):
        with self._lock:
            self._edges.append((level, tick))

    def _tick(self):
        if self.chip is None:
            return
        if self._pinged:
            self._read_ping()
        with self._lock:
            self._edges = []
        lgpio.gpio_write(self.chip, self.trigger, 1)
        time.sleep(0.00001)  # >= 10 us trigger pulse
        lgpio.gpio_write(self.chip, self.trigger, 0)
        self._pinged = True

    def _read_ping(self):
        with self._lock:
            edges = list(self._edges)
        rise = next((t for level, t in edges if level == 1), None)
        fall = next((t for level, t in edges if level == 0 and rise is not None and t > rise), None)
        if rise is None or fall is None:
            self._missed += 1
            if self._missed in (5, 50) or self._missed % 500 == 0:
                self.get_logger().warn(f'No echo pulse from HC-SR04 ({self._missed} pings)')
            return
        self._missed = 0

        distance = (fall - rise) * 1e-9 * self.speed_of_sound / 2.0
        if distance < self.min_range:
            return
        distance = min(distance, self.max_range)

        # Back-date the stamp to the echo's falling edge (ticks are CLOCK_MONOTONIC)
        age_ns = max(0, time.monotonic_ns() - fall)
        msg = Range()
        msg.header.stamp = (self.get_clock().now() - Duration(nanoseconds=age_ns)).to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = float(self.fov)
        msg.min_range = float(self.min_range)
        msg.max_range = float(self.max_range)
        msg.range = float(distance)
        self.pub.publish(msg)

    def _release(self):
        if self.cb is not None:
            self.cb.cancel()
            self.cb = None
        if self.chip is not None:
            for gpio in (self.trigger, self.echo):
                try:
                    lgpio.gpio_free(self.chip, gpio)
                except Exception:
                    pass
            lgpio.gpiochip_close(self.chip)
            self.chip = None

    def destroy_node(self):
        self._release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
