#!/usr/bin/env python3
"""
Startup Sequence Node for Hexapod Robot
Manages safe servo initialization with visual/audio warnings

Sequence:
1. YELLOW (rear LED) - System ready, waiting to initialize
2. RED + beeping - Warning: servos about to snap to home
3. HOME + CYAN - Legs snap to home, place robot on floor now
4. Beep - Warning: robot about to stand
5. STAND - Robot stands up
6. GREEN - Safe, robot ready

The sequence can be triggered:
- Automatically at boot (if auto_start enabled)
- Via /robot/safe_startup service (for leg reset after pickup)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger


class StartupSequence(Node):
    def __init__(self):
        super().__init__('startup_sequence')

        # Use reentrant callback group for service calls during callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters
        self.declare_parameter('startup.auto_start', False)  # Auto-run on boot
        self.declare_parameter('startup.warning_duration', 2.0)  # seconds
        self.declare_parameter('startup.beep_interval', 0.5)  # seconds
        self.declare_parameter('startup.place_delay', 10.0)  # seconds between home and stand

        # Get parameters
        self.auto_start = self.get_parameter('startup.auto_start').value
        self.warning_duration = self.get_parameter('startup.warning_duration').value
        self.beep_interval = self.get_parameter('startup.beep_interval').value
        self.place_delay = self.get_parameter('startup.place_delay').value

        # State tracking
        self.sequence_running = False
        self.initialized = False

        # Publishers
        self.led_pub = self.create_publisher(String, 'leds/zone', 10)
        self.buzzer_pub = self.create_publisher(Bool, 'buzzer/state', 10)
        self.pose_pub = self.create_publisher(String, 'pose_command', 10)

        # Service to trigger startup sequence
        self.startup_srv = self.create_service(
            Trigger,
            '/robot/safe_startup',
            self.safe_startup_callback,
            callback_group=self.callback_group
        )

        # Wait for LED controller to be ready, then set initial state
        self._initial_led_timer = self.create_timer(
            1.0, self._set_initial_led, callback_group=self.callback_group
        )

        self.get_logger().info('Startup sequence node ready')
        self.get_logger().info(f'  Warning: {self.warning_duration}s, Place delay: {self.place_delay}s')
        self.get_logger().info('  REAR LED: YELLOW (waiting for /robot/safe_startup)')

        # Auto-start if configured
        if self.auto_start:
            self.get_logger().info('Auto-start enabled, beginning sequence in 2 seconds...')
            self.create_timer(2.0, self.auto_start_callback, callback_group=self.callback_group)

    def _set_initial_led(self):
        """Set initial LED state after delay (one-shot timer)"""
        self.set_rear_led('yellow')
        self.get_logger().info('REAR LED set to YELLOW (waiting)')
        # Cancel the timer after first run
        if hasattr(self, '_initial_led_timer'):
            self._initial_led_timer.cancel()

    def set_rear_led(self, color):
        """Set rear LED to a named color"""
        colors = {
            'yellow': '255,255,0',
            'red': '255,0,0',
            'green': '0,255,0',
            'cyan': '0,255,255',   # Place on floor indicator
            'off': '0,0,0',
        }
        msg = String()
        msg.data = f'rear:{colors.get(color, "0,0,0")}'
        self.led_pub.publish(msg)

    def beep(self, on):
        """Turn buzzer on or off"""
        msg = Bool()
        msg.data = on
        self.buzzer_pub.publish(msg)

    def send_pose_command(self, command):
        """Send pose command (home, stand, relax)"""
        msg = String()
        msg.data = command
        self.pose_pub.publish(msg)
        self.get_logger().info(f'Sent pose command: {command}')

    def auto_start_callback(self):
        """One-shot callback for auto-start"""
        # Cancel the timer after first run
        self.destroy_timer(self._timers[0])
        self.run_startup_sequence()

    def safe_startup_callback(self, request, response):
        """Service callback to trigger startup sequence"""
        if self.sequence_running:
            response.success = False
            response.message = 'Startup sequence already running'
            return response

        success, message = self.run_startup_sequence()
        response.success = success
        response.message = message
        return response

    def run_startup_sequence(self):
        """Execute the full startup sequence (blocking)"""
        if self.sequence_running:
            return False, 'Sequence already running'

        self.sequence_running = True
        self.get_logger().info('Starting safe startup sequence...')

        try:
            # Phase 1: Warning - RED + beeping
            self.get_logger().info(f'Phase 1: WARNING - servos will snap in {self.warning_duration}s')
            self.set_rear_led('red')

            # Beep at interval for warning duration
            beeps = int(self.warning_duration / self.beep_interval)
            for i in range(beeps):
                self.beep(True)
                self._sleep(0.1)  # Short beep
                self.beep(False)
                self._sleep(self.beep_interval - 0.1)

            # Phase 2: HOME - legs snap to home position
            self.get_logger().info('Phase 2: HOME - legs snapping to home position')
            self.send_pose_command('home')
            self._sleep(0.5)  # Brief pause for command to execute

            # Phase 3: Place delay - CYAN indicates "place robot on floor now"
            self.get_logger().info(f'Phase 3: PLACE ON FLOOR - {self.place_delay}s to position robot')
            self.set_rear_led('cyan')
            self._sleep(self.place_delay)

            # Phase 4: Stand warning - quick beep before standing
            self.get_logger().info('Phase 4: STANDING - robot will stand now')
            self.beep(True)
            self._sleep(0.2)
            self.beep(False)
            self._sleep(0.3)

            # Phase 5: STAND - robot stands up
            self.send_pose_command('stand')
            self._sleep(1.5)  # Wait for stand animation to complete

            # Phase 6: Success - GREEN
            self.get_logger().info('Phase 6: SAFE - robot ready')
            self.set_rear_led('green')
            self.initialized = True

            self.sequence_running = False
            return True, 'Startup sequence complete'

        except Exception as e:
            self.get_logger().error(f'Startup sequence error: {e}')
            self.set_rear_led('red')
            self.sequence_running = False
            return False, f'Error: {e}'

    def _sleep(self, duration):
        """Sleep while allowing callbacks"""
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds / 1e9 < duration:
            rclpy.spin_once(self, timeout_sec=0.01)


def main(args=None):
    rclpy.init(args=args)
    node = StartupSequence()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
