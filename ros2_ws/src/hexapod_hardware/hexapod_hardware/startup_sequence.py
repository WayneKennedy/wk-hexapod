#!/usr/bin/env python3
"""
Startup Sequence Node for Hexapod Robot
Manages safe servo initialization with visual/audio warnings

Sequence:
0. Wait until hexapod_controller and servo_driver subscribe to pose_command.
   The topic is volatile: a command sent earlier is lost (OQ-28).
1. YELLOW (rear LED) - System ready, waiting to initialize
2. RED + beeping - Warning: servos about to snap to home
3. HOME + CYAN - Legs snap to home, place robot on floor now
4. Beep - Warning: robot about to stand
5. STAND - Robot stands up
6. GREEN - Safe, robot ready

HOME must be confirmed by the controller (hexapod/initialized) before the
sequence goes on. A timeout in step 0 or an unconfirmed HOME ends the sequence
on RED, and /robot/initialized is not published.

The sequence can be triggered:
- Automatically at boot (if auto_start enabled)
- Via /robot/safe_startup service (for leg reset after pickup)
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
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
        self.declare_parameter('startup.controller_timeout', 60.0)  # seconds to wait for subscribers
        self.declare_parameter('startup.confirm_timeout', 5.0)  # seconds to wait for home confirmation

        # Get parameters
        self.auto_start = self.get_parameter('startup.auto_start').value
        self.warning_duration = self.get_parameter('startup.warning_duration').value
        self.beep_interval = self.get_parameter('startup.beep_interval').value
        self.place_delay = self.get_parameter('startup.place_delay').value
        self.controller_timeout = self.get_parameter('startup.controller_timeout').value
        self.confirm_timeout = self.get_parameter('startup.confirm_timeout').value

        # State tracking
        self.sequence_running = False
        self.initialized = False
        self.controller_initialized = False

        # Publishers
        self.led_pub = self.create_publisher(String, 'leds/zone', 10)
        self.buzzer_pub = self.create_publisher(Bool, 'buzzer/state', 10)
        self.pose_pub = self.create_publisher(String, 'pose_command', 10)
        self.initialized_pub = self.create_publisher(Bool, '/robot/initialized', 10)

        # Subscribers
        self.controller_init_sub = self.create_subscription(
            Bool, 'hexapod/initialized', self._controller_initialized_callback, 10,
            callback_group=self.callback_group)

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
        self._auto_start_timer = None
        if self.auto_start:
            self.get_logger().info('Auto-start enabled, beginning sequence in 2 seconds...')
            self._auto_start_timer = self.create_timer(
                2.0, self.auto_start_callback, callback_group=self.callback_group)

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

    def _controller_initialized_callback(self, msg):
        self.controller_initialized = msg.data

    def _wait_for_pose_subscribers(self):
        """Block until the nodes that act on pose_command subscribe to it."""
        needed = {'hexapod_controller', 'servo_driver'}
        deadline = time.monotonic() + self.controller_timeout
        while True:
            present = {info.node_name for info in
                       self.get_subscriptions_info_by_topic(self.pose_pub.topic_name)}
            missing = needed - present
            if not missing:
                return True
            if time.monotonic() >= deadline:
                self.get_logger().error(
                    f'No pose_command subscription from {sorted(missing)} '
                    f'after {self.controller_timeout}s')
                return False
            self._sleep(0.2)

    def _wait_for_home_confirmed(self):
        """Block until the controller reports it is initialized."""
        deadline = time.monotonic() + self.confirm_timeout
        while not self.controller_initialized:
            if time.monotonic() >= deadline:
                self.get_logger().error(
                    f'Controller did not confirm home within {self.confirm_timeout}s')
                return False
            self._sleep(0.1)
        return True

    def _abort(self, message):
        self.set_rear_led('red')
        self.sequence_running = False
        return False, message

    def _publish_initialized(self):
        """Publish initialization status (called periodically after init)"""
        msg = Bool()
        msg.data = self.initialized
        self.initialized_pub.publish(msg)

    def auto_start_callback(self):
        """One-shot callback for auto-start"""
        # Cancel the timer before running so the sequence cannot re-trigger
        if self._auto_start_timer is not None:
            self._auto_start_timer.cancel()
            self._auto_start_timer = None
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
            # Phase 0: the warning must directly precede the snap, so wait first
            if not self._wait_for_pose_subscribers():
                return self._abort('Controller or servo driver not listening')

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
            if not self._wait_for_home_confirmed():
                return self._abort('Home not confirmed by controller')

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

            # Publish initialization complete and start periodic republishing
            self._publish_initialized()
            self._init_status_timer = self.create_timer(
                1.0, self._publish_initialized, callback_group=self.callback_group
            )

            self.sequence_running = False
            return True, 'Startup sequence complete'

        except Exception as e:
            self.get_logger().error(f'Startup sequence error: {e}')
            self.set_rear_led('red')
            self.sequence_running = False
            return False, f'Error: {e}'

    def _sleep(self, duration):
        """Blocking sleep. Other callbacks keep running on the multithreaded executor."""
        time.sleep(duration)


def main(args=None):
    rclpy.init(args=args)
    node = StartupSequence()
    # The sequence blocks inside a callback for ~15 s; a multithreaded executor
    # keeps the service, timers and publishers responsive meanwhile.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
