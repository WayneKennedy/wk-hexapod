#!/usr/bin/env python3
"""
Autonomy Manager for Hexapod Robot

Central state machine that coordinates autonomous behavior:
1. Wait for startup sequence to complete
2. Check if map exists, try localization
3. Look around to gather visual features
4. If localized, wait for external mission; else start mapping
5. If no mission received, start frontier exploration

State Machine:
  WAITING_FOR_STARTUP -> CHECKING_MAP -> LOOK_AROUND ->
    -> LOCALIZATION_MODE (if localized) -> WAITING_FOR_MISSION ->
       -> EXECUTING_MISSION (if command received)
       -> EXPLORING (if timeout)
    -> MAPPING_MODE (if not localized) -> EXPLORING
"""

import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, String
from hexapod_interfaces.msg import AutonomyState, LocalizationStatus
from hexapod_interfaces.action import LookAround, ExploreFrontiers
from hexapod_interfaces.srv import StartMission, GetAutonomyState
from enum import IntEnum
import time
import threading


class State(IntEnum):
    """Autonomy states matching AutonomyState.msg constants."""
    WAITING_FOR_STARTUP = 0
    CHECKING_MAP = 1
    LOOK_AROUND = 2
    MAPPING_MODE = 3
    LOCALIZATION_MODE = 4
    WAITING_FOR_MISSION = 5
    EXECUTING_MISSION = 6
    EXPLORING = 7
    EXPLORATION_COMPLETE = 8
    ERROR = 9


STATE_NAMES = {
    State.WAITING_FOR_STARTUP: 'waiting_for_startup',
    State.CHECKING_MAP: 'checking_map',
    State.LOOK_AROUND: 'look_around',
    State.MAPPING_MODE: 'mapping_mode',
    State.LOCALIZATION_MODE: 'localization_mode',
    State.WAITING_FOR_MISSION: 'waiting_for_mission',
    State.EXECUTING_MISSION: 'executing_mission',
    State.EXPLORING: 'exploring',
    State.EXPLORATION_COMPLETE: 'exploration_complete',
    State.ERROR: 'error',
}

LED_COLORS = {
    State.WAITING_FOR_STARTUP: '255,255,0',      # Yellow
    State.CHECKING_MAP: '255,255,0',             # Yellow
    State.LOOK_AROUND: '255,0,255',              # Magenta
    State.MAPPING_MODE: '0,255,255',             # Cyan
    State.LOCALIZATION_MODE: '0,0,255',          # Blue
    State.WAITING_FOR_MISSION: '0,255,0',        # Green
    State.EXECUTING_MISSION: '255,255,255',      # White
    State.EXPLORING: '0,255,0',                  # Green (should pulse)
    State.EXPLORATION_COMPLETE: '0,0,255',       # Blue
    State.ERROR: '255,0,0',                      # Red
}


class AutonomyManager(Node):
    """Central coordinator for autonomous behavior."""

    def __init__(self):
        super().__init__('autonomy_manager')

        self.callback_group = ReentrantCallbackGroup()

        # RTAB-Map mode switch (localization -> mapping after a failed localization)
        self.rtabmap_mapping_client = self.create_client(
            Empty, '/rtabmap/set_mode_mapping', callback_group=self.callback_group)

        # Parameters
        self.declare_parameter('mission_timeout_sec', 60.0)
        self.declare_parameter('map_db_path', '~/.hexapod/maps/rtabmap.db')
        self.declare_parameter('state_publish_rate_hz', 2.0)

        self.mission_timeout = self.get_parameter('mission_timeout_sec').value
        self.map_db_path = os.path.expanduser(
            self.get_parameter('map_db_path').value
        )
        publish_rate = self.get_parameter('state_publish_rate_hz').value

        # State tracking
        self.current_state = State.WAITING_FOR_STARTUP
        self.slam_mode = 'unknown'
        self.robot_initialized = False
        self.localization_status = None
        self.mission_active = False
        self.current_mission_id = ''
        self.exploration_progress = 0.0
        self.mission_timeout_remaining = 0.0
        self.error_message = ''
        self.waiting_start_time = None

        # Action state tracking (thread-safe)
        self._action_lock = threading.Lock()
        self._look_around_in_progress = False
        self._exploration_in_progress = False
        self._current_goal_handle = None

        # Action clients
        self.look_around_client = ActionClient(
            self, LookAround, '/look_around',
            callback_group=self.callback_group
        )
        self.explore_client = ActionClient(
            self, ExploreFrontiers, '/explore_frontiers',
            callback_group=self.callback_group
        )

        # Publishers
        self.state_pub = self.create_publisher(AutonomyState, '/autonomy/state', 10)
        self.led_pub = self.create_publisher(String, '/leds/zone', 10)

        # Subscribers
        self.initialized_sub = self.create_subscription(
            Bool,
            '/robot/initialized',
            self.initialized_callback,
            10,
            callback_group=self.callback_group
        )
        self.localization_sub = self.create_subscription(
            LocalizationStatus,
            '/localization_status',
            self.localization_callback,
            10,
            callback_group=self.callback_group
        )
        self.mission_sub = self.create_subscription(
            String,
            '/mission/command',
            self.mission_command_callback,
            10,
            callback_group=self.callback_group
        )

        # Services
        self.get_state_srv = self.create_service(
            GetAutonomyState,
            '/autonomy/get_state',
            self.get_state_callback,
            callback_group=self.callback_group
        )

        # State machine timer
        self.state_timer = self.create_timer(
            1.0 / publish_rate,
            self.state_machine_tick,
            callback_group=self.callback_group
        )

        self.get_logger().info('Autonomy Manager started')
        self.get_logger().info(f'  Mission timeout: {self.mission_timeout}s')
        self.get_logger().info(f'  Map DB path: {self.map_db_path}')

        # Set initial LED
        self.set_led_for_state()

    def initialized_callback(self, msg):
        """Robot initialization status."""
        if msg.data and not self.robot_initialized:
            self.robot_initialized = True
            self.get_logger().info('Robot initialization complete')

    def localization_callback(self, msg):
        """Track localization status from slam_monitor."""
        self.localization_status = msg

    def mission_command_callback(self, msg):
        """Handle external mission command."""
        if self.current_state == State.WAITING_FOR_MISSION:
            self.get_logger().info(f'Received mission command: {msg.data}')
            self.current_mission_id = msg.data
            self.mission_active = True
            self.transition_to(State.EXECUTING_MISSION)

    def get_state_callback(self, request, response):
        """Service to query current state."""
        response.state = self.build_state_message()
        return response

    def build_state_message(self):
        """Build AutonomyState message."""
        msg = AutonomyState()
        msg.state = int(self.current_state)
        msg.state_name = STATE_NAMES.get(self.current_state, 'unknown')
        msg.slam_mode = self.slam_mode
        msg.mission_active = self.mission_active
        msg.mission_id = self.current_mission_id
        msg.exploration_progress = self.exploration_progress
        msg.mission_timeout_remaining = self.mission_timeout_remaining
        msg.error_message = self.error_message
        return msg

    def publish_state(self):
        """Publish current state."""
        msg = self.build_state_message()
        self.state_pub.publish(msg)

    def set_led_for_state(self):
        """Set LED color based on current state."""
        color = LED_COLORS.get(self.current_state, '255,0,0')
        msg = String()
        msg.data = f'rear:{color}'
        self.led_pub.publish(msg)

    def transition_to(self, new_state):
        """Transition to a new state."""
        if new_state != self.current_state:
            self.get_logger().info(
                f'State transition: {STATE_NAMES[self.current_state]} -> {STATE_NAMES[new_state]}'
            )
            self.current_state = new_state
            self.set_led_for_state()

    def map_exists(self):
        """Check if RTAB-Map database exists."""
        return os.path.exists(self.map_db_path)

    def is_localized(self):
        """Check if currently localized."""
        if self.localization_status is None:
            return False
        return self.localization_status.status == LocalizationStatus.LOCALIZED

    def _send_look_around_goal(self):
        """Send look around goal and set up callbacks."""
        if not self.look_around_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Look around action server not available')
            self._on_look_around_complete(False)
            return

        goal = LookAround.Goal()
        goal.check_localization = True
        goal.timeout_sec = 15.0

        self.get_logger().info('Starting look around sequence')
        send_goal_future = self.look_around_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._look_around_goal_response_callback)

    def _look_around_goal_response_callback(self, future):
        """Handle look around goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Look around goal rejected')
            self._on_look_around_complete(False)
            return

        self.get_logger().info('Look around goal accepted')
        with self._action_lock:
            self._current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._look_around_result_callback)

    def _look_around_result_callback(self, future):
        """Handle look around result."""
        try:
            result = future.result()
            self.get_logger().info(f'Look around complete: {result.result.message}')
            localized = result.result.localized
        except Exception as e:
            self.get_logger().error(f'Look around error: {e}')
            localized = False

        with self._action_lock:
            self._current_goal_handle = None

        self._on_look_around_complete(localized)

    def _on_look_around_complete(self, localized):
        """Handle look around completion and transition state."""
        with self._action_lock:
            self._look_around_in_progress = False

        if localized:
            self.transition_to(State.LOCALIZATION_MODE)
        else:
            self.get_logger().info('Localization failed, switching to mapping mode')
            self.slam_mode = 'mapping'
            self._set_rtabmap_mapping()
            self.transition_to(State.MAPPING_MODE)

    def _set_rtabmap_mapping(self):
        """Tell RTAB-Map to extend the map (it started in localization mode)."""
        if not self.rtabmap_mapping_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/rtabmap/set_mode_mapping not available; map will not grow')
            return
        future = self.rtabmap_mapping_client.call_async(Empty.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info('RTAB-Map switched to mapping mode')
            if f.exception() is None else
            self.get_logger().error(f'set_mode_mapping failed: {f.exception()}'))

    def _send_exploration_goal(self):
        """Send exploration goal and set up callbacks."""
        if not self.explore_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Exploration action server not available')
            self._on_exploration_complete(False, 'Action server not available')
            return

        goal = ExploreFrontiers.Goal()
        goal.max_duration_sec = 0.0  # No timeout
        goal.min_frontier_size = 0.3

        self.get_logger().info('Starting frontier exploration')
        send_goal_future = self.explore_client.send_goal_async(
            goal,
            feedback_callback=self.exploration_feedback_callback
        )
        send_goal_future.add_done_callback(self._exploration_goal_response_callback)

    def _exploration_goal_response_callback(self, future):
        """Handle exploration goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Exploration goal rejected')
            self._on_exploration_complete(False, 'Goal rejected')
            return

        self.get_logger().info('Exploration goal accepted')
        with self._action_lock:
            self._current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._exploration_result_callback)

    def _exploration_result_callback(self, future):
        """Handle exploration result."""
        try:
            result = future.result()
            self.get_logger().info(f'Exploration complete: {result.result.message}')
            success = result.result.success
            message = result.result.message
        except Exception as e:
            self.get_logger().error(f'Exploration error: {e}')
            success = False
            message = str(e)

        with self._action_lock:
            self._current_goal_handle = None

        self._on_exploration_complete(success, message)

    def _on_exploration_complete(self, success, message):
        """Handle exploration completion and transition state."""
        with self._action_lock:
            self._exploration_in_progress = False

        if success:
            self.transition_to(State.EXPLORATION_COMPLETE)
        else:
            self.error_message = message
            self.transition_to(State.ERROR)

    def exploration_feedback_callback(self, feedback_msg):
        """Handle exploration feedback."""
        feedback = feedback_msg.feedback
        self.exploration_progress = feedback.progress_percent / 100.0
        self.get_logger().debug(
            f'Exploration: {feedback.frontiers_remaining} frontiers, '
            f'{feedback.progress_percent:.1f}%'
        )

    def state_machine_tick(self):
        """Main state machine tick - called periodically."""
        self.publish_state()

        if self.current_state == State.WAITING_FOR_STARTUP:
            if self.robot_initialized:
                self.transition_to(State.CHECKING_MAP)

        elif self.current_state == State.CHECKING_MAP:
            if self.map_exists():
                self.get_logger().info('Map database found, attempting localization')
                self.slam_mode = 'localization'
                self.transition_to(State.LOOK_AROUND)
            else:
                self.get_logger().info('No map database, starting in mapping mode')
                self.slam_mode = 'mapping'
                self.transition_to(State.MAPPING_MODE)

        elif self.current_state == State.LOOK_AROUND:
            # Look around is async - triggered once when entering this state
            with self._action_lock:
                if not self._look_around_in_progress:
                    self._look_around_in_progress = True
                    self._send_look_around_goal()
            # Transition handled in _on_look_around_complete callback

        elif self.current_state == State.MAPPING_MODE:
            # In mapping mode, go directly to exploration
            self.transition_to(State.EXPLORING)

        elif self.current_state == State.LOCALIZATION_MODE:
            # Successfully localized, wait for mission
            self.waiting_start_time = time.time()
            self.transition_to(State.WAITING_FOR_MISSION)

        elif self.current_state == State.WAITING_FOR_MISSION:
            if self.waiting_start_time:
                elapsed = time.time() - self.waiting_start_time
                self.mission_timeout_remaining = max(0, self.mission_timeout - elapsed)

                if elapsed >= self.mission_timeout:
                    self.get_logger().info('Mission timeout, starting exploration')
                    self.waiting_start_time = None
                    self.transition_to(State.EXPLORING)

        elif self.current_state == State.EXECUTING_MISSION:
            # Mission execution is handled by mission_server
            # Just wait for completion
            if not self.mission_active:
                self.transition_to(State.WAITING_FOR_MISSION)
                self.waiting_start_time = time.time()

        elif self.current_state == State.EXPLORING:
            # Exploration is async - triggered once when entering this state
            with self._action_lock:
                if not self._exploration_in_progress:
                    self._exploration_in_progress = True
                    self._send_exploration_goal()
            # Transition handled in _on_exploration_complete callback


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyManager()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
