#!/usr/bin/env python3
"""
Look Around Action Server for Hexapod Robot

Executes a physical sweep sequence using head pan/tilt and body rotation
to gather visual features for SLAM localization assessment.

Phases:
1. Head sweep - Pan/tilt the head camera left/right/up/down
2. Body rotation - Rotate body in place using legs
3. Combined - Head movement with body pitch for vertical coverage
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from hexapod_interfaces.action import LookAround
from hexapod_interfaces.msg import LocalizationStatus
import time
import math


class LookAroundServer(Node):
    """Action server for look around behavior."""

    def __init__(self):
        super().__init__('look_around')

        self.callback_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('head_pan_min', 45.0)
        self.declare_parameter('head_pan_max', 135.0)
        self.declare_parameter('head_pan_center', 90.0)
        self.declare_parameter('head_tilt_min', 70.0)
        self.declare_parameter('head_tilt_max', 110.0)
        self.declare_parameter('head_tilt_center', 90.0)
        self.declare_parameter('body_yaw_speed', 0.3)  # rad/s for cmd_vel
        self.declare_parameter('body_rotation_duration', 0.5)  # seconds per direction
        self.declare_parameter('position_hold_time', 0.4)  # seconds
        self.declare_parameter('default_timeout', 15.0)  # seconds

        self.head_pan_min = self.get_parameter('head_pan_min').value
        self.head_pan_max = self.get_parameter('head_pan_max').value
        self.head_pan_center = self.get_parameter('head_pan_center').value
        self.head_tilt_min = self.get_parameter('head_tilt_min').value
        self.head_tilt_max = self.get_parameter('head_tilt_max').value
        self.head_tilt_center = self.get_parameter('head_tilt_center').value
        self.body_yaw_speed = self.get_parameter('body_yaw_speed').value
        self.body_rotation_duration = self.get_parameter('body_rotation_duration').value
        self.position_hold_time = self.get_parameter('position_hold_time').value
        self.default_timeout = self.get_parameter('default_timeout').value

        # State tracking
        self.localization_status = None
        self.is_localized = False
        self.goal_handle = None

        # Publishers
        self.head_pub = self.create_publisher(Float64MultiArray, '/head_command', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.localization_sub = self.create_subscription(
            LocalizationStatus,
            '/localization_status',
            self.localization_callback,
            10,
            callback_group=self.callback_group
        )

        # Action server
        self._action_server = ActionServer(
            self,
            LookAround,
            '/look_around',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Look Around action server started')

    def localization_callback(self, msg):
        """Track localization status."""
        self.localization_status = msg
        self.is_localized = msg.status == LocalizationStatus.LOCALIZED

    def goal_callback(self, goal_request):
        """Accept or reject a goal."""
        if self.goal_handle is not None and self.goal_handle.is_active:
            self.get_logger().warn('Rejecting goal: another look_around in progress')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept cancellation requests."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def set_head_position(self, pan, tilt):
        """Set head pan/tilt position."""
        msg = Float64MultiArray()
        msg.data = [float(pan), float(tilt)]
        self.head_pub.publish(msg)

    def send_cmd_vel(self, angular_z):
        """Send rotation command."""
        msg = Twist()
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def stop_movement(self):
        """Stop all movement."""
        self.send_cmd_vel(0.0)

    async def execute_callback(self, goal_handle):
        """Execute the look around sequence."""
        self.goal_handle = goal_handle
        self.get_logger().info('Executing look around sequence')

        request = goal_handle.request
        check_localization = request.check_localization
        timeout = request.timeout_sec if request.timeout_sec > 0 else self.default_timeout

        feedback = LookAround.Feedback()
        result = LookAround.Result()

        start_time = time.monotonic()

        try:
            # Phase 1: Head sweep
            feedback.current_phase = 'head_sweep'
            feedback.progress = 0.0
            feedback.localization_detected = False
            goal_handle.publish_feedback(feedback)

            localized = await self._execute_head_sweep(
                goal_handle, feedback, check_localization
            )

            if localized and check_localization:
                result.success = True
                result.localized = True
                result.message = 'Localized during head sweep'
                goal_handle.succeed()
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Canceled'
                return result

            # Check timeout
            if time.monotonic() - start_time > timeout:
                result.success = True
                result.localized = False
                result.message = 'Timeout reached'
                goal_handle.succeed()
                return result

            # Phase 2: Body rotation
            feedback.current_phase = 'body_rotation'
            feedback.progress = 0.4
            goal_handle.publish_feedback(feedback)

            localized = await self._execute_body_rotation(
                goal_handle, feedback, check_localization
            )

            if localized and check_localization:
                result.success = True
                result.localized = True
                result.message = 'Localized during body rotation'
                goal_handle.succeed()
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Canceled'
                return result

            # Check timeout
            if time.monotonic() - start_time > timeout:
                result.success = True
                result.localized = False
                result.message = 'Timeout reached'
                goal_handle.succeed()
                return result

            # Phase 3: Combined head + body movement
            feedback.current_phase = 'combined'
            feedback.progress = 0.7
            goal_handle.publish_feedback(feedback)

            localized = await self._execute_combined_sweep(
                goal_handle, feedback, check_localization
            )

            # Final result
            feedback.progress = 1.0
            goal_handle.publish_feedback(feedback)

            result.success = True
            result.localized = self.is_localized if check_localization else False
            result.message = 'Look around complete'
            if check_localization:
                result.message += f' - {"localized" if result.localized else "not localized"}'

            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f'Look around error: {e}')
            result.success = False
            result.localized = False
            result.message = f'Error: {e}'
            goal_handle.abort()
            return result

        finally:
            # Return head to center and stop movement
            self.set_head_position(self.head_pan_center, self.head_tilt_center)
            self.stop_movement()
            self.goal_handle = None

    async def _execute_head_sweep(self, goal_handle, feedback, check_localization):
        """Execute head pan/tilt sweep."""
        # Head positions: (pan, tilt)
        positions = [
            (self.head_pan_center, self.head_tilt_center),  # Center
            (self.head_pan_max, self.head_tilt_min),        # Right, up
            (self.head_pan_max, self.head_tilt_max),        # Right, down
            (self.head_pan_center, self.head_tilt_center),  # Center
            (self.head_pan_min, self.head_tilt_min),        # Left, up
            (self.head_pan_min, self.head_tilt_max),        # Left, down
            (self.head_pan_center, self.head_tilt_center),  # Center
        ]

        for i, (pan, tilt) in enumerate(positions):
            if goal_handle.is_cancel_requested:
                return False

            self.set_head_position(pan, tilt)
            feedback.progress = 0.1 + (0.3 * i / len(positions))
            goal_handle.publish_feedback(feedback)

            await self._sleep(self.position_hold_time)

            if check_localization and self.is_localized:
                feedback.localization_detected = True
                goal_handle.publish_feedback(feedback)
                return True

        return False

    async def _execute_body_rotation(self, goal_handle, feedback, check_localization):
        """Execute body rotation using cmd_vel."""
        # Return head to center for body rotation
        self.set_head_position(self.head_pan_center, self.head_tilt_center)
        await self._sleep(0.2)

        # Rotate right
        self.send_cmd_vel(-self.body_yaw_speed)
        await self._sleep(self.body_rotation_duration)
        self.stop_movement()
        await self._sleep(0.3)

        if goal_handle.is_cancel_requested:
            return False
        if check_localization and self.is_localized:
            feedback.localization_detected = True
            goal_handle.publish_feedback(feedback)
            return True

        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)

        # Rotate back to center
        self.send_cmd_vel(self.body_yaw_speed)
        await self._sleep(self.body_rotation_duration)
        self.stop_movement()
        await self._sleep(0.3)

        if goal_handle.is_cancel_requested:
            return False
        if check_localization and self.is_localized:
            feedback.localization_detected = True
            goal_handle.publish_feedback(feedback)
            return True

        # Rotate left
        self.send_cmd_vel(self.body_yaw_speed)
        await self._sleep(self.body_rotation_duration)
        self.stop_movement()
        await self._sleep(0.3)

        if check_localization and self.is_localized:
            feedback.localization_detected = True
            goal_handle.publish_feedback(feedback)
            return True

        feedback.progress = 0.6
        goal_handle.publish_feedback(feedback)

        # Return to center
        self.send_cmd_vel(-self.body_yaw_speed)
        await self._sleep(self.body_rotation_duration)
        self.stop_movement()

        return False

    async def _execute_combined_sweep(self, goal_handle, feedback, check_localization):
        """Execute combined head + rotation for maximum coverage."""
        # Combined positions: (head_pan, head_tilt, rotation_direction)
        movements = [
            (self.head_pan_max, self.head_tilt_center, -1),   # Right + rotate right
            (self.head_pan_min, self.head_tilt_center, 1),    # Left + rotate left
            (self.head_pan_center, self.head_tilt_max, 0),    # Look down
            (self.head_pan_center, self.head_tilt_min, 0),    # Look up
            (self.head_pan_center, self.head_tilt_center, 0), # Center
        ]

        for i, (pan, tilt, rotation) in enumerate(movements):
            if goal_handle.is_cancel_requested:
                return False

            self.set_head_position(pan, tilt)

            if rotation != 0:
                self.send_cmd_vel(rotation * self.body_yaw_speed * 0.5)
                await self._sleep(self.body_rotation_duration * 0.5)
                self.stop_movement()
            else:
                await self._sleep(self.position_hold_time)

            feedback.progress = 0.7 + (0.3 * i / len(movements))
            goal_handle.publish_feedback(feedback)

            if check_localization and self.is_localized:
                feedback.localization_detected = True
                goal_handle.publish_feedback(feedback)
                return True

        return False

    def _sleep(self, duration):
        """Awaitable sleep driven by the rclpy executor.

        asyncio.sleep() cannot be used inside rclpy coroutine callbacks: rclpy
        drives the coroutine itself and there is no asyncio event loop running.
        A one-shot timer completing an rclpy Future yields correctly instead.
        """
        future = Future()
        timer = None

        def _done():
            timer.cancel()
            try:
                self.destroy_timer(timer)
            except Exception:
                pass
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(duration, _done, callback_group=self.callback_group)
        return future


def main(args=None):
    rclpy.init(args=args)
    node = LookAroundServer()

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
