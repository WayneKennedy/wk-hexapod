#!/usr/bin/env python3
"""
Mission Server Node for Hexapod Robot

Provides ROS 2 service interface for external mission commands:
- StartMission: Start a named mission (explore, navigate, return_home)
- StopMission: Stop current mission
- Publishes mission commands to /mission/command for autonomy_manager
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from hexapod_interfaces.action import ExploreFrontiers

# Nav2 is optional
try:
    from nav2_msgs.action import NavigateToPose
    NAV2_AVAILABLE = True
except ImportError:
    NAV2_AVAILABLE = False
from hexapod_interfaces.srv import StartMission, StopMission
from hexapod_interfaces.msg import AutonomyState
import uuid
import time


class MissionServer(Node):
    """ROS 2 service interface for mission control."""

    def __init__(self):
        super().__init__('mission_server')

        self.callback_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('navigation_timeout_sec', 120.0)
        self.declare_parameter('return_home_on_abort', True)

        self.nav_timeout = self.get_parameter('navigation_timeout_sec').value
        self.return_home_on_abort = self.get_parameter('return_home_on_abort').value

        # State tracking
        self.current_mission = None
        self.current_mission_id = ''
        self.start_pose = None
        self.is_executing = False
        self.active_goal_handle = None

        # Action clients
        self.nav_client = None
        if NAV2_AVAILABLE:
            self.nav_client = ActionClient(
                self, NavigateToPose, '/navigate_to_pose',
                callback_group=self.callback_group
            )
        else:
            self.get_logger().warn('Nav2 not available - navigate/patrol missions disabled')

        self.explore_client = ActionClient(
            self, ExploreFrontiers, '/explore_frontiers',
            callback_group=self.callback_group
        )

        # Publishers
        self.mission_cmd_pub = self.create_publisher(String, '/mission/command', 10)

        # Subscribers
        self.autonomy_state_sub = self.create_subscription(
            AutonomyState,
            '/autonomy/state',
            self.autonomy_state_callback,
            10,
            callback_group=self.callback_group
        )

        # Services
        self.start_srv = self.create_service(
            StartMission,
            '/mission/start',
            self.start_mission_callback,
            callback_group=self.callback_group
        )
        self.stop_srv = self.create_service(
            StopMission,
            '/mission/stop',
            self.stop_mission_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Mission Server started')

    def autonomy_state_callback(self, msg):
        """Track autonomy state."""
        # Could use this to sync state with autonomy_manager
        pass

    def generate_mission_id(self):
        """Generate unique mission ID."""
        return f'mission_{uuid.uuid4().hex[:8]}'

    def publish_mission_command(self, command):
        """Publish mission command for autonomy_manager."""
        msg = String()
        msg.data = command
        self.mission_cmd_pub.publish(msg)

    async def start_mission_callback(self, request, response):
        """Handle start mission request."""
        mission_type = request.mission_type.lower()

        # Check if already executing
        if self.is_executing:
            response.accepted = False
            response.message = f'Already executing mission {self.current_mission_id}'
            response.mission_id = ''
            return response

        # Validate mission type
        valid_types = ['explore', 'navigate', 'return_home', 'patrol']
        if mission_type not in valid_types:
            response.accepted = False
            response.message = f'Unknown mission type: {mission_type}. Valid: {valid_types}'
            response.mission_id = ''
            return response

        # Generate mission ID
        mission_id = self.generate_mission_id()
        self.current_mission = mission_type
        self.current_mission_id = mission_id
        self.is_executing = True

        self.get_logger().info(f'Starting mission: {mission_type} ({mission_id})')

        # Notify autonomy manager
        self.publish_mission_command(f'{mission_type}:{mission_id}')

        # Execute mission based on type
        if mission_type == 'explore':
            success, message = await self._execute_explore(request.timeout_sec)
        elif mission_type == 'navigate':
            if not request.waypoints:
                response.accepted = False
                response.message = 'Navigate mission requires waypoints'
                response.mission_id = ''
                self.is_executing = False
                return response
            success, message = await self._execute_navigate(request.waypoints, request.timeout_sec)
        elif mission_type == 'return_home':
            success, message = await self._execute_return_home()
        elif mission_type == 'patrol':
            if not request.waypoints:
                response.accepted = False
                response.message = 'Patrol mission requires waypoints'
                response.mission_id = ''
                self.is_executing = False
                return response
            success, message = await self._execute_patrol(request.waypoints, request.timeout_sec)
        else:
            success = False
            message = 'Not implemented'

        self.is_executing = False

        response.accepted = True
        response.message = message
        response.mission_id = mission_id

        self.get_logger().info(f'Mission complete: {message}')
        return response

    async def stop_mission_callback(self, request, response):
        """Handle stop mission request."""
        if not self.is_executing:
            response.success = False
            response.message = 'No active mission'
            return response

        # Cancel active goal
        if self.active_goal_handle is not None:
            try:
                await self.active_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f'Error canceling goal: {e}')

        self.is_executing = False
        mission_id = self.current_mission_id
        self.current_mission = None
        self.current_mission_id = ''

        response.success = True
        response.message = f'Stopped mission {mission_id}'

        # Return home if requested
        if request.return_home and self.start_pose is not None:
            self.get_logger().info('Returning to start position')
            await self._navigate_to_pose(self.start_pose)

        return response

    async def _execute_explore(self, timeout_sec):
        """Execute exploration mission."""
        if not self.explore_client.wait_for_server(timeout_sec=5.0):
            return False, 'Exploration action server not available'

        goal = ExploreFrontiers.Goal()
        goal.max_duration_sec = timeout_sec if timeout_sec > 0 else 0.0
        goal.min_frontier_size = 0.3

        try:
            future = self.explore_client.send_goal_async(goal)
            self.active_goal_handle = await future

            if not self.active_goal_handle.accepted:
                return False, 'Exploration goal rejected'

            result = await self.active_goal_handle.get_result_async()
            return result.result.success, result.result.message

        except Exception as e:
            return False, str(e)

        finally:
            self.active_goal_handle = None

    async def _execute_navigate(self, waypoints, timeout_sec):
        """Execute navigation to waypoints."""
        if not waypoints:
            return False, 'No waypoints provided'

        for i, waypoint in enumerate(waypoints):
            if not self.is_executing:
                return False, 'Mission stopped'

            self.get_logger().info(f'Navigating to waypoint {i + 1}/{len(waypoints)}')

            success = await self._navigate_to_pose(waypoint)
            if not success:
                return False, f'Failed to reach waypoint {i + 1}'

        return True, f'Reached all {len(waypoints)} waypoints'

    async def _execute_patrol(self, waypoints, timeout_sec):
        """Execute patrol through waypoints (loops continuously)."""
        if not waypoints:
            return False, 'No waypoints provided'

        start_time = time.monotonic()
        loops = 0

        while self.is_executing:
            # Check timeout
            if timeout_sec > 0 and (time.monotonic() - start_time) > timeout_sec:
                return True, f'Patrol timeout after {loops} loops'

            for waypoint in waypoints:
                if not self.is_executing:
                    return True, f'Patrol stopped after {loops} loops'

                success = await self._navigate_to_pose(waypoint)
                if not success:
                    self.get_logger().warn('Failed to reach patrol waypoint, continuing...')

            loops += 1
            self.get_logger().info(f'Completed patrol loop {loops}')

        return True, f'Patrol complete after {loops} loops'

    async def _execute_return_home(self):
        """Execute return to starting position."""
        if self.start_pose is None:
            # Use origin as fallback
            home = PoseStamped()
            home.header.frame_id = 'map'
            home.pose.position.x = 0.0
            home.pose.position.y = 0.0
            home.pose.orientation.w = 1.0
            self.start_pose = home

        success = await self._navigate_to_pose(self.start_pose)
        if success:
            return True, 'Returned home'
        return False, 'Failed to return home'

    async def _navigate_to_pose(self, pose):
        """Navigate to a specific pose."""
        if self.nav_client is None:
            self.get_logger().warn('Nav2 not installed')
            return False

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 action server not available')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        if not goal.pose.header.frame_id:
            goal.pose.header.frame_id = 'map'

        try:
            future = self.nav_client.send_goal_async(goal)
            goal_handle = await future

            if not goal_handle.accepted:
                return False

            self.active_goal_handle = goal_handle
            result = await goal_handle.get_result_async()
            return True  # Nav2 doesn't return success field

        except Exception as e:
            self.get_logger().error(f'Navigation error: {e}')
            return False

        finally:
            self.active_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = MissionServer()

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
