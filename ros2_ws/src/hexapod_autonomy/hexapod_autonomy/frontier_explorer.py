#!/usr/bin/env python3
"""
Frontier Explorer Node for Hexapod Robot

Implements frontier-based exploration:
1. Subscribe to the map (map_topic: Nav2's global costmap, built from the
   head's sonar sweeps)
2. Detect frontier cells (unknown adjacent to free)
3. Cluster frontiers and filter by size
4. Select closest frontier as navigation goal
5. Send goal to Nav2 /navigate_to_pose
6. On arrival, survey with the head (LookAround) so the map grows without
   the body turning
7. Repeat until no frontiers remain
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from hexapod_interfaces.action import ExploreFrontiers, LookAround

# TF2 for robot pose lookup
from tf2_ros import Buffer, TransformListener, TransformException

# Nav2 is optional - exploration will work but navigation won't
try:
    from nav2_msgs.action import NavigateToPose
    NAV2_AVAILABLE = True
except ImportError:
    NAV2_AVAILABLE = False
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from collections import deque
import math
import time


class FrontierExplorer(Node):
    """Frontier-based exploration action server."""

    def __init__(self):
        super().__init__('frontier_explorer')

        self.callback_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('min_frontier_size', 0.3)  # meters
        self.declare_parameter('goal_tolerance', 0.3)     # meters
        self.declare_parameter('exploration_timeout_sec', 300.0)
        self.declare_parameter('goal_selection_strategy', 'closest')
        self.declare_parameter('unknown_threshold', -1)   # OccupancyGrid unknown
        self.declare_parameter('free_threshold', 50)      # Below this = free
        self.declare_parameter('max_nav_failures', 3)
        self.declare_parameter('map_topic', '/global_costmap/costmap')
        self.declare_parameter('survey_sweeps', 1)
        self.declare_parameter('min_goal_distance', 0.4)

        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.exploration_timeout = self.get_parameter('exploration_timeout_sec').value
        self.goal_strategy = self.get_parameter('goal_selection_strategy').value
        self.unknown_threshold = self.get_parameter('unknown_threshold').value
        self.free_threshold = self.get_parameter('free_threshold').value
        self.max_nav_failures = self.get_parameter('max_nav_failures').value
        self.survey_sweeps = self.get_parameter('survey_sweeps').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value

        # State
        self.current_map = None
        self.robot_pose = None
        self.goal_handle = None
        self.nav_goal_handle = None
        self.frontiers_explored = 0
        self.nav_failures = 0

        # TF2 for robot pose lookup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 action client (optional)
        # Goals Nav2 failed to reach; nearby frontiers are skipped afterwards
        self.failed_goals = []
        self.declare_parameter('blacklist_radius', 0.3)
        self.blacklist_radius = self.get_parameter('blacklist_radius').value

        self.nav_client = None
        if NAV2_AVAILABLE:
            self.nav_client = ActionClient(
                self, NavigateToPose, '/navigate_to_pose',
                callback_group=self.callback_group
            )
        else:
            self.get_logger().warn('Nav2 not available - exploration will detect frontiers but cannot navigate')

        self.look_around_client = ActionClient(
            self, LookAround, '/look_around', callback_group=self.callback_group)

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self.map_callback,
            10,
            callback_group=self.callback_group
        )

        # Publishers
        self.frontier_marker_pub = self.create_publisher(
            MarkerArray, '/frontiers/markers', 10
        )
        self.current_goal_pub = self.create_publisher(
            PoseStamped, '/frontiers/current_goal', 10
        )

        # Action server
        self._action_server = ActionServer(
            self,
            ExploreFrontiers,
            '/explore_frontiers',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Frontier Explorer started')

    def map_callback(self, msg):
        """Store latest map."""
        self.current_map = msg

    def goal_callback(self, goal_request):
        """Accept or reject exploration goal."""
        if self.goal_handle is not None and self.goal_handle.is_active:
            self.get_logger().warn('Rejecting goal: exploration already in progress')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept cancellation requests."""
        self.get_logger().info('Received cancel request')
        # Cancel ongoing navigation
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    def detect_frontiers(self, occupancy_grid):
        """
        Detect frontier cells in the occupancy grid.

        Frontier = unknown cell (-1) 4-adjacent to a free cell (0-49).
        Returns (cells as (row, col) array, grid info).
        """
        info = occupancy_grid.info
        data = np.array(occupancy_grid.data, dtype=np.int16).reshape(info.height, info.width)

        unknown = (data == self.unknown_threshold)
        free = (data >= 0) & (data < self.free_threshold)

        free_dilated = free.copy()
        free_dilated[1:, :] |= free[:-1, :]
        free_dilated[:-1, :] |= free[1:, :]
        free_dilated[:, 1:] |= free[:, :-1]
        free_dilated[:, :-1] |= free[:, 1:]

        return np.argwhere(unknown & free_dilated), info

    def cluster_frontiers(self, frontier_cells, info):
        """
        Group 8-connected frontier cells (a flood fill, linear in the cell
        count). Returns list of (centroid_x, centroid_y, size_meters) in the
        map frame.
        """
        remaining = set(map(tuple, frontier_cells.tolist()))
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        clusters = []
        while remaining:
            seed = remaining.pop()
            queue = deque([seed])
            members = [seed]
            while queue:
                r, c = queue.popleft()
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        n = (r + dr, c + dc)
                        if n in remaining:
                            remaining.remove(n)
                            queue.append(n)
                            members.append(n)
            cells = np.array(members)
            centroid_x = ox + (cells[:, 1].mean() + 0.5) * res
            centroid_y = oy + (cells[:, 0].mean() + 0.5) * res
            clusters.append((centroid_x, centroid_y, len(members) * res))
        return clusters

    def filter_frontiers(self, clusters, min_size):
        """Filter clusters by minimum size."""
        return [c for c in clusters if c[2] >= min_size]

    def select_goal(self, frontiers, robot_x, robot_y, strategy='closest'):
        """
        Select next exploration goal from frontier clusters.

        Strategies:
        - closest: Nearest frontier
        - largest: Biggest frontier
        """
        if not frontiers:
            return None

        if strategy == 'largest':
            # Sort by size (descending)
            frontiers = sorted(frontiers, key=lambda f: f[2], reverse=True)
            return frontiers[0]

        # Default: closest
        def distance(f):
            return math.sqrt((f[0] - robot_x) ** 2 + (f[1] - robot_y) ** 2)

        frontiers = sorted(frontiers, key=distance)
        return frontiers[0]

    def get_robot_pose(self):
        """
        Get current robot pose from TF.
        Returns (x, y) in map frame, or map origin as fallback.
        """
        try:
            # Look up transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
            return (
                transform.transform.translation.x,
                transform.transform.translation.y
            )
        except TransformException as e:
            self.get_logger().debug(f'Could not get robot pose from TF: {e}')
            # Fall back to map origin if TF not available
            if self.current_map:
                return (
                    self.current_map.info.origin.position.x,
                    self.current_map.info.origin.position.y
                )
            return (0.0, 0.0)

    def create_goal_pose(self, x, y, from_x=None, from_y=None):
        """Create PoseStamped for navigation goal, facing the direction of travel.

        The goal checker ignores yaw (nav2_params.yaml), so this orientation
        only tells the planner which way the path arrives; the body does not
        turn in place on arrival.
        """
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        yaw = 0.0 if from_x is None else math.atan2(y - from_y, x - from_x)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    async def survey(self):
        """Head-only look-around at the current position; failures are logged, not fatal."""
        if self.survey_sweeps <= 0:
            return
        if not self.look_around_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('look_around (head_controller) not available; skipping survey')
            return
        goal = LookAround.Goal()
        goal.sweeps = self.survey_sweeps
        try:
            handle = await self.look_around_client.send_goal_async(goal)
            if not handle.accepted:
                self.get_logger().warn('Head survey rejected')
                return
            result = await handle.get_result_async()
            self.get_logger().info(f'Head survey: {result.result.message}')
        except Exception as e:
            self.get_logger().warn(f'Head survey failed: {e}')

    def publish_frontier_markers(self, frontiers, current_goal=None):
        """Publish visualization markers for frontiers."""
        marker_array = MarkerArray()

        # Delete old markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # Add frontier markers
        for i, (x, y, size) in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            marker.scale.x = max(0.1, size * 0.5)
            marker.scale.y = max(0.1, size * 0.5)
            marker.scale.z = 0.1

            # Color: green for normal, red for current goal
            if current_goal and abs(x - current_goal[0]) < 0.1 and abs(y - current_goal[1]) < 0.1:
                marker.color.r = 1.0
                marker.color.g = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.7

            marker_array.markers.append(marker)

        self.frontier_marker_pub.publish(marker_array)

    def _is_blacklisted(self, x, y):
        """True if (x, y) is within blacklist_radius of a goal Nav2 failed to reach."""
        return any(math.hypot(x - fx, y - fy) < self.blacklist_radius
                   for fx, fy in self.failed_goals)

    async def navigate_to(self, goal_pose):
        """Send navigation goal and wait for result."""
        if self.nav_client is None:
            self.get_logger().warn('Nav2 not installed')
            return False, 'Nav2 not installed'

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 action server not available')
            return False, 'Nav2 not available'

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        self.current_goal_pub.publish(goal_pose)

        future = self.nav_client.send_goal_async(goal)

        try:
            self.nav_goal_handle = await future

            if not self.nav_goal_handle.accepted:
                return False, 'Goal rejected'

            result_future = self.nav_goal_handle.get_result_async()
            result = await result_future

            # NavigateToPose has no success field; the action status tells us
            # whether Nav2 reached the goal, aborted, or was cancelled.
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                return True, 'Navigation complete'
            status_names = {
                GoalStatus.STATUS_ABORTED: 'aborted',
                GoalStatus.STATUS_CANCELED: 'canceled',
            }
            return False, f'Navigation {status_names.get(result.status, result.status)}'

        except Exception as e:
            return False, str(e)

        finally:
            self.nav_goal_handle = None

    async def execute_callback(self, goal_handle):
        """Execute frontier exploration."""
        self.goal_handle = goal_handle
        self.frontiers_explored = 0
        self.failed_goals = []
        self.nav_failures = 0

        request = goal_handle.request
        min_frontier_size = request.min_frontier_size if request.min_frontier_size > 0 else self.min_frontier_size
        max_duration = request.max_duration_sec if request.max_duration_sec > 0 else self.exploration_timeout

        feedback = ExploreFrontiers.Feedback()
        result = ExploreFrontiers.Result()

        start_time = time.monotonic()

        self.get_logger().info('Starting frontier exploration')

        try:
            while True:
                # Check cancellation
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Canceled'
                    result.frontiers_explored = self.frontiers_explored
                    return result

                # Check timeout
                elapsed = time.monotonic() - start_time
                if max_duration > 0 and elapsed >= max_duration:
                    result.success = True
                    result.message = 'Timeout reached'
                    result.frontiers_explored = self.frontiers_explored
                    goal_handle.succeed()
                    return result

                # Check map availability
                if self.current_map is None:
                    self.get_logger().warn('No map available, waiting...')
                    await self._sleep(1.0)
                    continue

                # Detect frontiers
                frontier_cells, info = self.detect_frontiers(self.current_map)

                if len(frontier_cells) == 0:
                    self.get_logger().info('No frontiers detected, exploration complete')
                    result.success = True
                    result.message = 'No more frontiers'
                    result.frontiers_explored = self.frontiers_explored
                    goal_handle.succeed()
                    return result

                # Cluster and filter
                clusters = self.cluster_frontiers(frontier_cells, info)
                frontiers = self.filter_frontiers(clusters, min_frontier_size)

                if not frontiers:
                    self.get_logger().info('No frontiers above size threshold')
                    result.success = True
                    result.message = 'No significant frontiers'
                    result.frontiers_explored = self.frontiers_explored
                    goal_handle.succeed()
                    return result

                # Skip frontiers near goals Nav2 already failed to reach
                frontiers = [f for f in frontiers if not self._is_blacklisted(f[0], f[1])]
                if not frontiers:
                    self.get_logger().info('All remaining frontiers are unreachable, exploration complete')
                    result.success = True
                    result.message = 'Remaining frontiers unreachable'
                    result.frontiers_explored = self.frontiers_explored
                    goal_handle.succeed()
                    return result

                # Get robot pose and select goal. Frontiers at the robot's own
                # feet (cells beside the body the forward sonar never sees)
                # would be "reached" at once, forever; skip them.
                robot_x, robot_y = self.get_robot_pose()
                frontiers = [f for f in frontiers
                             if math.hypot(f[0] - robot_x, f[1] - robot_y) >= self.min_goal_distance]
                if not frontiers:
                    self.get_logger().info('Only frontiers within min_goal_distance remain, exploration complete')
                    result.success = True
                    result.message = 'No frontiers beyond min_goal_distance'
                    result.frontiers_explored = self.frontiers_explored
                    goal_handle.succeed()
                    return result
                selected = self.select_goal(frontiers, robot_x, robot_y, self.goal_strategy)

                if selected is None:
                    continue

                goal_x, goal_y, _ = selected

                # Publish markers
                self.publish_frontier_markers(frontiers, (goal_x, goal_y))

                # Publish feedback
                feedback.frontiers_remaining = len(frontiers)
                feedback.current_goal = self.create_goal_pose(goal_x, goal_y, robot_x, robot_y)
                feedback.progress_percent = min(99.0, self.frontiers_explored * 10.0)
                feedback.status = f'Navigating to frontier at ({goal_x:.2f}, {goal_y:.2f})'
                goal_handle.publish_feedback(feedback)

                self.get_logger().info(f'Navigating to frontier at ({goal_x:.2f}, {goal_y:.2f})')

                # Navigate to frontier
                goal_pose = self.create_goal_pose(goal_x, goal_y, robot_x, robot_y)
                success, message = await self.navigate_to(goal_pose)

                if success:
                    self.frontiers_explored += 1
                    self.nav_failures = 0
                    self.get_logger().info(f'Reached frontier, total explored: {self.frontiers_explored}')
                    await self.survey()
                else:
                    self.nav_failures += 1
                    self.failed_goals.append((goal_x, goal_y))
                    self.get_logger().warn(f'Navigation failed: {message}')

                    if self.nav_failures >= self.max_nav_failures:
                        result.success = False
                        result.message = f'Too many navigation failures: {message}'
                        result.frontiers_explored = self.frontiers_explored
                        goal_handle.abort()
                        return result

                # Small delay before next iteration
                await self._sleep(0.5)

        except Exception as e:
            self.get_logger().error(f'Exploration error: {e}')
            result.success = False
            result.message = str(e)
            result.frontiers_explored = self.frontiers_explored
            goal_handle.abort()
            return result

        finally:
            self.goal_handle = None

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
    node = FrontierExplorer()

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
