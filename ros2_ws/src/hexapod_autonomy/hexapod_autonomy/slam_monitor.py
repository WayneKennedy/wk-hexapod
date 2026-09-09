#!/usr/bin/env python3
"""
SLAM Monitor Node for Hexapod Robot

Monitors RTAB-Map topics to detect:
- Whether a map database exists
- Localization status (localizing, localized, lost, mapping)
- Loop closure detection for localization confidence

Publishes LocalizationStatus for autonomy_manager to consume.
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from hexapod_interfaces.msg import LocalizationStatus


class SlamMonitor(Node):
    """Monitor RTAB-Map SLAM status and publish localization state."""

    # Status constants matching LocalizationStatus.msg
    STATUS_UNKNOWN = 0
    STATUS_LOCALIZING = 1
    STATUS_LOCALIZED = 2
    STATUS_LOST = 3
    STATUS_MAPPING = 4

    STATUS_NAMES = {
        STATUS_UNKNOWN: 'unknown',
        STATUS_LOCALIZING: 'localizing',
        STATUS_LOCALIZED: 'localized',
        STATUS_LOST: 'lost',
        STATUS_MAPPING: 'mapping',
    }

    def __init__(self):
        super().__init__('slam_monitor')

        self.callback_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('map_db_path', '~/.hexapod/maps/rtabmap.db')
        self.declare_parameter('min_loop_closures', 3)
        self.declare_parameter('localization_timeout_sec', 15.0)
        self.declare_parameter('publish_rate_hz', 2.0)

        self.map_db_path = os.path.expanduser(
            self.get_parameter('map_db_path').value
        )
        self.min_loop_closures = self.get_parameter('min_loop_closures').value
        self.localization_timeout = self.get_parameter('localization_timeout_sec').value
        publish_rate = self.get_parameter('publish_rate_hz').value

        # State tracking
        self.map_exists = os.path.exists(self.map_db_path)
        self.map_available = False
        self.last_map_time = None
        self.loop_closure_count = 0
        self.consecutive_loop_closures = 0
        self.localization_start_time = None
        self.current_status = self.STATUS_UNKNOWN
        self.is_mapping_mode = False

        # Publisher
        self.status_pub = self.create_publisher(
            LocalizationStatus, '/localization_status', 10
        )

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10,
            callback_group=self.callback_group
        )

        # Subscribe to rtabmap/info for loop closure detection
        # Note: rtabmap_ros/Info is not a standard message, we'll use a workaround
        # by monitoring the /map topic updates as a proxy for SLAM activity
        self.initialized_sub = self.create_subscription(
            Bool,
            '/robot/initialized',
            self.initialized_callback,
            10,
            callback_group=self.callback_group
        )

        # Periodic status publisher
        self.status_timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_status,
            callback_group=self.callback_group
        )

        self.get_logger().info('SLAM Monitor started')
        self.get_logger().info(f'  Map DB path: {self.map_db_path}')
        self.get_logger().info(f'  Map exists: {self.map_exists}')

    def initialized_callback(self, msg):
        """Robot initialization complete - start monitoring if not already."""
        if msg.data and self.localization_start_time is None:
            self.localization_start_time = self.get_clock().now()
            self.get_logger().info('Robot initialized, starting localization monitoring')

    def map_callback(self, msg):
        """Handle incoming map messages."""
        now = self.get_clock().now()

        # Track map availability
        was_available = self.map_available
        self.map_available = True
        self.last_map_time = now

        # If map just became available, this could indicate localization
        if not was_available:
            self.get_logger().info('Map topic now receiving data')

        # Use map updates as a proxy for SLAM activity
        # In localization mode with successful loop closures, map updates happen
        if self.map_available and not self.is_mapping_mode:
            # Count consecutive map updates as evidence of localization
            self.consecutive_loop_closures += 1
            self.loop_closure_count += 1

    def get_localization_duration(self):
        """Get time spent attempting localization."""
        if self.localization_start_time is None:
            return 0.0
        elapsed = self.get_clock().now() - self.localization_start_time
        return elapsed.nanoseconds / 1e9

    def assess_status(self):
        """Determine current localization status."""
        # Check if map database exists
        self.map_exists = os.path.exists(self.map_db_path)

        # If in explicit mapping mode
        if self.is_mapping_mode:
            return self.STATUS_MAPPING

        # If no map exists, we're in unknown state until mapping starts
        if not self.map_exists:
            return self.STATUS_UNKNOWN

        # If localization hasn't started yet
        if self.localization_start_time is None:
            return self.STATUS_UNKNOWN

        # Check for successful localization
        if self.consecutive_loop_closures >= self.min_loop_closures:
            return self.STATUS_LOCALIZED

        # Check for timeout
        duration = self.get_localization_duration()
        if duration > self.localization_timeout:
            return self.STATUS_LOST

        # Still attempting to localize
        if self.map_available:
            return self.STATUS_LOCALIZING

        return self.STATUS_UNKNOWN

    def calculate_confidence(self):
        """Calculate localization confidence (0.0 to 1.0)."""
        if self.current_status == self.STATUS_LOCALIZED:
            # Base confidence on consecutive loop closures
            confidence = min(1.0, self.consecutive_loop_closures / (self.min_loop_closures * 2))
            return confidence
        elif self.current_status == self.STATUS_LOCALIZING:
            # Partial confidence during localization attempt
            return 0.3 + (0.4 * min(1.0, self.consecutive_loop_closures / self.min_loop_closures))
        return 0.0

    def publish_status(self):
        """Publish current localization status."""
        self.current_status = self.assess_status()

        msg = LocalizationStatus()
        msg.status = self.current_status
        msg.status_name = self.STATUS_NAMES.get(self.current_status, 'unknown')
        msg.map_exists = self.map_exists
        msg.map_available = self.map_available
        msg.loop_closure_count = self.loop_closure_count
        msg.confidence = self.calculate_confidence()
        msg.localization_duration_sec = self.get_localization_duration()

        self.status_pub.publish(msg)

    def set_mapping_mode(self, is_mapping):
        """Called externally to indicate mapping mode."""
        self.is_mapping_mode = is_mapping
        if is_mapping:
            self.consecutive_loop_closures = 0
            self.loop_closure_count = 0
            self.get_logger().info('Switched to mapping mode')
        else:
            self.localization_start_time = self.get_clock().now()
            self.get_logger().info('Switched to localization mode')

    def reset_localization_tracking(self):
        """Reset localization tracking for a new attempt."""
        self.consecutive_loop_closures = 0
        self.localization_start_time = self.get_clock().now()
        self.get_logger().info('Reset localization tracking')


def main(args=None):
    rclpy.init(args=args)
    node = SlamMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
