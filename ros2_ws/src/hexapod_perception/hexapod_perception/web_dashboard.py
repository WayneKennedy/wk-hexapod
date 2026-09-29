#!/usr/bin/env python3
"""
Web Dashboard for Hexapod Robot

Provides a lightweight web interface with:
- MJPEG camera stream with face detection overlay
- Lidar view: the current 360 degree scan around the robot, in the body frame
- Map display: Nav2's global costmap (slam_toolbox's map plus live lidar
  obstacles, DEC-32) with the robot, its heading and the current scan drawn on it
- Robot status display

Access at http://<robot-ip>:8080
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
import math
import threading
import time
from flask import Flask, Response, render_template_string, request
import json

# Try to import autonomy messages
try:
    from hexapod_interfaces.msg import AutonomyState
    from hexapod_interfaces.srv import StartMission, StopMission, GetAutonomyState
    AUTONOMY_MSGS_AVAILABLE = True
except ImportError:
    AUTONOMY_MSGS_AVAILABLE = False

# Try to import face detection messages
try:
    from hexapod_interfaces.msg import FaceArray
    FACE_MSGS_AVAILABLE = True
except ImportError:
    FACE_MSGS_AVAILABLE = False


class WebDashboard(Node):
    def __init__(self):
        super().__init__('web_dashboard')

        self.callback_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('port', 8080)
        self.declare_parameter('quality', 80)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('map_topic', '/global_costmap/costmap')
        self.declare_parameter('lidar_view_range', 3.0)  # m, radius of the lidar view
        # Drawn on the map; the figure Nav2 plans with (nav2_params.yaml)
        self.declare_parameter('robot_radius', 0.24)
        self.robot_radius = self.get_parameter('robot_radius').value
        self.port = self.get_parameter('port').value
        self.quality = self.get_parameter('quality').value
        self.lidar_view_range = self.get_parameter('lidar_view_range').value

        # State
        self.current_frame = None
        self.current_map = None
        self.map_info = None      # (frame, resolution, origin x, origin y, height in cells)
        self.latest_scan = None   # (monotonic time received, LaserScan)
        self.current_faces = []
        self.battery_voltages = [0.0, 0.0]
        self.autonomy_state = None
        self.frame_lock = threading.Lock()
        self.map_lock = threading.Lock()
        self.autonomy_lock = threading.Lock()

        # Pi camera (camera_ros)
        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_callback,
            10
        )

        # Map (Nav2 global costmap, DEC-32)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self.map_callback,
            10
        )

        # The robot's pose and the current scan, drawn on the map and in the
        # lidar view. The scan is only stored here; it is converted when a
        # frame is drawn.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # Subscribe to face detection
        if FACE_MSGS_AVAILABLE:
            self.face_sub = self.create_subscription(
                FaceArray,
                '/face_recognition/faces',
                self.face_callback,
                10
            )

        # Subscribe to battery
        self.battery_sub = self.create_subscription(
            Float32MultiArray,
            '/battery/voltages',
            self.battery_callback,
            10
        )

        # Autonomy integration
        self.start_mission_client = None
        self.stop_mission_client = None
        self.get_state_client = None

        if AUTONOMY_MSGS_AVAILABLE:
            # Subscribe to autonomy state
            self.autonomy_sub = self.create_subscription(
                AutonomyState,
                '/autonomy/state',
                self.autonomy_callback,
                10,
                callback_group=self.callback_group
            )

            # Service clients for mission control
            self.start_mission_client = self.create_client(
                StartMission, '/mission/start',
                callback_group=self.callback_group
            )
            self.stop_mission_client = self.create_client(
                StopMission, '/mission/stop',
                callback_group=self.callback_group
            )
            self.get_state_client = self.create_client(
                GetAutonomyState, '/autonomy/get_state',
                callback_group=self.callback_group
            )

        self.get_logger().info(f'Web dashboard starting on port {self.port}')

    def image_callback(self, msg):
        """Convert ROS image to OpenCV format"""
        try:
            if msg.encoding == 'rgb8':
                frame = np.frombuffer(msg.data, dtype=np.uint8)
                frame = frame.reshape((msg.height, msg.width, 3))
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                frame = np.frombuffer(msg.data, dtype=np.uint8)
                frame = frame.reshape((msg.height, msg.width, 3))
            elif msg.encoding in ('bgra8', 'rgba8'):
                frame = np.frombuffer(msg.data, dtype=np.uint8)
                frame = frame.reshape((msg.height, msg.step // 4, 4))[:, :msg.width]
                code = cv2.COLOR_BGRA2BGR if msg.encoding == 'bgra8' else cv2.COLOR_RGBA2BGR
                frame = cv2.cvtColor(frame, code)
            else:
                return

            with self.frame_lock:
                self.current_frame = frame
        except Exception as e:
            self.get_logger().warn(f'Failed to process image: {e}')

    def map_callback(self, msg):
        """Convert occupancy grid to image"""
        try:
            # Convert occupancy grid to image
            width = msg.info.width
            height = msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((height, width))

            # BGR. Costmap values: -1 unknown, 0 free, 1-98 inflation,
            # 99 inscribed (the body would touch), 100 an obstacle itself.
            map_img = np.zeros((height, width, 3), dtype=np.uint8)
            map_img[data == -1] = [40, 40, 40]
            map_img[data == 0] = [200, 200, 200]
            inflated = (data > 0) & (data < 99)
            f = data[inflated].astype(np.float32)[:, None] / 98.0
            map_img[inflated] = (200 * (1 - f) + np.array([90, 190, 240]) * f).astype(np.uint8)
            map_img[data == 99] = [60, 130, 250]
            map_img[data == 100] = [0, 0, 200]

            # Flip vertically (ROS maps have origin at bottom-left)
            map_img = cv2.flip(map_img, 0)

            with self.map_lock:
                self.current_map = map_img
                self.map_info = (msg.header.frame_id, msg.info.resolution,
                                 msg.info.origin.position.x, msg.info.origin.position.y,
                                 height)
        except Exception as e:
            self.get_logger().warn(f'Failed to process map: {e}')

    def scan_callback(self, msg):
        self.latest_scan = (time.monotonic(), msg)

    def _pose_in(self, frame, child):
        """(x, y, yaw, age in s) of child in frame from TF, or None"""
        try:
            t = self.tf_buffer.lookup_transform(frame, child, Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        return t.transform.translation.x, t.transform.translation.y, yaw, age

    def _scan_points(self, frame):
        """The current scan as (x, y) arrays in frame, or None without a fresh scan or TF"""
        scan = self.latest_scan
        if scan is None or time.monotonic() - scan[0] > 1.0:
            return None
        laser = self._pose_in(frame, scan[1].header.frame_id)
        if laser is None:
            return None
        lx, ly, lyaw, _ = laser
        m = scan[1]
        r = np.asarray(m.ranges, dtype=np.float32)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment + lyaw
        ok = np.isfinite(r) & (r > m.range_min)
        return lx + r[ok] * np.cos(a[ok]), ly + r[ok] * np.sin(a[ok])

    @staticmethod
    def _draw_points(img, u, v, colour, size):
        """Draw points at pixel columns u and rows v as squares of size pixels"""
        u = u.astype(np.int32)
        v = v.astype(np.int32)
        ok = (u >= 0) & (u < img.shape[1]) & (v >= 0) & (v < img.shape[0])
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        mask[v[ok], u[ok]] = 1
        if size > 1:
            mask = cv2.dilate(mask, np.ones((size, size), np.uint8))
        img[mask > 0] = colour

    def _draw_robot(self, img, info, k):
        """Draw the scale, the current scan and the robot on a map image of k pixels per cell"""
        frame, res, ox, oy, height = info

        def px(x, y):
            return int((x - ox) / res * k), int((height - (y - oy) / res) * k)

        # Scale bar: 1 m
        bar = int(1.0 / res * k)
        y0 = img.shape[0] - 10
        cv2.line(img, (10, y0), (10 + bar, y0), (255, 255, 255), 2)
        cv2.putText(img, '1 m', (14, y0 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        # The scan as the lidar sees it now
        points = self._scan_points(frame)
        if points is not None:
            self._draw_points(img, (points[0] - ox) / res * k,
                              (height - (points[1] - oy) / res) * k,
                              (255, 255, 0), max(2, k // 2))

        # The robot: its planning radius and its heading
        pose = self._pose_in(frame, 'base_link')
        if pose is None:
            cv2.putText(img, 'robot pose unknown', (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            return
        x, y, yaw, age = pose
        stale = age > 2.0
        colour = (150, 150, 150) if stale else (0, 220, 0)
        radius = self.robot_radius / res * k
        centre = px(x, y)
        cv2.circle(img, centre, int(radius), colour, 2)
        tip = (int(centre[0] + 1.8 * radius * math.cos(yaw)),
               int(centre[1] - 1.8 * radius * math.sin(yaw)))
        cv2.arrowedLine(img, centre, tip, colour, 2, tipLength=0.35)
        if stale:
            cv2.putText(img, f'robot pose {age:.0f} s old', (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

    def face_callback(self, msg):
        """Store detected faces for overlay"""
        faces = []
        for face in msg.faces:
            faces.append({
                'x': face.bounding_box.x_offset,
                'y': face.bounding_box.y_offset,
                'w': face.bounding_box.width,
                'h': face.bounding_box.height,
                'name': face.identity if face.recognized else 'Unknown',
                'recognized': face.recognized
            })
        with self.frame_lock:
            self.current_faces = faces

    def battery_callback(self, msg):
        """Store battery voltages"""
        if len(msg.data) >= 2:
            self.battery_voltages = [msg.data[0], msg.data[1]]

    def autonomy_callback(self, msg):
        """Store autonomy state"""
        with self.autonomy_lock:
            self.autonomy_state = msg

    def call_start_mission(self, mission_type, timeout=0.0):
        """Call start mission service synchronously"""
        if self.start_mission_client is None:
            return {'accepted': False, 'message': 'Autonomy not available'}

        if not self.start_mission_client.wait_for_service(timeout_sec=2.0):
            return {'accepted': False, 'message': 'Mission service not available'}

        # The generated message setters do not type-check by default; a non-float
        # (a JSON integer, say) aborts the process in the C conversion layer.
        request = StartMission.Request()
        request.mission_type = str(mission_type)
        request.timeout_sec = float(timeout)

        future = self.start_mission_client.call_async(request)

        # Wait for result with timeout
        import time
        start = time.time()
        while not future.done() and (time.time() - start) < 5.0:
            time.sleep(0.1)

        if future.done():
            result = future.result()
            return {
                'accepted': result.accepted,
                'message': result.message,
                'mission_id': result.mission_id
            }
        return {'accepted': False, 'message': 'Service call timeout'}

    def call_stop_mission(self, return_home=False):
        """Call stop mission service synchronously"""
        if self.stop_mission_client is None:
            return {'success': False, 'message': 'Autonomy not available'}

        if not self.stop_mission_client.wait_for_service(timeout_sec=2.0):
            return {'success': False, 'message': 'Mission service not available'}

        request = StopMission.Request()
        request.return_home = bool(return_home)

        future = self.stop_mission_client.call_async(request)

        import time
        start = time.time()
        while not future.done() and (time.time() - start) < 5.0:
            time.sleep(0.1)

        if future.done():
            result = future.result()
            return {
                'success': result.success,
                'message': result.message
            }
        return {'success': False, 'message': 'Service call timeout'}

    def get_frame_with_overlay(self):
        """Get current frame with face detection overlay"""
        with self.frame_lock:
            if self.current_frame is None:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, 'No Camera Feed', (180, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            else:
                frame = self.current_frame.copy()

                for face in self.current_faces:
                    color = (0, 255, 0) if face['recognized'] else (0, 165, 255)
                    cv2.rectangle(frame,
                                 (face['x'], face['y']),
                                 (face['x'] + face['w'], face['y'] + face['h']),
                                 color, 2)
                    label = face['name']
                    cv2.putText(frame, label,
                               (face['x'], face['y'] - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return frame

    def get_lidar_frame(self):
        """The current scan around the robot: body frame, forward up, left to the left."""
        size = 600
        frame = np.zeros((size, size, 3), dtype=np.uint8)
        c = size // 2
        scale = (c - 10) / self.lidar_view_range      # pixels per metre
        for r in np.arange(0.5, self.lidar_view_range + 1e-6, 0.5):
            cv2.circle(frame, (c, c), int(r * scale), (60, 60, 60), 1)
            if abs(r - round(r)) < 1e-6:
                cv2.putText(frame, f'{r:.0f} m', (c + 4, c - int(r * scale) + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (110, 110, 110), 1)
        points = self._scan_points('base_link')
        if points is None:
            cv2.putText(frame, 'No Lidar Data', (190, 200),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        else:
            # base_link: x forward, y left
            self._draw_points(frame, c - points[1] * scale, c - points[0] * scale,
                              (255, 255, 0), 3)
        radius = int(self.robot_radius * scale)
        cv2.circle(frame, (c, c), radius, (0, 220, 0), 2)
        cv2.arrowedLine(frame, (c, c), (c, c - int(1.8 * radius)), (0, 220, 0), 2,
                        tipLength=0.35)
        return frame

    def _nearest_return(self):
        """(range m, body bearing deg, + left) of the nearest return, or None"""
        points = self._scan_points('base_link')
        if points is None or len(points[0]) == 0:
            return None
        d = np.hypot(points[0], points[1])
        i = int(np.argmin(d))
        return float(d[i]), math.degrees(math.atan2(points[1][i], points[0][i]))

    def get_map_frame(self):
        """Get the occupancy map image"""
        with self.map_lock:
            if self.current_map is None:
                frame = np.zeros((400, 400, 3), dtype=np.uint8)
                cv2.putText(frame, 'No Map', (140, 200),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 100), 2)
                cv2.putText(frame, '(Nav2 global costmap not published)', (40, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 80, 80), 1)
                return frame
            img = self.current_map.copy()
            info = self.map_info
        # Enlarge so that the overlay has pixels to be drawn in
        k = max(1, 640 // max(img.shape[:2]))
        img = cv2.resize(img, None, fx=k, fy=k, interpolation=cv2.INTER_NEAREST)
        self._draw_robot(img, info, k)
        return img

    def generate_mjpeg(self, frame_getter):
        """Generator for MJPEG stream"""
        while rclpy.ok():
            frame = frame_getter()

            _, jpeg = cv2.imencode('.jpg', frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, self.quality])

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' +
                   jpeg.tobytes() + b'\r\n')

            threading.Event().wait(0.05)  # ~20fps

    def get_status(self):
        """Get robot status as JSON"""
        with self.frame_lock:
            face_count = len(self.current_faces)
            recognized = [f['name'] for f in self.current_faces if f['recognized']]

        with self.map_lock:
            has_map = self.current_map is not None

        nearest = self._nearest_return()

        # Autonomy state
        autonomy_info = {
            'available': AUTONOMY_MSGS_AVAILABLE,
            'state': 'unknown',
            'state_name': 'unknown',
            'slam_mode': 'unknown',
            'mission_active': False,
            'mission_id': '',
            'exploration_progress': 0.0,
            'mission_timeout_remaining': 0.0
        }

        with self.autonomy_lock:
            if self.autonomy_state is not None:
                autonomy_info['state'] = self.autonomy_state.state
                autonomy_info['state_name'] = self.autonomy_state.state_name
                autonomy_info['slam_mode'] = self.autonomy_state.slam_mode
                autonomy_info['mission_active'] = self.autonomy_state.mission_active
                autonomy_info['mission_id'] = self.autonomy_state.mission_id
                autonomy_info['exploration_progress'] = round(
                    self.autonomy_state.exploration_progress * 100, 1
                )
                autonomy_info['mission_timeout_remaining'] = round(
                    self.autonomy_state.mission_timeout_remaining, 1
                )

        return {
            'battery': {
                'load': round(self.battery_voltages[0], 2),
                'control': round(self.battery_voltages[1], 2)
            },
            'faces': {
                'count': face_count,
                'recognized': recognized
            },
            'slam': {
                'map_available': has_map,
                'scan_available': nearest is not None,
                'scan_nearest': round(nearest[0], 3) if nearest else None,
                'scan_nearest_bearing': round(nearest[1]) if nearest else None
            },
            'autonomy': autonomy_info
        }


# Flask app
app = Flask(__name__)
dashboard_node = None

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>Hexapod Dashboard</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, sans-serif;
            background: #1a1a2e;
            color: #eee;
            min-height: 100vh;
        }
        .header {
            text-align: center;
            padding: 15px;
            background: #16213e;
            margin-bottom: 20px;
        }
        .header h1 { margin: 0; font-size: 1.5em; }
        .container { max-width: 1400px; margin: 0 auto; padding: 0 20px 20px; }
        .main-grid {
            display: grid;
            grid-template-columns: 1fr 1fr 280px;
            gap: 15px;
        }
        @media (max-width: 1200px) {
            .main-grid { grid-template-columns: 1fr 1fr; }
            .status-panel { grid-column: span 2; }
        }
        @media (max-width: 800px) {
            .main-grid { grid-template-columns: 1fr; }
            .status-panel { grid-column: span 1; }
        }
        .panel {
            background: #16213e;
            border-radius: 8px;
            overflow: hidden;
        }
        .panel-header {
            background: #0f3460;
            padding: 10px 15px;
            font-weight: 600;
            font-size: 0.9em;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        .video-panel img {
            width: 100%;
            height: auto;
            display: block;
            background: #000;
        }
        .status-panel { padding: 15px; }
        .stat {
            display: flex;
            justify-content: space-between;
            padding: 8px 0;
            border-bottom: 1px solid #222;
            font-size: 0.9em;
        }
        .stat:last-child { border-bottom: none; }
        .stat-label { color: #888; }
        .stat-value { font-weight: bold; }
        .stat-value.good { color: #4ade80; }
        .stat-value.warn { color: #fbbf24; }
        .stat-value.bad { color: #f87171; }
        .stat-value.info { color: #60a5fa; }
        .faces {
            margin-top: 15px;
            padding-top: 10px;
            border-top: 1px solid #333;
        }
        .face-tag {
            display: inline-block;
            background: #4ade80;
            color: #000;
            padding: 3px 10px;
            border-radius: 15px;
            margin: 3px;
            font-size: 0.85em;
        }
        .legend {
            margin-top: 15px;
            padding-top: 10px;
            border-top: 1px solid #333;
            font-size: 0.8em;
            color: #666;
        }
        .legend-item {
            display: flex;
            align-items: center;
            margin: 5px 0;
        }
        .legend-color {
            width: 20px;
            height: 10px;
            margin-right: 8px;
            border-radius: 2px;
        }
        .mission-btn {
            width: 100%;
            padding: 12px 20px;
            border: none;
            border-radius: 6px;
            background: #4ade80;
            color: #000;
            font-weight: 600;
            cursor: pointer;
            transition: background 0.2s;
        }
        .mission-btn:hover { background: #22c55e; }
        .mission-btn:disabled {
            background: #444;
            color: #888;
            cursor: not-allowed;
        }
        .mission-btn.stop {
            background: #f87171;
        }
        .mission-btn.stop:hover { background: #ef4444; }
    </style>
</head>
<body>
    <div class="header">
        <h1>Hexapod Mission Control</h1>
    </div>
    <div class="container">
        <div class="main-grid">
            <div class="panel video-panel">
                <div class="panel-header">Camera + Face Detection</div>
                <img src="/stream/color" alt="Camera Feed">
            </div>
            <div class="panel video-panel">
                <div class="panel-header">Lidar (360&deg;, robot facing up)</div>
                <img src="/stream/lidar" alt="Lidar View">
            </div>
            <div class="panel status-panel">
                <div class="stat">
                    <span class="stat-label">Power Mode</span>
                    <span class="stat-value" id="power-mode">--</span>
                </div>
                <div class="stat">
                    <span class="stat-label">Load Battery</span>
                    <span class="stat-value" id="load-voltage">--</span>
                </div>
                <div class="stat">
                    <span class="stat-label">Ctrl Battery</span>
                    <span class="stat-value" id="ctrl-voltage">--</span>
                </div>
                <div class="stat">
                    <span class="stat-label">Nearest return</span>
                    <span class="stat-value" id="scan-status">--</span>
                </div>
                <div class="stat">
                    <span class="stat-label">Map</span>
                    <span class="stat-value" id="map-status">--</span>
                </div>
                <div class="stat">
                    <span class="stat-label">Faces</span>
                    <span class="stat-value" id="face-count">0</span>
                </div>
                <div class="faces" id="recognized-faces"></div>
                <div class="legend">
                    <strong>Lidar view:</strong>
                    <div class="legend-item">
                        <span class="legend-color" style="background: #0ff;"></span>
                        <span>Return in the current scan</span>
                    </div>
                    <div class="legend-item">
                        <span class="legend-color" style="background: #00dc00;"></span>
                        <span>The robot, 0.24 m radius, facing up</span>
                    </div>
                </div>
            </div>
            <div class="panel status-panel" style="grid-column: span 2;">
                <div class="panel-header" style="background: #0f3460; margin: -15px -15px 15px -15px; padding: 10px 15px;">Mission Control</div>
                <div style="display: flex; gap: 15px; flex-wrap: wrap;">
                    <div style="flex: 1; min-width: 200px;">
                        <div class="stat">
                            <span class="stat-label">Autonomy State</span>
                            <span class="stat-value" id="autonomy-state">--</span>
                        </div>
                        <div class="stat">
                            <span class="stat-label">SLAM Mode</span>
                            <span class="stat-value" id="slam-mode">--</span>
                        </div>
                        <div class="stat">
                            <span class="stat-label">Mission</span>
                            <span class="stat-value" id="mission-status">--</span>
                        </div>
                        <div class="stat">
                            <span class="stat-label">Exploration</span>
                            <span class="stat-value" id="exploration-progress">--</span>
                        </div>
                    </div>
                    <div style="flex: 1; min-width: 200px;">
                        <div style="display: flex; flex-direction: column; gap: 8px;">
                            <button class="mission-btn" onclick="startMission('explore')">Start Exploration</button>
                            <button class="mission-btn" onclick="startMission('return_home')">Return Home</button>
                            <button class="mission-btn stop" onclick="stopMission()">Stop Mission</button>
                        </div>
                        <div id="mission-result" style="margin-top: 10px; font-size: 0.85em; color: #888;"></div>
                    </div>
                </div>
            </div>
            <div class="panel video-panel" style="grid-column: span 1;">
                <div class="panel-header">Map (lidar)</div>
                <img src="/stream/map" alt="SLAM Map" style="width: 100%; height: auto;">
                <div style="padding: 8px 12px; font-size: 0.8em; color: #888;">
                    <span style="color: #00dc00;">Green</span>: the robot (circle 0.24 m, the radius Nav2 plans with) and its heading.
                    <span style="color: #00ffff;">Cyan</span>: what the lidar sees now.
                    <span style="color: #c80000;">Red</span>: obstacle.
                    <span style="color: #fa823c;">Orange</span>: too close to one for the body to pass.
                    Light grey: free. Dark: unknown.
                </div>
            </div>
        </div>
    </div>
    <script>
        function updateStatus() {
            fetch('/status')
                .then(r => r.json())
                .then(data => {
                    const loadV = data.battery.load;
                    const ctrlV = data.battery.control;

                    document.getElementById('load-voltage').textContent = loadV + 'V';
                    document.getElementById('ctrl-voltage').textContent = ctrlV + 'V';

                    const mode = document.getElementById('power-mode');
                    if (loadV < 0.5) {
                        mode.textContent = 'USB';
                        mode.className = 'stat-value info';
                    } else if (loadV >= 7.0) {
                        mode.textContent = 'Battery OK';
                        mode.className = 'stat-value good';
                    } else if (loadV >= 6.5) {
                        mode.textContent = 'Battery Low';
                        mode.className = 'stat-value warn';
                    } else {
                        mode.textContent = 'Critical!';
                        mode.className = 'stat-value bad';
                    }

                    // SLAM status
                    const scanEl = document.getElementById('scan-status');
                    scanEl.textContent = data.slam.scan_available
                        ? data.slam.scan_nearest.toFixed(2) + ' m at ' + data.slam.scan_nearest_bearing + '\u00b0'
                        : 'No Data';
                    scanEl.className = 'stat-value ' + (data.slam.scan_available ? 'good' : '');

                    const mapEl = document.getElementById('map-status');
                    mapEl.textContent = data.slam.map_available ? 'Active' : 'Not Running';
                    mapEl.className = 'stat-value ' + (data.slam.map_available ? 'good' : '');

                    // Faces
                    document.getElementById('face-count').textContent = data.faces.count;

                    const facesDiv = document.getElementById('recognized-faces');
                    if (data.faces.recognized.length > 0) {
                        facesDiv.innerHTML = '<strong>Recognized:</strong><br>' +
                            data.faces.recognized.map(n =>
                                '<span class="face-tag">' + n + '</span>'
                            ).join('');
                    } else {
                        facesDiv.innerHTML = '';
                    }

                    // Autonomy state
                    if (data.autonomy) {
                        const stateEl = document.getElementById('autonomy-state');
                        stateEl.textContent = data.autonomy.state_name || '--';
                        stateEl.className = 'stat-value ' + (
                            data.autonomy.state_name === 'exploring' ? 'good' :
                            data.autonomy.state_name === 'waiting_for_mission' ? 'info' :
                            data.autonomy.state_name === 'error' ? 'bad' : ''
                        );

                        document.getElementById('slam-mode').textContent =
                            data.autonomy.slam_mode || '--';

                        const missionEl = document.getElementById('mission-status');
                        if (data.autonomy.mission_active) {
                            missionEl.textContent = 'Active: ' + data.autonomy.mission_id;
                            missionEl.className = 'stat-value good';
                        } else if (data.autonomy.mission_timeout_remaining > 0) {
                            missionEl.textContent = 'Waiting (' +
                                Math.round(data.autonomy.mission_timeout_remaining) + 's)';
                            missionEl.className = 'stat-value info';
                        } else {
                            missionEl.textContent = 'None';
                            missionEl.className = 'stat-value';
                        }

                        const progressEl = document.getElementById('exploration-progress');
                        if (data.autonomy.state_name === 'exploring') {
                            progressEl.textContent = data.autonomy.exploration_progress + '%';
                            progressEl.className = 'stat-value good';
                        } else {
                            progressEl.textContent = '--';
                            progressEl.className = 'stat-value';
                        }
                    }
                })
                .catch(e => console.error('Status update failed:', e));
        }

        function startMission(missionType) {
            document.getElementById('mission-result').textContent = 'Starting...';
            fetch('/api/mission/start', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({mission_type: missionType})
            })
            .then(r => r.json())
            .then(data => {
                document.getElementById('mission-result').textContent =
                    data.accepted ? 'Started: ' + data.mission_id : 'Failed: ' + data.message;
            })
            .catch(e => {
                document.getElementById('mission-result').textContent = 'Error: ' + e;
            });
        }

        function stopMission() {
            document.getElementById('mission-result').textContent = 'Stopping...';
            fetch('/api/mission/stop', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({return_home: false})
            })
            .then(r => r.json())
            .then(data => {
                document.getElementById('mission-result').textContent =
                    data.success ? 'Stopped' : 'Failed: ' + data.message;
            })
            .catch(e => {
                document.getElementById('mission-result').textContent = 'Error: ' + e;
            });
        }

        setInterval(updateStatus, 2000);
        updateStatus();
    </script>
</body>
</html>
'''

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/stream/color')
def stream_color():
    return Response(dashboard_node.generate_mjpeg(dashboard_node.get_frame_with_overlay),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/stream/lidar')
def stream_lidar():
    return Response(dashboard_node.generate_mjpeg(dashboard_node.get_lidar_frame),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/stream/map')
def stream_map():
    return Response(dashboard_node.generate_mjpeg(dashboard_node.get_map_frame),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/stream')
def stream_legacy():
    """Legacy endpoint for backwards compatibility"""
    return stream_color()

@app.route('/status')
def status():
    return json.dumps(dashboard_node.get_status())


@app.route('/api/mission/start', methods=['POST'])
def api_start_mission():
    """Start a mission via API"""
    data = request.get_json(silent=True) or {}
    mission_type = data.get('mission_type', 'explore')
    try:
        timeout = float(data.get('timeout_sec', 0.0))
    except (TypeError, ValueError):
        return json.dumps({'accepted': False,
                           'message': 'timeout_sec must be a number'}), 400

    result = dashboard_node.call_start_mission(mission_type, timeout)
    return json.dumps(result)


@app.route('/api/mission/stop', methods=['POST'])
def api_stop_mission():
    """Stop current mission via API"""
    data = request.get_json() or {}
    return_home = data.get('return_home', False)

    result = dashboard_node.call_stop_mission(return_home)
    return json.dumps(result)


@app.route('/api/autonomy/state')
def api_autonomy_state():
    """Get autonomy state via API"""
    status_data = dashboard_node.get_status()
    return json.dumps(status_data.get('autonomy', {}))


def main(args=None):
    global dashboard_node

    rclpy.init(args=args)
    dashboard_node = WebDashboard()

    flask_thread = threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=dashboard_node.port,
                               threaded=True, use_reloader=False),
        daemon=True
    )
    flask_thread.start()

    dashboard_node.get_logger().info(
        f'Dashboard available at http://localhost:{dashboard_node.port}'
    )

    executor = MultiThreadedExecutor()
    executor.add_node(dashboard_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        dashboard_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
