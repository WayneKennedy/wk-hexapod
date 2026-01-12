#!/usr/bin/env python3
"""
Web Dashboard for Hexapod Robot

Provides a lightweight web interface with:
- MJPEG camera stream with face detection overlay
- Depth visualization
- SLAM map display (when available)
- Robot status display

Access at http://<robot-ip>:8080
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
import numpy as np
import cv2
import threading
from flask import Flask, Response, render_template_string
import json

# Try to import face detection messages
try:
    from hexapod_interfaces.msg import FaceArray
    FACE_MSGS_AVAILABLE = True
except ImportError:
    FACE_MSGS_AVAILABLE = False


class WebDashboard(Node):
    def __init__(self):
        super().__init__('web_dashboard')

        # Parameters
        self.declare_parameter('port', 8080)
        self.declare_parameter('quality', 80)
        self.port = self.get_parameter('port').value
        self.quality = self.get_parameter('quality').value

        # State
        self.current_frame = None
        self.current_depth = None
        self.current_map = None
        self.current_faces = []
        self.battery_voltages = [0.0, 0.0]
        self.frame_lock = threading.Lock()
        self.depth_lock = threading.Lock()
        self.map_lock = threading.Lock()

        # Subscribe to color camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Subscribe to depth camera
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        # Subscribe to SLAM map (when available)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

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
            else:
                return

            with self.frame_lock:
                self.current_frame = frame
        except Exception as e:
            self.get_logger().warn(f'Failed to process image: {e}')

    def depth_callback(self, msg):
        """Convert depth image to colorized visualization"""
        try:
            # RealSense depth is 16-bit unsigned (millimeters)
            if msg.encoding == '16UC1':
                depth = np.frombuffer(msg.data, dtype=np.uint16)
                depth = depth.reshape((msg.height, msg.width))

                # Normalize to 0-255 (clip at 4m max)
                depth_normalized = np.clip(depth / 4000.0 * 255, 0, 255).astype(np.uint8)

                # Apply colormap (TURBO gives nice depth visualization)
                depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_TURBO)

                # Mark invalid (0) pixels as black
                depth_colored[depth == 0] = [0, 0, 0]

                with self.depth_lock:
                    self.current_depth = depth_colored
            else:
                self.get_logger().warn(f'Unsupported depth encoding: {msg.encoding}')
        except Exception as e:
            self.get_logger().warn(f'Failed to process depth: {e}')

    def map_callback(self, msg):
        """Convert occupancy grid to image"""
        try:
            # Convert occupancy grid to image
            width = msg.info.width
            height = msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((height, width))

            # Create RGB image
            # -1 (unknown) -> gray, 0 (free) -> white, 100 (occupied) -> black
            map_img = np.zeros((height, width, 3), dtype=np.uint8)

            # Unknown = dark gray
            map_img[data == -1] = [40, 40, 40]
            # Free = light gray/white
            map_img[data == 0] = [200, 200, 200]
            # Occupied = colored based on certainty
            occupied_mask = data > 0
            map_img[occupied_mask, 0] = 0  # B
            map_img[occupied_mask, 1] = 0  # G
            map_img[occupied_mask, 2] = np.clip(data[occupied_mask] * 2.5, 0, 255).astype(np.uint8)  # R

            # Flip vertically (ROS maps have origin at bottom-left)
            map_img = cv2.flip(map_img, 0)

            with self.map_lock:
                self.current_map = map_img
        except Exception as e:
            self.get_logger().warn(f'Failed to process map: {e}')

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

    def get_depth_frame(self):
        """Get colorized depth frame"""
        with self.depth_lock:
            if self.current_depth is None:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, 'No Depth Data', (200, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                return frame
            return self.current_depth.copy()

    def get_map_frame(self):
        """Get SLAM map image"""
        with self.map_lock:
            if self.current_map is None:
                frame = np.zeros((400, 400, 3), dtype=np.uint8)
                cv2.putText(frame, 'No SLAM Map', (100, 200),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 100), 2)
                cv2.putText(frame, '(RTAB-Map not running)', (60, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 80, 80), 1)
                return frame
            return self.current_map.copy()

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

        with self.depth_lock:
            has_depth = self.current_depth is not None

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
                'depth_available': has_depth
            }
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
                <div class="panel-header">Depth Perception</div>
                <img src="/stream/depth" alt="Depth View">
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
                    <span class="stat-label">Depth Stream</span>
                    <span class="stat-value" id="depth-status">--</span>
                </div>
                <div class="stat">
                    <span class="stat-label">SLAM Map</span>
                    <span class="stat-value" id="map-status">--</span>
                </div>
                <div class="stat">
                    <span class="stat-label">Faces</span>
                    <span class="stat-value" id="face-count">0</span>
                </div>
                <div class="faces" id="recognized-faces"></div>
                <div class="legend">
                    <strong>Depth Legend:</strong>
                    <div class="legend-item">
                        <span class="legend-color" style="background: linear-gradient(90deg, #30123b, #7a0403, #f66b19, #f7e425);"></span>
                        <span>Near (blue) to Far (yellow)</span>
                    </div>
                    <div class="legend-item">
                        <span class="legend-color" style="background: #000;"></span>
                        <span>No data / out of range</span>
                    </div>
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
                    const depthEl = document.getElementById('depth-status');
                    depthEl.textContent = data.slam.depth_available ? 'Active' : 'No Data';
                    depthEl.className = 'stat-value ' + (data.slam.depth_available ? 'good' : '');

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
                })
                .catch(e => console.error('Status update failed:', e));
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

@app.route('/stream/depth')
def stream_depth():
    return Response(dashboard_node.generate_mjpeg(dashboard_node.get_depth_frame),
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
