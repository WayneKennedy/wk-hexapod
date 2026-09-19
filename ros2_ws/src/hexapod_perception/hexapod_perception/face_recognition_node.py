#!/usr/bin/env python3
"""
Face Recognition Node for Hexapod Robot

Detects and recognizes faces in camera images.
Publishes face detection results and events.
Provides services for training face recognition.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest
import numpy as np
import os
import time

# Face recognition imports with fallback
FACE_RECOGNITION_AVAILABLE = False
try:
    import face_recognition
    import cv2
    FACE_RECOGNITION_AVAILABLE = True
except ImportError:
    pass

# Import custom messages and services
from hexapod_interfaces.msg import Face, FaceArray, FaceEvent
from hexapod_interfaces.srv import CaptureTrainingImage, TrainModel, ListKnownFaces

# Import storage helper
from hexapod_perception.face_storage import FaceStorage


class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition_node')

        # Declare parameters
        self.declare_parameter('face_recognition.process_every_n_frames', 10)
        self.declare_parameter('face_recognition.detection_model', 'hog')
        self.declare_parameter('face_recognition.recognition_tolerance', 0.6)
        self.declare_parameter('face_recognition.min_face_size', 20)
        self.declare_parameter(
            'face_recognition.storage_path',
            '~/.hexapod/face_recognition'
        )
        self.declare_parameter('face_recognition.enable_events', True)
        self.declare_parameter('face_recognition.unknown_face_cooldown', 5.0)

        # Get parameters
        self.process_every_n = self.get_parameter(
            'face_recognition.process_every_n_frames'
        ).value
        self.detection_model = self.get_parameter(
            'face_recognition.detection_model'
        ).value
        self.tolerance = self.get_parameter(
            'face_recognition.recognition_tolerance'
        ).value
        self.min_face_size = self.get_parameter(
            'face_recognition.min_face_size'
        ).value
        self.storage_path = os.path.expanduser(
            self.get_parameter('face_recognition.storage_path').value
        )
        self.enable_events = self.get_parameter(
            'face_recognition.enable_events'
        ).value
        self.unknown_cooldown = self.get_parameter(
            'face_recognition.unknown_face_cooldown'
        ).value

        # State
        self.frame_count = 0
        self.known_encodings = {}  # {name: [encoding1, encoding2, ...]}
        self.known_names = []  # Flattened list for face_recognition.compare
        self.known_encoding_list = []  # Corresponding encodings
        self.last_frame = None  # Store last frame for training capture
        self.last_frame_rgb = None  # RGB version of last frame
        self.capture_pending = None  # Person name if capture is pending
        self.last_unknown_time = {}  # Track cooldown per face region

        # Initialize storage
        if FACE_RECOGNITION_AVAILABLE:
            self.storage = FaceStorage(self.storage_path)
            self._load_encodings()
            self.get_logger().info(
                f'Face recognition initialized with {len(self.known_encodings)} '
                f'known identities'
            )
        else:
            self.storage = None
            self.get_logger().warn(
                'face_recognition library not available, running in simulation mode'
            )

        # Subscriber for camera images
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.faces_pub = self.create_publisher(
            FaceArray,
            'face_recognition/faces',
            10
        )
        self.events_pub = self.create_publisher(
            FaceEvent,
            'face_recognition/events',
            10
        )

        # Services
        self.capture_srv = self.create_service(
            CaptureTrainingImage,
            'face_recognition/capture_training_image',
            self.capture_training_callback
        )
        self.train_srv = self.create_service(
            TrainModel,
            'face_recognition/train_model',
            self.train_model_callback
        )
        self.list_srv = self.create_service(
            ListKnownFaces,
            'face_recognition/list_known_faces',
            self.list_known_faces_callback
        )

        self.get_logger().info('Face recognition node started')

    def _load_encodings(self):
        """Load encodings from storage and prepare for recognition"""
        self.known_encodings = self.storage.load_encodings()

        # Flatten for face_recognition.compare_faces
        self.known_names = []
        self.known_encoding_list = []

        for name, encodings in self.known_encodings.items():
            for enc in encodings:
                self.known_names.append(name)
                self.known_encoding_list.append(enc)

        self.get_logger().info(
            f'Loaded {len(self.known_encoding_list)} encodings for '
            f'{len(self.known_encodings)} identities'
        )

    def image_callback(self, msg: Image):
        """Process incoming camera frames"""
        self.frame_count += 1

        # Convert ROS Image to an RGB numpy array. camera_ros chooses the pixel
        # format from what the sensor offers, so the encoding is not fixed.
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding == 'rgb8':
                frame = buf.reshape((msg.height, msg.width, 3))
            elif msg.encoding == 'bgr8':
                frame = buf.reshape((msg.height, msg.width, 3))[:, :, ::-1]
            elif msg.encoding in ('rgba8', 'bgra8'):
                frame = buf.reshape((msg.height, msg.step // 4, 4))[:, :msg.width, :3]
                if msg.encoding == 'bgra8':
                    frame = frame[:, :, ::-1]
            else:
                self.get_logger().warn(f'Unsupported image encoding: {msg.encoding}')
                return
            self.last_frame = msg
            self.last_frame_rgb = np.ascontiguousarray(frame)
        except Exception as e:
            self.get_logger().warn(f'Failed to decode image: {e}')
            return

        # Handle pending capture request
        if self.capture_pending:
            self._do_capture(frame)
            return

        # Skip frames for performance
        if self.frame_count % self.process_every_n != 0:
            return

        if not FACE_RECOGNITION_AVAILABLE:
            return

        # Detect and recognize faces
        self._process_frame(frame, msg.header)

    def _process_frame(self, frame: np.ndarray, header):
        """Detect faces and perform recognition"""
        # Detect faces
        face_locations = face_recognition.face_locations(
            frame,
            model=self.detection_model
        )

        if not face_locations:
            return

        # Filter by minimum size
        filtered_locations = []
        for top, right, bottom, left in face_locations:
            width = right - left
            height = bottom - top
            if width >= self.min_face_size and height >= self.min_face_size:
                filtered_locations.append((top, right, bottom, left))

        if not filtered_locations:
            return

        # Get face encodings
        face_encodings = face_recognition.face_encodings(frame, filtered_locations)

        # Process each face
        faces_msg = FaceArray()
        faces_msg.header = header

        for location, encoding in zip(filtered_locations, face_encodings):
            top, right, bottom, left = location

            face = Face()
            face.header = header
            face.bounding_box = RegionOfInterest(
                x_offset=left,
                y_offset=top,
                width=right - left,
                height=bottom - top
            )
            face.confidence = 1.0  # face_recognition doesn't provide detection confidence

            # Try to recognize
            if self.known_encoding_list:
                name, distance = self._recognize_face(encoding)
                if name:
                    face.recognized = True
                    face.identity = name
                    face.recognition_distance = distance

                    # Publish known face event
                    if self.enable_events:
                        self._publish_event(
                            FaceEvent.KNOWN_FACE_DETECTED,
                            name,
                            location,
                            distance,
                            header
                        )
                else:
                    face.recognized = False
                    face.identity = ''
                    face.recognition_distance = distance if distance else 1.0

                    # Publish unknown face event (with cooldown)
                    if self.enable_events:
                        self._maybe_publish_unknown_event(location, header)
            else:
                face.recognized = False
                face.identity = ''
                face.recognition_distance = 1.0

                if self.enable_events:
                    self._maybe_publish_unknown_event(location, header)

            faces_msg.faces.append(face)

        self.faces_pub.publish(faces_msg)

    def _recognize_face(self, encoding: np.ndarray):
        """
        Recognize a face encoding against known faces.

        Returns:
            Tuple of (name, distance) or (None, min_distance)
        """
        if not self.known_encoding_list:
            return None, None

        # Compare against all known faces
        distances = face_recognition.face_distance(
            self.known_encoding_list,
            encoding
        )

        min_idx = np.argmin(distances)
        min_distance = distances[min_idx]

        if min_distance <= self.tolerance:
            return self.known_names[min_idx], float(min_distance)

        return None, float(min_distance)

    def _publish_event(self, event_type, identity, location, confidence, header):
        """Publish a face event"""
        top, right, bottom, left = location

        event = FaceEvent()
        event.header = header
        event.event_type = event_type
        event.identity = identity if identity else ''
        event.bounding_box = RegionOfInterest(
            x_offset=left,
            y_offset=top,
            width=right - left,
            height=bottom - top
        )
        event.confidence = confidence if confidence else 0.0

        self.events_pub.publish(event)

    def _maybe_publish_unknown_event(self, location, header):
        """Publish unknown face event with cooldown"""
        top, right, bottom, left = location
        center_x = (left + right) // 2
        center_y = (top + bottom) // 2

        # Use grid-based key for cooldown (divide image into regions)
        grid_x = center_x // 100
        grid_y = center_y // 100
        region_key = (grid_x, grid_y)

        current_time = time.time()
        last_time = self.last_unknown_time.get(region_key, 0)

        if current_time - last_time >= self.unknown_cooldown:
            self.last_unknown_time[region_key] = current_time
            self._publish_event(
                FaceEvent.UNKNOWN_FACE_DETECTED,
                '',
                location,
                0.0,
                header
            )

    def _do_capture(self, frame: np.ndarray):
        """Capture a training image for the pending person"""
        person_name = self.capture_pending
        self.capture_pending = None

        if not FACE_RECOGNITION_AVAILABLE:
            self.get_logger().warn('Cannot capture: face_recognition not available')
            return

        # Detect faces in current frame
        face_locations = face_recognition.face_locations(frame, model='hog')

        if not face_locations:
            self.get_logger().warn(f'No face detected for capture of {person_name}')
            return

        if len(face_locations) > 1:
            self.get_logger().warn(
                f'Multiple faces detected, using largest for {person_name}'
            )
            # Use the largest face
            face_locations = sorted(
                face_locations,
                key=lambda loc: (loc[2] - loc[0]) * (loc[1] - loc[3]),
                reverse=True
            )

        location = face_locations[0]
        success, message, total = self.storage.save_training_image(
            person_name,
            frame,
            face_location=location
        )

        if success:
            self.get_logger().info(
                f'Captured training image for {person_name} ({total} total)'
            )
        else:
            self.get_logger().error(f'Failed to capture for {person_name}: {message}')

    # Service callbacks

    def capture_training_callback(self, request, response):
        """Handle capture training image service request"""
        if not request.person_name:
            response.success = False
            response.message = 'Person name is required'
            response.total_images = 0
            return response

        if not FACE_RECOGNITION_AVAILABLE:
            response.success = False
            response.message = 'face_recognition library not available'
            response.total_images = 0
            return response

        if self.last_frame_rgb is None:
            response.success = False
            response.message = 'No camera frame available'
            response.total_images = 0
            return response

        # Detect face in last frame
        face_locations = face_recognition.face_locations(
            self.last_frame_rgb,
            model='hog'
        )

        if not face_locations:
            response.success = False
            response.message = 'No face detected in current frame'
            response.total_images = 0
            return response

        # Use largest face if multiple
        if len(face_locations) > 1:
            face_locations = sorted(
                face_locations,
                key=lambda loc: (loc[2] - loc[0]) * (loc[1] - loc[3]),
                reverse=True
            )

        location = face_locations[0]
        success, message, total = self.storage.save_training_image(
            request.person_name,
            self.last_frame_rgb,
            face_location=location
        )

        response.success = success
        response.message = message
        response.total_images = total

        if success:
            self.get_logger().info(
                f'Captured training image for {request.person_name} ({total} total)'
            )

        return response

    def train_model_callback(self, request, response):
        """Handle train model service request"""
        if not FACE_RECOGNITION_AVAILABLE:
            response.success = False
            response.message = 'face_recognition library not available'
            response.trained_identities = []
            response.total_encodings = 0
            return response

        training_images = self.storage.get_training_images()

        if not training_images:
            response.success = False
            response.message = 'No training images found'
            response.trained_identities = []
            response.total_encodings = 0
            return response

        self.get_logger().info('Training face recognition model...')

        new_encodings = {}
        total = 0

        for person_name, image_paths in training_images.items():
            person_encodings = []

            for image_path in image_paths:
                try:
                    # Load image
                    image = face_recognition.load_image_file(image_path)

                    # Get face encoding
                    encodings = face_recognition.face_encodings(image)

                    if encodings:
                        person_encodings.append(encodings[0])
                        total += 1
                    else:
                        self.get_logger().warn(
                            f'No face found in {image_path}'
                        )

                except Exception as e:
                    self.get_logger().error(
                        f'Failed to process {image_path}: {e}'
                    )

            if person_encodings:
                new_encodings[person_name] = person_encodings
                self.get_logger().info(
                    f'  {person_name}: {len(person_encodings)} encodings'
                )

        # Save and reload encodings
        if new_encodings:
            self.storage.save_encodings(new_encodings)
            self._load_encodings()

            response.success = True
            response.message = f'Trained model with {total} encodings'
            response.trained_identities = list(new_encodings.keys())
            response.total_encodings = total

            self.get_logger().info(
                f'Training complete: {len(new_encodings)} identities, '
                f'{total} encodings'
            )
        else:
            response.success = False
            response.message = 'No valid faces found in training images'
            response.trained_identities = []
            response.total_encodings = 0

        return response

    def list_known_faces_callback(self, request, response):
        """Handle list known faces service request"""
        identities = self.storage.list_identities() if self.storage else []

        response.identities = [name for name, _ in identities]
        response.image_counts = [count for _, count in identities]

        return response


def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
