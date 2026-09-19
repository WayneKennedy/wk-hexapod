#!/usr/bin/env python3
"""
Head Controller for Hexapod Robot

The single owner of the head pan/tilt servos. The head carries the camera and
the HC-SR04, so looking is done here, with two small servos, instead of by
turning the body on eighteen.

Behaviours, highest priority first:
- survey:  the LookAround action; full-width pan sweeps, body still.
- look_at: /head/look_at (PointStamped) holds the gaze on a point for
           look_at_hold_sec, pan and tilt.
- scan:    the default. The pan sweeps a sector centred on where the body is
           about to go: Nav2's pure-pursuit carrot (/lookahead_point) while it
           is fresh, else the turn direction from /cmd_vel, else straight
           ahead. Tilt is held level so the sonar reads horizontally.

Outputs: /head_command [pan, tilt] in servo degrees (to servo_driver), and
head_pan_joint / head_tilt_joint on /joint_states at update_rate for
robot_state_publisher. Joint positions follow a slew-rate model of the servo,
not the command, so the ultrasonic frame's TF is where the sensor was.

The head is only driven between a home/stand and a relax: it goes quiet on
/pose_command 'relax' or /servo_relax true, like the legs.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Bool, Float64MultiArray, String
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401  registers PointStamped with tf2

from hexapod_interfaces.action import LookAround


class HeadController(Node):

    def __init__(self):
        super().__init__('head_controller')

        # Servo geometry. Servo degrees; relative angles are + left / + up.
        self.declare_parameter('pan_center', 90.0)
        self.declare_parameter('tilt_center', 90.0)
        self.declare_parameter('pan_direction', 1.0)
        self.declare_parameter('tilt_direction', 1.0)
        self.declare_parameter('pan_limit_left', 40.0)
        self.declare_parameter('pan_limit_right', 40.0)
        self.declare_parameter('tilt_limit_up', 20.0)
        self.declare_parameter('tilt_limit_down', 20.0)
        self.declare_parameter('slew_rate', 300.0)
        # Scan behaviour
        self.declare_parameter('scan_half_width', 30.0)
        self.declare_parameter('scan_step', 10.0)
        self.declare_parameter('dwell_sec', 0.15)
        self.declare_parameter('tilt_level', 0.0)
        self.declare_parameter('lookahead_timeout_sec', 1.0)
        self.declare_parameter('turn_threshold', 0.05)
        self.declare_parameter('look_at_hold_sec', 3.0)
        self.declare_parameter('pan_joint_x', 0.10)
        self.declare_parameter('survey_timeout_sec', 20.0)
        self.declare_parameter('update_rate', 20.0)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.pan_center = gp('pan_center')
        self.tilt_center = gp('tilt_center')
        self.pan_dir = gp('pan_direction')
        self.tilt_dir = gp('tilt_direction')
        self.pan_min = -gp('pan_limit_right')
        self.pan_max = gp('pan_limit_left')
        self.tilt_min = -gp('tilt_limit_down')
        self.tilt_max = gp('tilt_limit_up')
        self.slew = gp('slew_rate')
        self.half_width = gp('scan_half_width')
        self.step = gp('scan_step')
        self.dwell = gp('dwell_sec')
        self.tilt_level = gp('tilt_level')
        self.lookahead_timeout = gp('lookahead_timeout_sec')
        self.turn_threshold = gp('turn_threshold')
        self.look_at_hold = gp('look_at_hold_sec')
        self.pan_joint_x = gp('pan_joint_x')
        self.survey_timeout = gp('survey_timeout_sec')
        update_rate = gp('update_rate')

        self._lock = threading.Lock()
        self.active = False
        # Modelled (actual) and commanded relative angles, degrees
        self.pan = 0.0
        self.tilt = 0.0
        self.pan_target = 0.0
        self.tilt_target = 0.0
        self.reached_at = None
        self.scan_dir = 1.0
        self.last_tick = time.monotonic()
        # Gaze inputs
        self.lookahead = None  # (bearing deg, monotonic time)
        self.turn = 0.0
        self.look_at = None    # (pan, tilt, until monotonic)
        # Survey state (set by the action, stepped by the timer)
        self.surveying = False
        self.survey_points = []
        self.survey_total = 0
        self.survey_readings = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cb_group = ReentrantCallbackGroup()
        self.head_pub = self.create_publisher(Float64MultiArray, 'head_command', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.create_subscription(String, 'pose_command', self._pose_cb, 10)
        self.create_subscription(Bool, 'servo_relax', self._relax_cb, 10)
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(PointStamped, 'lookahead_point', self._lookahead_cb, 10)
        self.create_subscription(PointStamped, 'head/look_at', self._look_at_cb, 10)
        self.create_subscription(Range, 'ultrasonic/range', self._range_cb, 10)

        self._action_server = ActionServer(
            self, LookAround, 'look_around',
            execute_callback=self._survey_execute,
            goal_callback=self._survey_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.cb_group)

        self.timer = self.create_timer(1.0 / update_rate, self._tick)
        self.get_logger().info(
            f'Head controller started: pan {self.pan_min:+.0f}..{self.pan_max:+.0f} deg, '
            f'scan +/-{self.half_width:.0f} deg in {self.step:.0f} deg steps')

    # ----- inputs -----

    def _pose_cb(self, msg):
        cmd = msg.data.lower().strip()
        with self._lock:
            if cmd in ('home', 'stand'):
                if not self.active:
                    self.active = True
                    self._command(self.pan_target, self.tilt_target)
            elif cmd == 'relax':
                self.active = False

    def _relax_cb(self, msg):
        if msg.data:
            with self._lock:
                self.active = False

    def _cmd_vel_cb(self, msg):
        self.turn = msg.angular.z if abs(msg.linear.x) < 0.02 else 0.0

    def _lookahead_cb(self, msg):
        point = self._to_base(msg)
        if point is not None:
            bearing = math.degrees(math.atan2(point[1], point[0] - self.pan_joint_x))
            self.lookahead = (bearing, time.monotonic())

    def _look_at_cb(self, msg):
        point = self._to_base(msg)
        if point is None:
            return
        dx = point[0] - self.pan_joint_x
        pan = math.degrees(math.atan2(point[1], dx))
        tilt = math.degrees(math.atan2(point[2], math.hypot(dx, point[1])))
        with self._lock:
            self.look_at = (pan, tilt, time.monotonic() + self.look_at_hold)

    def _range_cb(self, _msg):
        with self._lock:
            if self.surveying:
                self.survey_readings += 1

    def _to_base(self, msg):
        if msg.header.frame_id in ('', 'base_link'):
            return (msg.point.x, msg.point.y, msg.point.z)
        try:
            out = self.tf_buffer.transform(msg, 'base_link', timeout=Duration(seconds=0.0))
            return (out.point.x, out.point.y, out.point.z)
        except TransformException as e:
            self.get_logger().debug(f'Cannot place gaze point: {e}')
            return None

    # ----- the loop -----

    def _tick(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now
        with self._lock:
            self._slew(dt, now)
            if self.active and self.reached_at is not None and now - self.reached_at >= self.dwell:
                self._next_target(now)
            pan, tilt = self.pan, self.tilt
        self._publish_joints(pan, tilt)

    def _slew(self, dt, now):
        """Advance the servo model toward the target at the slew rate."""
        if not self.active:
            return
        max_step = self.slew * dt
        moved = False
        for attr, target in (('pan', self.pan_target), ('tilt', self.tilt_target)):
            cur = getattr(self, attr)
            delta = target - cur
            if abs(delta) > 1e-6:
                setattr(self, attr, cur + max(-max_step, min(max_step, delta)))
                moved = True
        if moved:
            self.reached_at = None
        elif self.reached_at is None:
            self.reached_at = now

    def _next_target(self, now):
        if self.survey_points:
            pan, tilt = self.survey_points.pop(0), self.tilt_level
        elif self.look_at is not None and now < self.look_at[2]:
            pan, tilt = self.look_at[0], self.look_at[1]
        else:
            self.look_at = None
            pan, tilt = self._scan_step(self._gaze_centre(now)), self.tilt_level
        pan = max(self.pan_min, min(self.pan_max, pan))
        tilt = max(self.tilt_min, min(self.tilt_max, tilt))
        if abs(pan - self.pan_target) > 1e-6 or abs(tilt - self.tilt_target) > 1e-6:
            self._command(pan, tilt)
        else:
            self.reached_at = now  # hold: re-evaluate after another dwell

    def _gaze_centre(self, now):
        if self.lookahead is not None and now - self.lookahead[1] < self.lookahead_timeout:
            return self.lookahead[0]
        if abs(self.turn) > self.turn_threshold:
            # Turning in place: look into the turn before the body gets there
            return self.pan_max if self.turn > 0 else self.pan_min
        return 0.0

    def _scan_step(self, centre):
        lo = max(self.pan_min, centre - self.half_width)
        hi = min(self.pan_max, centre + self.half_width)
        if lo >= hi:
            return lo
        nxt = self.pan_target + self.scan_dir * self.step
        if nxt > hi:
            self.scan_dir = -1.0
            nxt = hi if self.pan_target < hi else hi - self.step
        elif nxt < lo:
            self.scan_dir = 1.0
            nxt = lo if self.pan_target > lo else lo + self.step
        return max(lo, min(hi, nxt))

    def _command(self, pan, tilt):
        self.pan_target, self.tilt_target = pan, tilt
        self.reached_at = None
        msg = Float64MultiArray()
        msg.data = [self.pan_center + self.pan_dir * pan,
                    self.tilt_center + self.tilt_dir * tilt]
        self.head_pub.publish(msg)

    def _publish_joints(self, pan, tilt):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['head_pan_joint', 'head_tilt_joint']
        # URDF: pan about +z (left positive), tilt about +y (positive = down)
        msg.position = [math.radians(pan), -math.radians(tilt)]
        self.joint_pub.publish(msg)

    # ----- LookAround action: the survey -----

    def _survey_goal(self, _goal):
        with self._lock:
            if self.surveying:
                self.get_logger().warn('Rejecting look_around: a survey is running')
                return GoalResponse.REJECT
            if not self.active:
                self.get_logger().warn('Rejecting look_around: head is relaxed')
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _survey_execute(self, goal_handle):
        sweeps = max(1, goal_handle.request.sweeps)
        timeout = goal_handle.request.timeout_sec or self.survey_timeout
        n = max(1, int(round((self.pan_max - self.pan_min) / self.step)))
        one_way = [self.pan_min + i * (self.pan_max - self.pan_min) / n for i in range(n + 1)]
        points = []
        for _ in range(sweeps):
            points += one_way + one_way[-2::-1]
        with self._lock:
            self.surveying = True
            self.survey_points = list(points)
            self.survey_total = len(points)
            self.survey_readings = 0
        self.get_logger().info(f'Survey: {sweeps} sweep(s), {len(points)} positions')

        result = LookAround.Result()
        feedback = LookAround.Feedback()
        deadline = time.monotonic() + timeout
        outcome = 'succeed'
        while True:
            with self._lock:
                remaining = len(self.survey_points)
                settled = self.reached_at is not None
                feedback.pan_deg = float(self.pan)
                readings = self.survey_readings
                active = self.active
            if remaining == 0 and settled:
                break
            if goal_handle.is_cancel_requested:
                outcome = 'cancel'
                break
            if not active or time.monotonic() > deadline:
                outcome = 'abort'
                break
            feedback.progress = float(1.0 - remaining / self.survey_total)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)

        with self._lock:
            self.surveying = False
            self.survey_points = []
            readings = self.survey_readings
        result.readings = readings
        result.success = outcome == 'succeed'
        if outcome == 'succeed':
            result.message = f'Survey complete: {readings} ranges'
            goal_handle.succeed()
        elif outcome == 'cancel':
            result.message = 'Canceled'
            goal_handle.canceled()
        else:
            result.message = 'Head relaxed' if not active else 'Timeout'
            goal_handle.abort()
        self.get_logger().info(result.message)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = HeadController()
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
