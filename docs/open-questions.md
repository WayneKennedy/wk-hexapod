# Open Questions (pending decisions)

Unresolved. Resolve → move to [`decisions.md`](decisions.md). A recommendation is one
option with an argument attached, not a decision; do not build against one without the
owner deciding.

---

## Locomotion and navigation

- **OQ-19 — Mapping and obstacle sensing without depth.** (2026-09-18, opened by
  [DEC-25](decisions.md).) **Resolved 2026-09-19 on the robot**, merged 2026-09-21: the
  camera and ultrasonic drivers are back (DEC-25, implemented), the sonar feeds a
  range-sensor costmap layer that is the map ([DEC-28](decisions.md)), and the head owns
  looking ([DEC-27](decisions.md)). What it left open is OQ-20 to OQ-23.

- **OQ-16 — Look with the head before turning the body.** (Owner, 2026-09-15.)
  **Largely resolved 2026-09-19 by [DEC-27](decisions.md)**, and the sensor change made it
  urgent rather than cosmetic: the head now carries the only range sensor. The head scans
  ahead of the body's next move, the explorer surveys with the head at each frontier, and
  every body rotation that existed purely to look is gone. **Still open:** whether
  `use_rotate_to_heading` should also go. It is kept true because a 15° cone must be
  pointed along a path before the robot walks it, but the robot is omnidirectional and
  could strafe instead (+y unverified, DEC-22). Measure the battery cost per turn and the
  map growth per sweep on the floor before changing it.

- **OQ-03 — Collision monitor and costmap tuning on the floor.** The collision monitor
  takes the sonar (`/ultrasonic/range`) as its source since 2026-09-19. **First floor run
  (2026-09-15, [`test-log.md`](test-log.md)): the robot walked into obstacles and the
  monitor never stopped it**, when the source was the depth-derived `/scan`, whose minimum
  range exceeded the stop polygon. [DEC-25](decisions.md) changed the geometry with the
  sensors: the sonar reads from 0.03 m, the stop polygon reaches 0.32 m ahead (past the
  0.225 m foot tips), a slowdown polygon reaches 0.50 m, and both costmaps use a 0.24 m
  robot radius instead of the body's 0.15 m. **None of it is tested against a real
  obstacle**, and the sonar brings its own blind spots: one cone wherever the head points,
  nothing below its height, and specular loss on angled or soft surfaces. Measure the
  stopping distance at 0.05 m/s on the floor, then tune inflation against it.
  **2026-09-29, first contact on the sonar:** the robot walked into a TV stand while
  exploring; the monitor did issue stops, then flickered between stop and slowdown at a
  sonar range of 0.29–0.31 m, the edge of the 0.32 m stop polygon
  ([`test-log.md`](test-log.md)). The lidar's `/scan` is not a source yet (OQ-20).

- **OQ-05 — Return-to-home and semantic waypoints.** `return_home` exists as a mission
  type; the home pose is the map origin. No named places, no docking (Nav2's docking server
  runs with no docks defined).

- **OQ-12 — Two writers to the head servos.** Resolved 2026-09-19 by
  [DEC-27](decisions.md): `head_controller` is the only writer; the leg controller sends
  NaN in the head slots of `/joint_commands` and `servo_driver` skips them.

- **OQ-13 — IMU filter conventions.** `imu_filter_madgwick` was added on 2026-09-09 to
  provide `/imu/data` (DEC-19). Whether its ENU yaw sign and frame match what the
  controller's complementary filter and balance loop assume has not been checked against
  the robot turning. Check before trusting yaw fusion. **Tilted and turned by hand,
  2026-10-01** ([`test-log.md`](test-log.md)): yaw sign right (90° anticlockwise read
  +53°, the turn's true size unmeasured); **tilting the nose down moved the controller's
  roll and tilting the left side down moved its pitch**, so the chip's axes sit 90° from
  `base_link` and nothing in the chain (URDF `imu_joint` `rpy="0 0 0"`, the filter, the
  controller) corrected it; the owner confirmed the order. **Fixed in the driver the same
  day:** `imu_driver` rotates accelerations and rates by `imu.mounting_yaw_deg` (−90) into
  body axes, so the URDF's `rpy="0 0 0"` is now right. **Verified 2026-10-02:** nose down
  reads pitch +50°, left side down roll −50°, as ROS expects
  ([`test-log.md`](test-log.md)). **Roll, pitch and the yaw sign are settled.** Left:
  the yaw magnitude after a turn of known size, and the gyro bias (both under OQ-36).

- **OQ-20 — The map drifts and does not survive the run.** (2026-09-19,
  [DEC-28](decisions.md).) `map → odom` is a static identity, so the map is only as good as
  gait odometry: it drifts with slip and yaw error, nothing closes a loop, and every boot
  starts an empty 12 × 12 m grid. Enough to explore a room; not enough to come back to one,
  which milestones 2 and 3 need ([`roadmap.md`](roadmap.md)). Options, none examined:
  accept it and keep maps within a run; put landmarks the mono camera can recognise (ArUco
  or AprilTag) on walls to correct `map → odom`; attempt scan matching on accumulated sonar
  sweeps (doubtful with a 15° cone); or fit a cheap 2D lidar, which would make
  `slam_toolbox` viable. Weigh against [OQ-22](#perception-and-sensing), and check
  [wk-inventory](https://github.com/WayneKennedy/wk-inventory/blob/main/docs/stock.md)
  before buying anything. **2026-09-23: the lidar route is chosen in principle, pending the
  owner's order** — a Slamtec RPLIDAR C1 (£57; datasheet facts and Jazzy driver status in
  [wk-robotics `common.md`](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/common.md#depth-which-kind-for-which-task)),
  for this robot first. Stock and invoices held no 2D lidar. Mount decided in principle:
  [DEC-30](decisions.md), a plate on extended Pi standoffs — open: the standoff length (scan
  plane clear of the head at full tilt and the coxa servos across their stroke), the tilt
  cost of 110 g up high, the USB route. Then `rplidar_ros` from apt,
  `slam_toolbox` in place of gait-only odometry, and whether OQ-22's landmark route is
  still needed. **2026-09-28: the C1 is fitted and publishing, and `slam_toolbox` builds
  `/map` from it on the bench** (DEC-31; apt's `rplidar_ros` could not drive the C1, so the
  driver is `sllidar_ros2` from source). **Still open:** (1) wiring SLAM into the boot stack
  — drop the static `map → odom`, give Nav2's global costmap `/map` as its static layer and
  `/scan` as an obstacle source, point the frontier explorer at `/map`, move the collision
  monitor to `/scan` (OQ-03), and decide what the sonar keeps (OQ-26); this supersedes
  DEC-28; (2) saving and reloading maps (`slam_toolbox` serialisation, and localization
  mode for `checking_map`/`localization_mode`); (3) whether gait odometry is good enough for
  scan matching while walking — measured only stationary; (4) CPU: `slam_toolbox` ~4.5 % and
  the driver ~4 % of a core at rest, load average ~5 without Nav2 (OQ-02).
  **2026-09-29:** the first battery run with the lidar fitted walked into a TV stand with
  `/scan` publishing and unused ([`test-log.md`](test-log.md)); wiring `/scan` into the
  costmaps and the collision monitor is the same step.
  **2026-09-29, done ([DEC-32](decisions.md)):** `/scan` feeds both costmaps and the
  collision monitor and `slam_toolbox` runs in the boot stack. **Still open:** saving a map
  and localizing against it on the next boot, which milestone 2 needs.

- **OQ-30 — The head tilt servo's gears slip; the head is disabled.** (Owner, 2026-09-29:
  the servo could be heard trying to lower with the gears jumping, during the battery runs
  in [`test-log.md`](test-log.md).) Pan and tilt are both off by the owner's instruction:
  `head_controller` `enabled: false` and `servo_driver` `servos.head_enabled: false`, two
  switches that must agree. **Consequences:** the servos are limp, so where the sonar and
  camera point is wherever the head rests, while TF places them at pan 0, tilt 0; the
  costmaps and the collision monitor read a fixed cone of unknown direction; the startup
  survey is rejected and the state machine explores without it. **Open:** the cause
  (stripped gear, or a load the servo cannot hold) and the remedy: replace the servo
  (check [wk-inventory](https://github.com/WayneKennedy/wk-inventory/blob/main/docs/stock.md)
  first), fix the head level mechanically, or remove the head (OQ-26). The lidar as the
  range source ([OQ-20](#locomotion-and-navigation)) removes the dependence either way.
  **2026-10-02, driven by hand on the battery** ([`test-log.md`](test-log.md)): **with
  LOAD on and CTRL off** (the Pi on USB) the tilt servo drove up but not down, with the
  commanded pulse read back from the PCA9685. **With both on it tracked both ways and
  repeated:** level at 65°, 18° below level at 45° (phone inclinometer), twice; down travel
  ends at 40° on the wiring and bracket; no gear noise. So the servo is not shown faulty.
  **Open:** why CTRL off stops it driving down (the ADC also read 0 V on both rails then).
  **Owner's hypothesis:** no common ground between the USB-powered Pi and the LOAD rail
  with CTRL off, so the servo signal is referenced to a shifted ground; pan behaving
  normally is not explained by it. Check with a meter: continuity Pi GND to LOAD battery
  negative (all off), and DC volts between them with LOAD on, CTRL off, the Pi on USB.
  Until settled, drive servos with CTRL on. Also open: whether the slipping of 09-29,
  with both on, recurs under the gait's shaking.

- **OQ-21 — The head has no feedback, so its joint states are a model.** (2026-09-19.)
  `head_controller` publishes head joint states from a slew-rate model (`slew_rate`,
  default 300 °/s, unmeasured), and that model is what places `ultrasonic_link` on the TF
  tree. If it is wrong, or a servo stalls, readings taken while the head moves land at the
  wrong bearing in the map. Also unverified: `pan_direction` (which way a rising servo
  angle turns the head) and the travel limits, kept at ±40° inside the vendor app's 50–180
  clamp. **2026-10-02:** pan is channel 1 and tilt channel 0, the reverse of the vendor's
  code (`hardware.yaml` corrected); a rising tilt angle raises the head (`tilt_direction`
  +1 holds); tilt level is servo 65°, not 90° (`tilt_center` set); tilting 15–20° up brings the head beside the lidar's base (OQ-26, OQ-30). Needs the battery and the owner: command known angles, watch the head, and check
  `/ultrasonic/range` against a target at a known bearing (the dashboard's sonar fan went
  on 2026-09-29). Cheapest mitigation if the
  model proves poor: only trust readings taken while the head is settled.

- **OQ-28 — The startup sequence sends `home` before the controller is listening.**
  (Found 2026-09-29, [`test-log.md`](test-log.md).) `startup_sequence` publishes `home` and
  `stand` on `/pose_command` on a fixed schedule, 4 s after it starts, and declares the
  robot safe without checking the result. `/pose_command` is volatile, so a controller that
  starts late never sees `home`, refuses `stand`, and ignores `/cmd_vel` for the rest of the
  run while the autonomy stack explores on paper. Three of the five starts in the robot's
  journal lost the race. **Resolved 2026-09-29** (commit `a077db8`; one battery boot,
  [`test-log.md`](test-log.md); the timeout paths are untested): the sequence waits, before its
  warning, until `hexapod_controller` and `servo_driver` both subscribe to `/pose_command`
  (`startup.controller_timeout`, 60 s); the controller publishes `/hexapod/initialized`;
  and the sequence goes on from `home` only once that is true (`startup.confirm_timeout`,
  5 s). Either timeout ends on a red rear LED with `/robot/initialized` unpublished, so the
  autonomy stack stays in `waiting_for_startup`. `hardware.launch.py` runs the sequence
  with no controller and therefore now ends red after 60 s. **Recovery on a robot left
  uninitialised** is `/robot/safe_startup`, which reruns the sequence; the robot walks off
  when it completes.

- **OQ-31 — Does the lidar see the robot itself?** (2026-09-29, opened by
  [DEC-32](decisions.md).) The scan plane is 0.16 m above `base_link`; the 2026-09-28 bench
  test had the legs limp and never tested a leg or the head in the plane. SLAM and the
  costmaps drop returns nearer than 0.25 m. The collision monitor drops nothing, and its
  stop polygon covers the body, so one return from a knee or the head would hold the robot
  stopped. `scripts/scan-near.py` reports the nearest returns by body bearing; run it
  standing and walking. **Standing, 2026-09-29: none.** Two 3–5 s samples, 720 beams at
  10 Hz: the nearest return was 0.34 m, behind the robot, and every other 30° sector was
  beyond 0.70 m ([`test-log.md`](test-log.md)). What stood 0.34 m behind was not
  identified; it read the same in both runs. **Standing, 13:18 and 13:29: none again.**
  Every near return lay on a straight edge 0.28–0.36 m astern, at the same ranges with
  the body at 30 mm and at 80 mm. **Walking: not measured.** The owner suspects the knees
  show in the gait and raised the stand for it ([DEC-34](decisions.md)). The dashboard's
  lidar view is in the body frame, so a return from the robot stays put in it while the
  room moves. **Closed 2026-10-02 at the 50 mm stand** ([`test-log.md`](test-log.md)):
  on a pillar with the legs free, walking at 0.05 and 0.10 m/s and turning in place, no
  return nearer than 0.45 m in any sector over 18 s of scans, and the owner watching saw
  the knees well below the plane. Legs in the air are unloaded; on the floor the body
  sags under load by an unmeasured amount, so the margin is not known, only that it
  exists.

- **OQ-34 — No frontier was reachable from where the robot stood.** (2026-09-29,
  [`test-log.md`](test-log.md).) With the map reset at the standing pose the planner
  reached points 0.3–0.5 m away, and every frontier goal failed with "no valid path
  found". In the global costmap the cells reachable from the robot were a pocket of
  0.79 m² (1.15 × 1.30 m) closed on all sides by inscribed cells, with no unknown cell on
  its edge; 156 of 1437 free cells were inside it. Returns stood within 0.73–1.22 m in
  every sector. **Unknown:** whether the robot was boxed in by furniture and feet, which
  the 0.24 m robot radius then closes, or whether some returns are not obstacles (a tilted
  scan plane meeting the floor, OQ-26). Test in open floor with the owner describing the
  room. **Second occurrence, 13:14 the same day:** a pocket of 0.37 m² with the robot's
  own cell at cost 96, while `/map` alone left 3.21 m² reachable and 284 cells beside
  unknown space. An edge stood 0.30 m astern, a wall 0.55 m to the left and objects 0.6 m
  to the right, so the returns were obstacles. **What closes the pocket is not
  established:** the 0.24 m radius in a passage that narrow, or obstacles marked while
  the heading was wrong ([OQ-36](#locomotion-and-navigation)); the costmap held 430 lethal
  cells against `/map`'s 278. `scripts/costmap-reach.py` reports the pocket. **Started
  within `PolygonStop` of anything, the robot does not move at all** (13:28).
  **2026-10-02: a frontier reached** ([`test-log.md`](test-log.md)), from a start with
  free floor ahead and 0.5 m each side. The next frontier lay up a passage 0.5–0.6 m
  wide, which the planner refuses at `robot_radius` 0.24 m with 0.35 m inflation, and
  the run ended in `error` after two more failed goals. **Still open:** the explorer
  keeps choosing frontiers it cannot reach (the goal's own cell was inscribed, 1.46 m
  from reachable space): it should drop goals whose cell the planner cannot reach, or
  plan to the nearest reachable cell; and three failed goals end exploration in `error`
  (`max_nav_failures`), where waiting and retrying would suit a room people move in.
  **Changed 2026-10-02** (`frontier_explorer`, commit `6d83602`): each frontier's
  goal is the reachable costmap cell nearest its cells (a flood fill over known cells below
  inscribed, as `scripts/costmap-reach.py`); frontiers with none within `max_goal_offset`
  (1.0 m) are skipped; "closest" is by path length; after `max_nav_failures` in a row, or
  with nothing reachable, it waits `retry_wait_sec` (30 s) and retries with the failed goals
  forgotten, so exploration ends only on no frontiers, the session timeout or a cancel.
  Checked on a synthetic map and on USB ([`test-log.md`](test-log.md)); **not on the
  floor.** **Found on USB:** a goal the robot cannot make progress on is held by Nav2's
  recovery cycle for 7 min 21 s, longer than the 300 s session (`exploration_timeout_sec`),
  which the explorer checks only between goals; there is no per-goal limit.

- **OQ-36 — Three headings, three answers.** (2026-09-29, 13:26 UTC,
  [`test-log.md`](test-log.md).) After one `backup` of about 0.2 m and nothing else
  commanded: `slam_toolbox` put the robot at −27.9° in the map, gait odometry said −48.9°
  and `/imu/data` +4.4°. SLAM's is the one checked against the room: the live scan lay on
  its map to within 0.5°. **Unknown:** how far the robot turned (the owner saw it; nobody
  measured), so which of odometry and the IMU is further out. The IMU's yaw sign is
  unverified (OQ-13), `odometry.turn_scale` was measured on a rubber mat at 0.3 rad/s, and
  the part may not be the one its library was written for
  ([OQ-35](#perception-and-sensing)). **Why it matters:** `slam_toolbox` corrects
  `map → odom` only when it takes a scan, every 0.2 m or 0.3 rad of odometry; between
  corrections the costmaps mark what the lidar sees at the odometry's heading.
  **Found in the code, 2026-10-01:** while walking, the fusion (weight 0.98 per 100 Hz
  sample) replaced the odometry heading with the IMU's *absolute* yaw within a few
  samples, and in the boot stack odometry is never synced to it (`home` arrives by topic,
  which does not reset odometry). The filter's absolute yaw is arbitrary, it starts
  wherever the gravity vector put it: −3° on 2026-09-29, −91° and −97° on two starts after
  the axis fix of 2026-10-01. So the first step of every run turned the odometry heading
  by that much, and the heading while walking was the IMU's, which then drifts standing
  still (−48.9° to +4.4° cannot both be right). **Changed the same day** (`07842a1`): the
  fusion follows the IMU's change in yaw since the walk began, anchored to the heading
  odometry held then. **Seen working on the bench, 2026-10-02:** a 10 s turn commanded
  with the body held on a pillar left the odometry heading at 1.5° (the IMU saw no turn)
  instead of the ~120° the gait integrated. **Wrinkle found there:** when the turn ended
  the heading jumped by 11.6°, one cycle's worth, so the last cycle's gait integration
  escapes the fusion once walking stops; on the floor the two mostly agree, so the jump
  is the per-cycle gait-versus-IMU difference, not 11.6°. Not fixed. **Largely resolved
  on the floor, 2026-10-02:** over 4 min and more than a full rotation SLAM and odometry
  stayed within 0–5° and the IMU's yaw change matched SLAM to 0.2°
  ([`test-log.md`](test-log.md)). The 53° for a hand turn on 10-01 was the hand turn.
  **Left:** the 0–5° gap is the gait's per-cycle integration between IMU samples and the
  end-of-walk jump above; neither matters at this size. **Measured 2026-10-02:** at rest the filter's yaw
  moves −0.21°/s (11.6° in 55 s) while the raw gyro z reads +0.1…+0.3°/s, a bias nothing
  removes: a one-minute walk inherits about 12° from it. **Bias removed the same day:**
  `imu_driver` averages 200 readings at start-up (and on `/imu/calibrate_gyro`), skips
  them if the robot moved, and subtracts the mean; the start-up bias read −5.39, +1.60,
  +0.10 °/s and the yaw then held to +0.006°/s over 100 s at rest. The start-up window is
  the first 2 s of the driver, before the legs home. **Next:** the movement calibration in
  [`operations.md`](operations.md#movement-calibration), then the three headings again
  after a turn of known size.

- **OQ-37 — Hold the scan plane level while walking, from the IMU.** (Owner, 2026-09-29:
  can tilt measured by the IMU correct all the legs during the gait, with the lidar's
  height and plane as a high-priority goal?) **What exists** (`controller.py`, read, never
  run: no entry in [`test-log.md`](test-log.md)): an incremental PID on roll and pitch that
  writes `body_orientation`, clamped to ±15°, and the gait applies `body_orientation` to
  every foot in each of its 64 frames per cycle. **What stops it working while walking:**
  the balance timer shares the gait's mutually exclusive callback group, and a gait cycle
  blocks that group for its whole second, so the loop runs only between cycles; it also
  writes the servos itself; and `balance.enabled: true` sets a flag without starting the
  timer, which only `/hexapod/enable_balance` does. **Height is not the IMU's to hold:**
  it measures tilt, not height, and on a flat floor the body's height is what the leg
  kinematics command. **Measured on the floor at 50 mm, 2026-10-02** ([`test-log.md`](test-log.md)): walking,
  roll sd 1.4° (−7.2° to +3.1°) and pitch sd 1.1° (−3.3° to +5.0°) against sd 0.2–0.3°
  standing; the period was not extracted. A 5° pitch at the 0.21 m scan height meets the
  floor at 2.4 m, inside the costmap's 3 m marking range, so the peaks can mark floor as
  obstacles; gating scans on tilt would remove that for little effort.
  **Frame fixed 2026-10-01 and verified 2026-10-02 (OQ-13):** nose down is positive pitch
  and left side down negative roll at the controller. Still unknown: which way a positive
  `body_orientation` roll or pitch tilts the body (needs servos), so the loop's sign is
  untested.
  **Limits to expect, none measured here:** the servos take a new pulse at 50 Hz and
  report nothing, and the Madgwick filter lags. If the rocking repeats with the gait
  phase, a correction keyed to the phase may do more than feedback. **Cheaper
  complement:** drop scans taken while the IMU reads a tilt above a threshold
  ([OQ-26](#perception-and-sensing)).

- **OQ-27 — `base_footprint` is a detached tree.** (Found 2026-09-28.) The URDF makes
  `base_footprint` the parent of `base_link` (+0.03 m), while the controller publishes
  `odom → base_link`; tf2 keeps one parent per frame, so `base_footprint` is its own root and
  `odom → base_footprint` fails ("not part of the same tree"). Nothing uses it today —
  `slam_params.yaml` sets `base_frame: base_link` for that reason, and no Nav2 parameter names
  it. Fix when anything needs a ground-plane frame: invert the joint (`base_link →
  base_footprint`, z −0.05 at the 50 mm stand of [DEC-34](decisions.md); the URDF still
  says 0.03).

## Compute

- **OQ-02 — CPU load.** (Measured with the D435i and RGB-D RTAB-Map, both gone under
  DEC-25; the load under OQ-19's replacement is unmeasured.) Load average ~10 on the four-core Pi 5 with the full boot stack
  (2026-09-09; RealSense point cloud already disabled), 15 in the first minutes after a
  cold boot and 23 while exploring on the battery (2026-09-15). At that load `planner_server` misses `bt_navigator`'s 20 ms
  `default_server_timeout` on the 1 Hz replan and every Nav2 goal aborts within seconds
  ([`test-log.md`](test-log.md), 2026-09-15); the timeout was raised to 1000 ms on
  2026-09-15 (DEC-21); the load is the cause and is still open. The hot Python nodes were
  `web_dashboard` (corrected 2026-09-18 from the code, not measured: JPEG encoding runs only in each connected client's MJPEG generator, but with no viewer the node still subscribes to raw colour and depth at 640×480×15 — about 23 MB/s through rclpy — converting every colour frame, normalising and colour-mapping every depth frame, and re-rendering every `/map`; the fix is to subscribe or convert only while a stream client is connected, and each connected stream re-encodes at 20 fps even when the frame is unchanged), `frontier_explorer`
  (frontier detection over the whole map each loop), `mission_server`, and `imu_driver`
  (100 Hz I2C polling). Options: subscribe to and convert dashboard streams only while a client
  is connected, rate-limit frontier detection, drop the IMU rate, or move a perception stage
  off the CPU per the family's
  [perception placement](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/common.md#perception-placement).
  **Owner's hypothesis to test (2026-09-18):** the Pi carries reflex work (the 18-servo
  gait) as well as intent, and a separate reflex MCU — as on wk-devastator — would free it.
  Evidence so far argues against it for CPU and for it on timing. Walking with autonomy off
  ran at load average 1.0, the servo driver 5 % busy and the gait thread mostly asleep; the
  load of 10–12 was measured with the servos unpowered, so no gait ran; the hot nodes above
  are intent-tier. But reflex loops on the Pi do run late: a 4.21 s gait cycle against
  1.0 s from wake latency, the odometry starvation behind DEC-09, and 607 extrapolation
  errors in 150 s ([`test-log.md`](test-log.md)). **Test:** per-process CPU (`pidstat` or
  `py-spy`) with the full stack, standing with servos powered, then walking a fixed
  `/cmd_vel`, then walking with autonomy off; the standing-to-walking difference is the
  reflex cost, and late gait cycles or TF extrapolation errors under load are the timing
  cost.

- **OQ-09 — A reflex tier retrofit.** An MCU (Teensy or RP2040, micro-ROS) owning the
  PCA9685 chips, MPU6050, buzzer and servo-power pin would remove the buzzer float, the
  scheduler sensitivity and the IMU polling from the Pi, and make the robot a second
  consumer of koala-bot's reflex firmware. The controller already publishes
  `/joint_commands`, so it would not change. Cost: a board, a harness into the shield's
  I2C and GPIO, and firmware. Not planned; recorded because the family direction argues
  for it.

- **OQ-17 — TRIM on the USB-attached SSD.** Resolved 2026-09-18 by [DEC-24](decisions.md):
  the SSD is back on PCIe and TRIM works natively. Kept for the record. The RTL9210B bridge does not
  expose discard to the kernel, and forcing it hung the disk and the host (DEC-23). The
  drive runs without TRIM. Options, none examined: a bridge firmware update (Realtek's
  tool is reported to be Windows-only; unverified), an enclosure with a bridge that
  passes UNMAP cleanly (unverified which), or accept no TRIM on a 128 GB drive and watch
  wear. `smartmontools` is not installed; the bridge's NVMe SMART passthrough
  (`smartctl -d sntrealtek`) is untested. `fstrim.timer` stays enabled: it fails
  harmlessly while `provisioning_mode` is `full`.
  **One of those options is no longer unverified (2026-09-20):** the JMicron `152d:0562`
  SSK enclosure passes UNMAP correctly under both a range test and a scattered `fstrim`,
  and now runs TRIM on hailo
  ([wk-robotics `common.md`](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/common.md#trim-through-the-jmicron-152d0562-bridge)).
  That is a fact about that bridge only — the RTL9210B hang recorded here stands.

## Perception and sensing

- **OQ-35 — The IMU answers `WHO_AM_I` with `0x70`.** (Read 2026-09-29, 13:02 UTC:
  `i2cget -y 1 0x68 0x75`.) The documentation here and the kit's call the part an MPU6050.
  **From memory, not checked against a datasheet this session:** InvenSense gives `0x68`
  for the MPU-6050 and `0x70` for the MPU-6500. **Unknown:** which part is fitted (read its
  marking), and whether the `mpu6050-raspberrypi` library's ranges and scale factors hold
  for it. `/imu/data` has been in use since 2026-09-09 and its yaw sign is still open
  (OQ-13).

- **OQ-26 — Is the pan/tilt head redundant once the lidar is fitted?** (Owner, 2026-09-23,
  "probably", with the body sweeping the sensor instead: `/body_pose` pitch and roll tilt the
  scan plane, and the IMU reports the angle.) What the lidar replaces outright: the
  ultrasonic's one 15° cone and the head-scanning behaviour behind the `sonar_layer` — 360°
  at 10 Hz, 0.72° per point. What is not settled:
  - **Mapping needs a level plane.** `slam_toolbox` matches scans as one horizontal slice; a
    scan taken pitched sees a different wall height and, nearer in, the floor. Sweeps are for
    looking, not mapping: gate mapping scans on the IMU, or hold the body level while the
    map is being built.
  - **A tilted scan sees the floor as a wall** unless points are projected through `tf` with
    the body attitude and those below a height threshold are dropped. No such node exists;
    the standard costmap layers assume a horizontal scan.
  - **The low band.** The plane is ~190 mm above the floor with the body level (measured
    2026-09-28, [`hardware.md`](hardware.md#head-sensors)), so level scans miss anything
    lower than that; body pitch is a shallow sweep (range unrecorded — measure
    it), reaching a low obstacle a metre out, not the floor at the feet. The ultrasonic
    covered that band badly; the lidar does not cover it at all.
    **Seen 2026-10-02 (owner):** the robot approached a step it cannot climb, below the
    scan plane (about 0.21 m at the 50 mm stand, DEC-34), which the lidar did not see;
    step height and run time not recorded. The owner's view: the sonar is needed, tilted
    down to read low and directly ahead. **Next, with the battery and the owner:** run the
    head through its range, tilt above all, to find whether tilting up meets the lidar or
    its plane, and whether the slipping tilt gears (OQ-30) hold a fixed down angle.
    **Done 2026-10-02:** tilted well up (channel 0 at 100°, 35° above the 65° level by
    command; the owner judged 15–20° from where it rested) the head stands beside the
    lidar's base, so the useful tilt is level and below. Down to 18° below level at 45°;
    travel ends at 40° (OQ-30). **By arithmetic from the URDF, unverified:** the sonar
    sits about 0.10 m above the floor at the 50 mm stand, so at 18° down its axis meets
    the floor about 0.31 m ahead of it, and the HC-SR04's ~15° cone spans roughly
    0.21–0.54 m: it would read the floor itself, inside the 0.32 m stop polygon. A
    step detector needs a shallower angle or floor-return handling; not designed.
  - **The camera.** It shares the head. Not working (OQ-23), but face recognition and
    `head/look_at` are in the autonomy stack; removing the head means a body mount and a
    body-yaw look-at, decided deliberately.
  Gains if the head goes: two servos, the HC-SR04 and the camera's ribbon off the top of the
  robot, and OQ-21 (the head's feedback-less joint states) closes with it.

- **OQ-33 — The dashboard shows values it cannot vouch for.** (Owner asked for an
  assessment, 2026-09-29; read from `web_dashboard.py` and compared with the robot's logs.)
  Faithful: the two voltages while `battery_monitor` publishes, the autonomy state and the
  mission fields (2 Hz from `autonomy_manager`), "No Camera Feed". Not faithful:
  - **Nothing goes stale** except the scan (1 s) and the robot's pose on the map (2 s). Every other value is the last one
    received, for ever. With the I2C bus dead (OQ-32) `battery_monitor` published nothing
    and the page showed 0.0 V, which it labels "USB", on the battery.
  - **Sonar fan: removed 2026-09-29** (owner). In its place the lidar view draws the
    current scan in the body frame, robot facing up, and the status shows the nearest
    return and its bearing.
  - **Map: partly fixed 2026-09-29** (owner asked for the robot on it). It is still the
    global costmap. Obstacles (100), inscribed cells (99) and inflation (1–98) now have
    different colours, and the robot's radius and heading, the current scan and a 1 m
    scale are drawn from TF. No goal or frontier is drawn. "Map: Active" still means one
    costmap message was ever received.
  - **SLAM mode** is a constant set in `autonomy_manager`, not read from `slam_toolbox`. It
    said "mapping" under DEC-28, when there was no SLAM, and while SLAM produced no map.
  - **Exploration %** is frontiers reached × 10.
  - **Faces: 0** with the camera dead means "no data", not "nobody".
  - **Absent:** whether the controller is initialised (OQ-28 showed `exploring` with limp
    legs), the head being disabled, the lidar, the collision monitor's state, planner and
    controller failures, `AutonomyState.error_message`, driver I/O errors, undervoltage.
  Wanted: an age on every value with "no data" past a limit, the map drawn from `/map`
  with the robot and goal on it, the fan replaced or relabelled, and a health row from the
  nodes' own reports.

- **OQ-22 — What the mono camera is for.** (2026-09-19, [DEC-28](decisions.md).) The
  OV5647 feeds only the dashboard stream and optional face recognition; nothing in
  navigation uses it. It is the robot's only rich sensor, and every obvious use costs CPU
  the Pi may not have ([OQ-02](#compute)): fiducial markers for drift correction
  ([OQ-20](#locomotion-and-navigation)), monocular visual odometry, or a small detector for
  what the sonar misses — chair legs, edges, anything soft. Nothing decided, nothing built.

- **OQ-23 — The Pi camera does not probe.** (2026-09-19.) `ov5647: probe of 10-0036 failed
  with error -121` at every boot and libcamera reports no cameras, so `camera_ros` exits at
  once and `camera:=false` is the working default for bench runs. The software side is in
  place (overlay, libcamera 0.7.2 with the PiSP pipeline and an OV5647 tuning file,
  `camera_ros` from apt); the sensor simply does not acknowledge on I2C. Suspects, in
  order: ribbon seating, contact orientation, the wrong connector (the Pi 5 has CAM/DISP 0
  and 1), or a cable that is not the Pi 5's 22-pin type. A physical check by the owner, not
  a software change.

- **OQ-06 — Face recognition scope and cost.** `face_recognition_node` is configured for
  the CNN detector, which on a Pi CPU is far slower than HOG and would add to OQ-02. It is
  not in the boot stack. Whether faces are a goal of this robot at all is undecided.

- **OQ-07 — Which IMU.** Closed 2026-09-19 by [DEC-25](decisions.md): the D435i left, so
  the shield's MPU6050 is the only IMU and there is nothing to choose.

## Power and safety

- **OQ-14 — Charging while the servos stay powered.** Today the two 18650 cells feed the
  shield's LOAD (servo) and CTRL (Pi) rails and are charged by the kit's charger with the
  robot idle; on USB-C the servo rail is dead ([`hardware.md`](hardware.md#power)). The
  owner wants to look into a USB-C PD power path between the batteries and the electronics
  so the robot can charge with servo control intact. **Constraint stated by the owner
  (2026-09-09):** the kit's hardware design leaves little room for this, and a significant
  deviation from Freenove's design would mean printing and building another hexapod rather
  than modifying this one. Not examined; the shield's charge circuit and rail topology
  would need reading from the vendor schematic first.

- **OQ-32 — The I2C bus stopped answering: a device held SDA low.** (2026-09-29,
  [`test-log.md`](test-log.md), two entries.) **Freed the same day without a power cycle;
  the cause is unknown.**
  - **What happened.** At 12:39:57.19 UTC, 11 s into the day's seventh start of the stack,
    the kernel logged `i2c_designware 1f00074000.i2c: i2c_dw_handle_tx_abort: lost
    arbitration`, the only such line in the robot's journal. From 12:39:58
    every transfer ended in `controller timed out`, 255 by 12:58. Servo power had been
    enabled 1.45 s before (12:39:55.74) and both PCA9685s initialised; the bus traffic at
    that moment was `imu_driver` at 100 Hz and `battery_monitor` at 1 Hz. No servo command
    had been sent.
  - **State found, 12:59.** SDA low and static, not driven by the Pi; SCL high; both pins
    still on their I2C function. Read from the RP1 pad registers, which reconfigures
    nothing.
  - **Recovery, 13:02.** `scripts/i2c-recover.py --recover`: one SCL pulse released SDA,
    so a device was part-way through a transfer and waiting for a clock. After the STOP and
    a rebind of the controller `i2cdetect -y 1` showed `40 41 48 68`. **Which device held
    the line is unknown.**
  - **Corrections to this entry as first written.** `servo_driver` did report the failure:
    it died at 12:40:16 with `TimeoutError` on the first write of its first
    `/joint_commands` message, so **no leg command reached a servo in the 12:39 run**, and
    at 13:03 no PCA9685 channel held a live PWM. The controller's `home` and `stand` "done"
    say only that it published. The LOAD 7.06 V, CTRL 7.82 V logged at 12:40:05 was
    `power_indicator`'s 10 s report of a value `battery_monitor` published before the
    failure, not a read that got through.
  - **Changed** (commit `40a234b`): `servo_driver` logs a failed transfer, drops that
    command and keeps running, so a bus that recovers is used again and the servos can be
    relaxed on shutdown. Tested with a stub, not against a dead bus.
  - **Recovered by the kernel since 2026-10-02** (commit `35ea55c`,
    [`test-log.md`](test-log.md)). `boot/hexapod-i2c1-recovery.dts` names GPIO 3 and 2 as
    the controller's `scl-gpios` and `sda-gpios`, so on every `controller timed out` the
    DesignWare driver clocks SCL until SDA is released and re-initialises itself, with the
    stack running. Tested on USB by holding SDA low from the pad
    (`scripts/i2c-fault-test.py`): the 09-29 log lines, then the IMU back at about 90 Hz
    within about 1 s. **Not yet seen against a real fault on the battery.**
  - **Open.** Why arbitration was lost: a glitch on SDA as the servo rail came up, the
    400 kHz clock on the shield's wiring, and a fault in one device are candidates, none
    examined. Whether it recurs: twice so far (09-29, 10-02), both on the battery with the
    servo rail energised. A held SCL is not recoverable by clocking. Whether flash-kernel
    leaves the custom `.dtbo` in `/boot/firmware/overlays/` across kernel updates is
    unverified: after one, check `dmesg | grep "gpio recovery"`. The stack does not report
    bus faults; with the bus dead the state machine still explores on paper
    ([OQ-33](#perception-and-sensing)).

- **OQ-29 — Undervoltage reset when the stack restarts on the battery.** (Found
  2026-09-29, [`test-log.md`](test-log.md).) `systemctl restart hexapod` with LOAD 7.76 V
  and CTRL 8.06 V was followed 7 s later by `hwmon: Undervoltage detected!` and a reset of
  the Pi. The cold boot 9 min earlier started the same stack without one. **Unknown:** what
  drew the rail down, and whether it repeats. Candidates, none examined: the servo rail
  being re-enabled while every node starts, the lidar motor spinning up, the SSD. Until it
  is known, a restart on the battery can cost a reboot. Family layer:
  [wk-robotics `common.md`](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/common.md#power-integrity).

- **OQ-11 — Low-voltage behaviour and a watchdog.** Nothing cuts the servo rail on a low
  pack, and if the Pi dies the servos hold their last pose under load until the battery
  sags. `battery_monitor` only publishes voltages. Battery-aware return-to-home is on the
  roadmap; a hardware cutoff is not designed. **First case, 2026-09-29:** standing, held
  by the collision monitor, the LOAD rail went from 7.41 V to below 6.5 V and nothing
  acted on it; the owner saw the LED ring red and switched the robot off
  ([`test-log.md`](test-log.md)). The stance was the 80 mm of [DEC-34](decisions.md),
  whose drain is unmeasured.

- **OQ-18 — 5 V budget with the SSD enclosure and the AI HAT+ 2.** Resolved 2026-09-18 by
  [DEC-24](decisions.md): neither goes on this robot. Kept for the record. Both
  were new loads on the CTRL rail on battery; neither is measured. `usb_max_current_enable=1`
  is set in `config.txt`. The Pi 5 PCIe connector supplies 5 V at 500 mA per pin, 1 A
  total, per Raspberry Pi's connector standard, and a HAT+ pulls the connector's detect
  pin high so the bootloader probes PCIe without an ID EEPROM. The shield occupies the
  GPIO header. Forum users report the original AI HAT+ (Hailo-8L, about 1.5 W) working
  on the ribbon alone, but the AI HAT+ 2 is a different case: a third-party review
  (faceofit.com, not Raspberry Pi) states it draws its power from the GPIO header and
  gives 1.2 W idle, 3.5–4.5 W vision inference, 8 W peak on LLM loads. 8 W exceeds the
  ribbon's 1 A rating, so **ribbon-only power is not a safe assumption for the HAT+ 2**.
  Its socket carries pins that do not protrude, so the shield cannot stack on it as
  supplied, and the Freenove riser has no height flexibility (owner, 2026-09-18), so
  stacking is out. Wiring 5 V and ground into the HAT's socket would only work if those
  and the ID EEPROM pins (27/28) are all the HAT uses; that is documented for the M.2
  HAT+ and claimed for the original AI HAT+ from its EEPROM overlay string, but **no
  schematic or pin list for the AI HAT+ 2 is published**, so it is unverified. The
  shield's 5 V regulator rating is also unknown. Decided the same day: the HAT goes on
  the tank bot and the SSD returns to the M.2 HAT (DEC-24).

## Mission planning

- **OQ-04 — What "approved mission planner" means.** The dashboard API accepts missions
  from anyone on the network (DEC-14). The stated end state is a planner entity whose
  authority the robot recognises. Candidates: a shared secret on the HTTP API, mTLS, or the
  family's Zenoh-bridged ROS 2 graph with the planner as a node. Decide before the robot
  is reachable from anywhere but the home network.

## Project

- **OQ-25 — Restart the stack under linger (family startup rule 10).** **Resolved 2026-09-22**
  by an unplanned power cycle, which gave the cold boot the question was waiting for
  ([`test-log.md`](test-log.md)): `Linger=yes`, 226 `fastrtps_*` entries in `/dev/shm` after
  four SSH sessions had opened and closed, and from the workstation `/imu/data` at 108 Hz and
  `/tf` at 50 Hz — the two that delivered nothing under the fault — plus `/imu/data_raw`,
  `/joint_states` and `/ultrasonic/range`. The stack was safe to let explore because the servo
  rails were dead (SBC on mains, 0.00 V on both), so the explore concern below did not arise.
  The original report, for the record. (2026-09-21.) Observed
  from the family's mission-planner hosts on the LAN: `/imu/data` and `/tf` delivered nothing,
  and `/joint_states` and `/ultrasonic/range` came and went, while `/imu/data_raw` kept
  arriving. On the Pi, `hexapod.service` had been active since 2026-09-20 with `Linger=no` and
  **no `fastrtps_*` segment in `/dev/shm`**. logind's `RemoveIPC` had deleted them when an SSH
  session closed, which breaks same-host Fast DDS delivery: `imu_filter` stops getting
  `/imu/data_raw`, for one. Linger was enabled on the Pi on 2026-09-21, and
  `systemd/install.sh` now enables it too. What was then open was the restart: the segments
  come back only when the stack restarts, and the unit starts autonomy, so it had to wait
  until the robot was safe to explore — which the power cycle above settled by cutting the
  servo rails.

- **OQ-24 — Conform to the family startup rule.** Resolved 2026-09-21, from the workstation
  (DEC-29). The robot's diverged checkout was merged (`c247f06`) and now tracks `main` over
  HTTPS. The startup now follows
  [*Robot startup is familial*](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/common.md#robot-startup-is-familial):
  the ROS environment is in `scripts/launch.sh` only, set before sourcing ROS, with
  discovery range `SUBNET` stated explicitly. `autonomy:=true` is passed by the unit alone,
  so a manual `launch.sh` no longer explores. The unit has `KillMode=mixed`. The host state
  outside git is listed in `AGENTS.md`.

- **OQ-10 — Tracking the vendor code.** Resolved 2026-09-13 by [DEC-20](decisions.md):
  the snapshot is deleted and upstream is cloned sparse.

- **OQ-15 — Is any of this repo's code a derived work of the vendor code?** (2026-09-13.)
  Freenove's code is CC BY-NC-SA 3.0; this repo is Apache-2.0, and the two are
  incompatible for derived works (ShareAlike and NonCommercial cannot be relicensed under
  Apache). The hardware drivers were written with the vendor files open as the reference
  (`AGENTS.md`: "read the reference first"). The only mentions of Freenove in the source
  tree are description strings in `hexapod_hardware`'s `setup.py` and `package.xml` and a
  comment in `body_params.yaml`; no file carries a vendor attribution or copyright line.
  Whether any function is a copy or close translation rather than a re-derivation from
  the datasheets has not been checked. Needed: a file-by-file comparison of
  `hexapod_hardware` against `Code/Server/`, then either a clean re-derivation or a
  licence note. Blocks the tri-licence adoption question until answered.
  Add to the comparison (2026-09-15): the controller's `_tripod_gait_step` and
  `_wave_gait_step` are line-by-line ports of `run_gait`, and DEC-22 now routes
  `/cmd_vel` through them.
