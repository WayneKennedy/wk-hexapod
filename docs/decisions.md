# Banked Decisions

Committed decisions with rationale. Unresolved items live in
[`open-questions.md`](open-questions.md). Format: `DEC-nn — decision — why`. Decisions
dated before 2026-09-09 are reconstructed from the commit history and the code; the ones
on 2026-09-09 were made during the native bring-up, with the owner where marked.

---

- **DEC-01 — Port the Freenove kit to ROS 2 rather than extend the vendor stack.** (Owner,
  2025-12-29.) The vendor code is a TCP-command remote-control server; the goal is
  autonomy, which means SLAM and Nav2, which means ROS 2. The vendor code stays as the
  confirmed-working reference for servo, home, stand and gait behaviour
  ([`references.md`](references.md)) and every hardware driver is a port of it.

- **DEC-02 — The RealSense D435i replaces the Pi camera and the ultrasonic sensor.**
  (Owner, 2025-12-31; **superseded by DEC-25, 2026-09-18**.) One USB 3 device gives RGB, depth computed in the camera, and an
  IMU. Depth feeds both RTAB-Map and, via `depthimage_to_laserscan`, the Nav2 costmaps. The
  ultrasonic driver and Pi camera node were kept as dead code until 2026-09-09 and are now
  removed (git history has them).

- **DEC-03 — Body-centric IK controller with a tripod gait, ported from `control.py`.**
  (2025-12-30.) Foot positions are held in the world frame while the body moves; angles are
  solved per leg with the kit's geometry and calibration. Wave gait exists but tripod is
  the default.

- **DEC-04 — Odometry is gait-integrated displacement with IMU yaw fusion, published as
  `/odom` and `odom → base_link`.** (2025-12-31.) The robot has no encoders or wheels;
  commanded displacement per gait cycle is the only proprioceptive signal. Yaw is blended
  with the IMU (complementary filter, weight 0.98) because heading drifts fastest.

- **DEC-05 — RTAB-Map RGB-D SLAM; the `slam_toolbox` path is removed.** (2026-01-06,
  confirmed 2026-09-09.) `slam_toolbox` was configured when the only range sensor was the
  ultrasonic and never ran; RTAB-Map uses the RealSense directly and produced a map on the
  bench on 2026-09-09. Its launch file, parameters and dependency were deleted the same day.

- **DEC-06 — Nav2 runs as navigation servers only; map and localization come from
  RTAB-Map.** (2026-09-09.) Jazzy's full `bringup_launch.py` starts a map server and AMCL
  that have nothing to load. `navigation_launch.py` is used instead, with the Jazzy-specific
  parameter forms (`pkg::Class` plugin names, packaged BT defaults, `collision_monitor` and
  `docking_server` sections) recorded in `nav2_params.yaml`.

- **DEC-07 — ROS 2 runs natively on Ubuntu 24.04; Docker is removed.** (Owner,
  2026-09-09.) The host is the Jazzy Tier 1 platform, so every package is an apt install.
  The container ran `privileged` with host networking (no isolation), needed source builds
  that existed only to work around the container (libcamera), took hours to rebuild, and
  had drifted five months behind the compose file that launched it. Images and volumes
  were deleted; the Docker engine is still installed on the host but unused.

- **DEC-08 — No reflex tier.** (Recorded 2026-09-09; inherent in the kit.) Every device is
  a direct peripheral of the Pi. This is the family's counter-example to the two-tier rule
  ([wk-robotics](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/common.md#compute-the-two-tier-split))
  and is viable because the walker is statically stable: a stalled Pi leaves the PCA9685
  chips holding the last pulse and the robot standing. Retrofitting an MCU is
  [OQ-09](open-questions.md), not a plan.

- **DEC-09 — `servo_driver` owns the servo hardware; the controller publishes angles.**
  (2026-09-09.) Both nodes used to claim GPIO 4 and both drove the PCA9685 chips; the
  second claim always failed, and the controller's gait steps wrote directly to chips it
  had failed to initialise. Now the controller publishes calibrated angles on
  `/joint_commands` for every gait sub-step and only drives hardware itself when
  `hardware.direct` is true (standalone tests). The controller also runs on a multithreaded
  executor with state publishing in a reentrant callback group, because a blocking gait
  cycle otherwise starves the odometry transform and Nav2 rejects it.

- **DEC-10 — The buzzer is disabled by default, guarded at boot, and held low by a
  service.** (Owner-driven, 2026-09-09.) Two incidents in one session, each ending in a
  hard power pull, and the noise was harmful to a family member. Mechanism and safeguards:
  [`hardware.md`](hardware.md#the-buzzer-hazard). Re-enabling is a deliberate two-step act.

- **DEC-11 — Saved maps live in `~/.hexapod/maps/`, separate from RTAB-Map's working
  database.** (2026-09-09.) The launch used to treat the existence of `~/.ros/rtabmap.db`
  as "a map exists", but RTAB-Map creates that file on its first mapping run, so every
  second boot localized against an empty database and never mapped. A map now counts only
  after `scripts/save-map.sh` copies it, and the autonomy manager tells RTAB-Map to map
  when localization fails.

- **DEC-12 — The URDF owns the camera TF tree; the RealSense driver publishes no TF.**
  (2026-09-09.) Two publishers of the same static transforms are incoherent. The camera
  hangs off revolute head joints, so the controller publishes `head_pan_joint` and
  `head_tilt_joint` states at 50 Hz for `robot_state_publisher`. The camera's own
  depth-to-colour extrinsics are not needed because depth is aligned to colour in the
  driver.

- **DEC-13 — The boot stack starts with autonomy on.** (Inherited from the compose
  configuration; kept 2026-09-09.) The robot is meant to explore unattended. The cost is
  that on the battery it walks off about 30 s after boot; `autonomy:=false` exists for
  bench work ([`operations.md`](operations.md#run)).

- **DEC-14 — The interim mission planner is the dashboard HTTP API, unauthenticated.**
  (Owner, 2026-09-09.) The first external planner will be Claude CLI sessions on another
  machine, which need nothing more than `curl`. What "approved planner" means, and how it
  authenticates, is [OQ-04](open-questions.md).

- **DEC-15 — Licence stays Apache 2.0.** (Owner, at repository creation; reaffirmed
  2026-09-09, closing OQ-08.) The repository is software and documentation only; the
  hardware is Freenove's and not relicensed here. The family tri-licence exists for
  projects with their own hardware design, and this one does none: any significant
  deviation from Freenove's design would be a new, printed hexapod rather than a change
  to this robot.

- **DEC-16 — The repository is the memory, in the family's format.** (2026-09-09.)
  `AGENTS.md` is the single onboarding; `CLAUDE.md`/`GEMINI.md` point at it; facts true of
  several robots live in wk-robotics and are linked, not copied; this repo is public and
  names no hosts or addresses.

- **DEC-17 — Calibration is resolved from a parameter, then `~/.hexapod`, then the
  package copy.** (2026-09-09.) The previous paths existed only inside the container or on
  one developer's disk, so the controller silently ran uncalibrated. One copy of
  `servo_calibration.txt` is tracked, in `hexapod_hardware/config`, and installed with the
  package.

- **DEC-18 — One controller configuration file.** (2026-09-09.) `robot.yaml` in bring-up
  and `body_params.yaml` in the controller package carried overlapping and partly unused
  parameters. `hexapod_controller/config/body_params.yaml` is now the only one, matching
  exactly the parameters the node declares.

- **DEC-19 — IMU orientation comes from `imu_filter_madgwick`.** (2026-09-09.) The
  MPU6050 driver publishes raw gyro and accel on `/imu/data_raw`; the controller's yaw
  fusion consumed `/imu/data` with a quaternion that nothing published, so IMU fusion had
  never received data. The stock Madgwick filter (no magnetometer, no TF) closes the gap.
  Sign and frame conventions against the controller are unverified ([OQ-13](open-questions.md)).

- **DEC-20 — The vendor reference is Freenove's upstream, cloned sparse; the
  `fn-hexapod` snapshot is deleted.** (2026-09-13, owner.) The snapshot existed because
  upstream is 477 MB and the robot's Pi needed ten files; its rewritten history meant it
  could never track upstream (OQ-10). A blobless, sparse clone of `Code/Server/` solves
  the size problem without a second repository, and pinning the upstream commit in
  `references.md` replaces the snapshot as the drift baseline. A second reason not to
  vendor the files here instead: the vendor code is CC BY-NC-SA 3.0 and this repo is
  Apache-2.0 ([OQ-15](open-questions.md)). Resolves OQ-10.

- **DEC-21 — The stack waits for a synchronised clock, and elapsed time is measured on
  the monotonic clock.** (2026-09-15.) The Pi 5 RTC has no backup cell; on a cold boot
  `systemd-timesyncd` restores the last recorded time and steps to NTP time when the
  network answers — +24 min, 45 s after boot, on the first battery run
  ([`test-log.md`](test-log.md)). The step ended the exploration action, stopped the
  collision monitor and broke RTAB-Map's TF lookups. Two changes: `hexapod.service` is
  ordered after `time-sync.target` with `systemd-time-wait-sync.service` enabled and
  bounded to 90 s (`systemd/time-wait-sync-timeout.conf`), so an offline boot still
  starts on the restored clock; and every node that measures a duration
  (`frontier_explorer`, `autonomy_manager`, `look_around`, `mission_server`, the
  controller's cycle and feedback timing) uses `time.monotonic()`. Message stamps stay on
  ROS time. Also that day: `bt_navigator.default_server_timeout` raised from 20 ms to
  1000 ms, since at boot load the planner missed the acknowledge window on every replan
  ([OQ-02](open-questions.md)).

- **DEC-22 — `/cmd_vel` is SI and drives the vendor's gait.** (2026-09-15.) The
  controller had read `linear.x = 1.0` as 25 mm per cycle and ran one cycle per message
  through a home-grown gait that, simulated from its own foot maths, moved the body 43 mm
  for a commanded 25 and dragged feet at each half-cycle transition; Nav2's 0.05 m/s
  became 1.25 mm per cycle and the robot stepped in place (OQ-01, closed). Now: a gait
  worker runs one cycle per tick on the latest command while it is fresh (0.5 s) and
  non-zero; each cycle moves the body `v × cycle_time`, converted to the vendor gait's
  per-cycle units (`_tripod_gait_step`, the port of `run_gait`) by the simulated factor
  of 4 for tripod (35 mm → 140.0 mm, 1° → 4.00° clockwise per cycle; wave is 2 from the
  maths, unsimulated) and clamped to the vendor's limits (35 mm, 10°); odometry
  integrates the commanded geometry times measured scale factors
  (`odometry.stride_scale` 0.82, `odometry.turn_scale` 0.72, measured the same day by
  the calibration in [`operations.md`](operations.md#movement-calibration)). The
  home-grown cycle is deleted and `MoveDistance` walks at a speed instead of a fixed
  step. Verified on the floor: +x walks forward, +z turns left. Three defects found and
  fixed on the way ([`test-log.md`](test-log.md)): frame loops now sleep to absolute
  deadlines with a 1 ms interpreter switch interval (cycles had run 2–4× long); the floor
  is one explicit reference (`GROUND_Z = 0`) for the initial feet, the stand reset and
  the gait lift; +y strafe is unverified (Nav2 does not use it).

- **DEC-23 — The SSD is on USB 3; the PCIe connector is reserved for the AI HAT+ 2.**
  (Owner, 2026-09-18; **superseded by DEC-24 the same day**.) The Pi 5 exposes one PCIe lane on one FPC connector and the owner
  wants it for a Raspberry Pi AI HAT+ 2 (Hailo-10H), so the 128 GB NVMe SSD moved from
  its M.2 HAT to an external USB 3 enclosure (Realtek RTL9210B bridge, UAS, 5 Gbps).
  Nothing in the image changed: root and boot are found by label. Bootloader
  `BOOT_ORDER` went from `0xf146` to `0xf14` (USB, then SD; the NVMe probe dropped) on
  the 2025-12-08 bootloader release. **TRIM stays off.** The bridge advertises UNMAP in
  its VPD pages but clears LBPME, so the kernel leaves `provisioning_mode` at `full` and
  `fstrim` reports "not supported"; forcing `unmap` and running `fstrim` hung the disk
  and the host on 2026-09-18 ([`test-log.md`](test-log.md)). No udev rule may set
  `unmap` on this bridge. Follow-ups: [OQ-17](open-questions.md) (TRIM),
  [OQ-18](open-questions.md) (5 V budget with the enclosure and the HAT).

- **DEC-24 — The SSD stays on PCIe; the AI HAT+ 2 goes to the Devastator tank, not the
  hexapod.** (Owner, 2026-09-18.) Supersedes DEC-23. The AI HAT+ 2 is powered through
  the GPIO header (third-party review; no schematic is published), its header socket
  cannot be stacked on as supplied, and the Freenove riser has no height flexibility, so
  there is no supported way to fit it to this robot ([OQ-18](open-questions.md), closed).
  The NVMe SSD returns to its M.2 HAT on the PCIe connector; `BOOT_ORDER` is `0xf146`
  again (flashed 2026-09-18 before the move, so the swap needs no software step) and
  TRIM works natively. The USB 3 enclosure is retired from this robot; its TRIM hang is
  kept in [`test-log.md`](test-log.md) as a negative result. The HAT's home is a
  Devastator fact and belongs in
  [wk-robotics](https://github.com/WayneKennedy/wk-robotics), not here.

- **DEC-25 — The head returns to the kit's sensors; the stack stays ROS 2.** (Owner,
  2026-09-18.) Supersedes DEC-02. The D435i leaves this robot and the kit's OV5647 Pi
  camera and HC-SR04 ultrasonic go back on the pan/tilt head; both are in hand. The D435i
  is banked with the family's Jetson Orin Nano as a pair, because depth computed in the
  camera is what justifies its cost over a standard camera, and a CPU-only Pi 5 could not
  use it fully here ([OQ-02](open-questions.md)); the family reasoning is in
  [wk-robotics `common.md`](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/common.md#perception-placement). The ROS 2 stack is kept.
  **What this breaks:** RTAB-Map RGB-D mapping, the depth-derived `/scan` that feeds both
  costmaps and the collision monitor, the dashboard's depth stream and the face node's
  colour source all took their input from the D435i. How the robot maps and sees obstacles
  with one camera and one range sensor is [OQ-19](open-questions.md). Resolves OQ-07. The
  ultrasonic driver and Pi camera node removed on 2026-09-09 are in git history (DEC-02).
  **Implemented on the robot, 2026-09-19** (its commit `343638f`, merged 2026-09-21): the
  one D435i in the family was
  reassigned to another robot, and the OV5647 Pi camera and HC-SR04 were refitted to the
  pan/tilt head. This reverses DEC-02 and takes RGB-D SLAM with it: removed from the
  stack are `realsense2_camera`, `depthimage_to_laserscan`, RTAB-Map (DEC-05), `/scan`,
  the saved-map and localization path (DEC-11, `scripts/save-map.sh`), `slam_monitor`
  and `LocalizationStatus`. Added: `ultrasonic_driver` (`/ultrasonic/range`) and
  `camera_ros` (`/camera/image_raw`), both on apt packages, no source builds.
  DEC-02's second reason for dropping the ultrasonic — unreliable software-timed echo —
  does not survive: the driver times the echo from kernel line-event timestamps, and
  fifteen consecutive pings agreed to 2 mm ([`test-log.md`](test-log.md), 2026-09-19).
  Its first reason stands: one range point cannot fill a costmap, which is what DEC-27
  and DEC-28 are for. **The camera has not yet worked**: the sensor does not acknowledge
  on I2C ([`hardware.md`](hardware.md#head-sensors), [OQ-23](open-questions.md)).

- **DEC-26 — The robot is the family's baseline intent-tier reference, and its hardware
  ceiling is the kit's.** (Owner, 2026-09-18.) It stays a working intent-tier reference —
  ROS 2 on a CPU-only Pi 5, no reflex tier (DEC-08), primitive sensors (DEC-25) — beneath
  wk-devastator, which adds a reflex tier and an accelerator. It is not the family's R&D
  platform for new hardware: the GPIO riser blocks the AI HAT+ 2 (DEC-24), and its servos
  are standard-size PWM hobby servos, lugged at each end and driven from a PWM board on
  that riser, so a switch to ST3215 bus servos is not feasible. The owner judges a new
  custom hexapod more feasible than remaking this one
  ([wk-robotics `ideas.md`](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/ideas.md#a-printed-hexapod)).

- **DEC-27 — The head looks; the body walks. `head_controller` owns the head servos.**
  (Owner, 2026-09-19: "it jars to see 18 servos walking on the spot to rotate in the pan
  axis, when a cheap servo mounted to the camera could be used".) Both sensors are on the
  head, so pointing them is now a first-class behaviour rather than a side effect of
  driving. One node owns pan and tilt, which also resolves OQ-12: the leg controller
  publishes NaN in the head slots of `/joint_commands` and `servo_driver` skips them, so
  gait sub-steps no longer re-centre the head. The head scans a ±30° sector centred on
  Nav2's pure-pursuit carrot (`/lookahead_point`), so it is already looking where the body
  is about to go, and `LookAround` is now a head-only survey (its body-rotation phases are
  deleted). Turning in place purely to look is removed everywhere it was found: no `Spin`
  recovery in the behaviour trees or `behavior_server`, and frontier goals no longer carry
  a heading to align with (goal-checker yaw tolerance π). Rotating to *travel* along a path
  is kept: `use_rotate_to_heading` stays true, because a single forward cone must be
  pointed where the robot is going, and the head leads the turn. Unverified: the pan sign,
  the travel limits, and the servo slew rate ([OQ-21](open-questions.md)) — the servos need
  the battery and the owner present.

- **DEC-28 — The map is Nav2's global costmap, built from the sonar and anchored to
  odometry. There is no SLAM.** (2026-09-19, implementing the owner's request to see what
  the primitive sensors can still do.) A single 15° cone at 15 Hz cannot support scan
  matching or loop closure, and the mono camera cannot be trusted for either on this CPU
  (OQ-22), so rather than pretend: `nav2_costmap_2d::RangeSensorLayer` on
  `/ultrasonic/range` fills a fixed 12 × 12 m global costmap, `map → odom` is a static
  identity, and the frontier explorer reads `/global_costmap/costmap` instead of `/map`.
  The layer's probabilistic cone model is the standard one for sonar and keeps unseen
  cells unknown, which is what makes frontier exploration work at all. Consequences,
  accepted: gait odometry drift is map drift, nothing corrects it, maps do not survive a
  run, and `checking_map`/`localization_mode` become unreachable states. Two changes fall
  out of it: the behaviour trees must not clear the global costmap (that would erase the
  map), and the explorer must ignore frontiers within `min_goal_distance` of the robot,
  because the cells beside its own body that the forward cone never sees would otherwise
  be "reached" instantly and forever. Verified end to end on USB power the same day;
  **the map's geometry is not verified**, because the head servos cannot move without the
  battery ([`test-log.md`](test-log.md)).

- **DEC-29 — No assistant runs on the robot. Sessions run on the always-on workstation and
  operate the robot over SSH.** (Owner, 2026-09-21. Supersedes the 2026-09-18 rule that the
  robot owns this repository for major changes, from a session running on it.) This matches
  the family's other ROS 2 hosts, the Orin and the AI HAT+ 2 bench, which carry no
  assistant. Code is authored in the workstation's checkout and pushed. The robot's checkout
  is then updated from `origin/main` and the change is tested over SSH. Claude Code was
  removed from the robot the same day, and its data was archived off the robot, without
  credentials. The robot's session left one unpushed commit, `343638f`, which was merged
  from the workstation the same day (OQ-24).
