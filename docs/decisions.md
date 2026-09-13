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
  (Owner, 2025-12-31.) One USB 3 device gives RGB, depth computed in the camera, and an
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
