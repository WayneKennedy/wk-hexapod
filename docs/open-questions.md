# Open Questions (pending decisions)

Unresolved. Resolve → move to [`decisions.md`](decisions.md). A recommendation is one
option with an argument attached, not a decision; do not build against one without the
owner deciding.

---

## Locomotion and navigation

- **OQ-01 — Velocity semantics between Nav2 and the controller.** The controller reads
  `cmd_vel` as gait parameters: `linear.x = 1.0` → 25 mm per gait cycle, `angular.z = 1.0`
  → 12° per cycle. Nav2 sends SI velocities capped at 0.1 m/s and 0.5 rad/s
  (`nav2_params.yaml`), which the controller turns into 2.5 mm and 6° per cycle — so slow
  that Nav2's progress checker aborts ("Failed to make progress", bench, 2026-09-09).
  **Recommendation:** make the controller interpret `cmd_vel` in m/s and rad/s using the
  measured cycle time and step geometry, then re-tune Nav2's limits from a measured walking
  speed. Needs the battery and a floor.

- **OQ-03 — Collision monitor and costmap tuning on the floor.** The collision monitor
  runs a 0.44 × 0.40 m stop polygon on `/scan` with a 3 s source timeout; costmaps use a
  0.15 m robot radius and 0.3 m inflation. All untested against real obstacles.

- **OQ-05 — Return-to-home and semantic waypoints.** `return_home` exists as a mission
  type; the home pose is the map origin. No named places, no docking (Nav2's docking server
  runs with no docks defined).

- **OQ-12 — Two writers to the head servos.** `look_around` publishes `/head_command`
  while the controller's `/joint_commands` carries pan and tilt fixed at 90°. During a
  combined sweep (head plus body rotation) the controller's gait sub-steps will keep
  re-centring the head. Unverified on hardware; the controller should probably track the
  commanded head angles or drop them from `/joint_commands`.

- **OQ-13 — IMU filter conventions.** `imu_filter_madgwick` was added on 2026-09-09 to
  provide `/imu/data` (DEC-19). Whether its ENU yaw sign and frame match what the
  controller's complementary filter and balance loop assume has not been checked against
  the robot turning. Check before trusting yaw fusion.

## Compute

- **OQ-02 — CPU load.** Load average ~10 on the four-core Pi 5 with the full boot stack
  (2026-09-09; RealSense point cloud already disabled). The hot Python nodes were
  `web_dashboard` (JPEG-encoding three streams with no viewer), `frontier_explorer`
  (frontier detection over the whole map each loop), `mission_server`, and `imu_driver`
  (100 Hz I2C polling). Options: encode dashboard streams only while a client is
  connected, rate-limit frontier detection, drop the IMU rate, or move a perception stage
  off the CPU per the family's
  [perception placement](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/common.md#perception-placement).

- **OQ-09 — A reflex tier retrofit.** An MCU (Teensy or RP2040, micro-ROS) owning the
  PCA9685 chips, MPU6050, buzzer and servo-power pin would remove the buzzer float, the
  scheduler sensitivity and the IMU polling from the Pi, and make the robot a second
  consumer of koala-bot's reflex firmware. The controller already publishes
  `/joint_commands`, so it would not change. Cost: a board, a harness into the shield's
  I2C and GPIO, and firmware. Not planned; recorded because the family direction argues
  for it.

## Perception and sensing

- **OQ-06 — Face recognition scope and cost.** `face_recognition_node` is configured for
  the CNN detector, which on a Pi CPU is far slower than HOG and would add to OQ-02. It is
  not in the boot stack. Whether faces are a goal of this robot at all is undecided.

- **OQ-07 — Which IMU.** The D435i's gyro and accel are streamed and ignored; the MPU6050
  on the shield is the only IMU used. The camera IMU sits on the moving head, which argues
  for keeping the body IMU for odometry, but the D435i's is better calibrated for visual-
  inertial use in RTAB-Map. Not examined.

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

- **OQ-11 — Low-voltage behaviour and a watchdog.** Nothing cuts the servo rail on a low
  pack, and if the Pi dies the servos hold their last pose under load until the battery
  sags. `battery_monitor` only publishes voltages. Battery-aware return-to-home is on the
  roadmap; a hardware cutoff is not designed.

## Mission planning

- **OQ-04 — What "approved mission planner" means.** The dashboard API accepts missions
  from anyone on the network (DEC-14). The stated end state is a planner entity whose
  authority the robot recognises. Candidates: a shared secret on the HTTP API, mTLS, or the
  family's Zenoh-bridged ROS 2 graph with the planner as a node. Decide before the robot
  is reachable from anywhere but the home network.

## Project

- **OQ-10 — Tracking the vendor code.** `fn-hexapod` is a rewritten-history snapshot of
  Freenove's repository pinned at 2025-11-28, on branch `master`, with no shared commits
  upstream, so it cannot fast-forward. Checked 2026-09-09: upstream has since changed only
  `control.py`, and only `np.mat` → `np.asmatrix` (numpy 2 compatibility); every other
  `Code/Server` file is byte-identical. Nothing to port now. Whether to re-import as a true
  fork, and whether the branch becomes `main`, is undecided.
