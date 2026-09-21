# Open Questions (pending decisions)

Unresolved. Resolve → move to [`decisions.md`](decisions.md). A recommendation is one
option with an argument attached, not a decision; do not build against one without the
owner deciding.

---

## Locomotion and navigation

- **OQ-19 — Mapping and obstacle sensing without depth.** (2026-09-18, opened by
  [DEC-25](decisions.md).) The head carries the OV5647 camera and one HC-SR04; the code
  still expects the D435i (`realsense_slam.launch.py`, `depthimage_to_laserscan`,
  RTAB-Map in RGB-D mode). To settle: the camera driver on Ubuntu 24.04 (the Pi camera
  node removed 2026-09-09 is in git history; the CSI camera stack on Ubuntu rather than
  Raspberry Pi OS is untested here); an ultrasonic driver (the old one was dropped because
  software-timed echo on a non-real-time kernel was unreliable —
  [`hardware.md`](hardware.md#sensor-swap)); how a single range reading reaches Nav2
  (candidates, unexamined: a `sensor_msgs/Range` into a range-sensor costmap layer, or a
  head-pan sweep assembled into a scan); and what mapping, if any, replaces RGB-D
  RTAB-Map. The OQ-03 analysis and the OQ-16 head-sweep argument were made for the
  D435i and need redoing against these sensors. Blocks roadmap milestone 1.

- **OQ-16 — Look with the head before turning the body.** (Owner, 2026-09-15; the
  D435i it assumes has left, DEC-25 — redo under OQ-19.) Nav2's
  pure-pursuit controller spins the whole robot to face each new path
  (`use_rotate_to_heading`), eighteen servos for what the head's pan servo and the
  D435i's 87° field of view could do standing still. Two candidates: the explorer runs a
  `LookAround` head sweep at each new goal so the map grows without walking, and
  `use_rotate_to_heading: false` so the controller arcs into a path instead of spinning.
  **Recommendation:** both, once the movement calibration is done; measure the map growth
  per sweep and the battery cost per turn before and after.

- **OQ-03 — Collision monitor and costmap tuning on the floor.** (The analysis below is
  for the D435i, which has left — DEC-25; the sensor question is now OQ-19.) The collision monitor
  runs a 0.44 × 0.40 m stop polygon on `/scan` with a 3 s source timeout; costmaps use a
  0.15 m robot radius and 0.3 m inflation. **First floor run (2026-09-15,
  [`test-log.md`](test-log.md)): the robot walked into obstacles and the monitor never
  stopped it.** Working analysis: the stop polygon ends 0.22 m ahead of `base_link`,
  inside the D435i's minimum depth range (about 0.28 m at the default profile; the
  launch sets no profile) and close to `range_min` 0.2 m, so nothing the monitor acts
  on can ever be seen; and the scan is a 10-pixel band at the camera's height, blind to
  anything lower. **Recommendation:** an `approach` or slowdown polygon reaching
  0.4–0.5 m ahead, a lower depth profile (424×240 halves the minimum range), a check
  of the camera height against the obstacles in the room, and then the costmap
  inflation against the measured stopping distance at 0.05 m/s.

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

- **OQ-06 — Face recognition scope and cost.** `face_recognition_node` is configured for
  the CNN detector, which on a Pi CPU is far slower than HOG and would add to OQ-02. It is
  not in the boot stack. Whether faces are a goal of this robot at all is undecided.

- **OQ-07 — Which IMU.** Resolved 2026-09-18 by [DEC-25](decisions.md): the D435i has
  left, so the MPU6050 is the only IMU. Kept for the record. The D435i's gyro and accel are streamed and ignored; the MPU6050
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

- **OQ-20 — Reconcile the robot's checkout, and conform to the family startup rule.**
  (2026-09-21, from a wk-robotics session on the workstation; proposals only, per
  [`AGENTS.md`](../AGENTS.md#working-on-the-robot-itself).)
  1. **The robot's checkout has diverged from `main`.** Commit `343638f` ("Sonar and head
     stack replaces the D435i", 2026-09-19, 47 files, including a removed message type) is
     what `hexapod.service` runs. It was never pushed, and is likely the code side of OQ-19.
     `main` has six newer commits: docs and `scripts/ubuntu-setup.sh` only. Both sides
     edited `decisions.md` around DEC-25/26, so expect conflicts there. The commit is now
     preserved on GitHub as branch `robot/343638f-sonar-head`. Proposal: rebase it onto
     `main` on the robot, resolve the decisions, push, then delete the branch.
  2. **Startup deviations** from
     [*Robot startup is familial*](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/common.md#robot-startup-is-familial)
     (owner's rule, 2026-09-21). Proposals:
     - Drop the ROS `Environment=` lines from `systemd/hexapod.service`; `launch.sh` owns
       the environment (rule 2).
     - In `scripts/launch.sh`, export `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` and
       `ROS_AUTOMATIC_DISCOVERY_RANGE` *before* sourcing ROS (rule 3). Jazzy's setup script
       sets the discovery range to `SUBNET` if unset. To keep today's behaviour, set
       `SUBNET` explicitly; the family's choice of range is still open.
     - Pass `autonomy:=true` in one place only (rule 4). Today it is in both the unit and
       `launch.sh`.
     - List the host state that git does not hold in `AGENTS.md` (rule 8).
     A behaviour check after the change: `ros2 node list` matches today's, and the dashboard
     comes up.

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
