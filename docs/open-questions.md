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

- **OQ-05 — Return-to-home and semantic waypoints.** `return_home` exists as a mission
  type; the home pose is the map origin. No named places, no docking (Nav2's docking server
  runs with no docks defined).

- **OQ-12 — Two writers to the head servos.** Resolved 2026-09-19 by
  [DEC-27](decisions.md): `head_controller` is the only writer; the leg controller sends
  NaN in the head slots of `/joint_commands` and `servo_driver` skips them.

- **OQ-13 — IMU filter conventions.** `imu_filter_madgwick` was added on 2026-09-09 to
  provide `/imu/data` (DEC-19). Whether its ENU yaw sign and frame match what the
  controller's complementary filter and balance loop assume has not been checked against
  the robot turning. Check before trusting yaw fusion.

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
  before buying anything.

- **OQ-21 — The head has no feedback, so its joint states are a model.** (2026-09-19.)
  `head_controller` publishes head joint states from a slew-rate model (`slew_rate`,
  default 300 °/s, unmeasured), and that model is what places `ultrasonic_link` on the TF
  tree. If it is wrong, or a servo stalls, readings taken while the head moves land at the
  wrong bearing in the map. Also unverified: `pan_direction` (which way a rising servo
  angle turns the head) and the travel limits, kept at ±40° inside the vendor app's 50–180
  clamp. Needs the battery and the owner: command known angles, watch the head, and check
  the dashboard's sonar fan against a target at a known bearing. Cheapest mitigation if the
  model proves poor: only trust readings taken while the head is settled.

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

- **OQ-25 — Restart the stack under linger (family startup rule 10).** (2026-09-21.) Observed
  from the family's mission-planner hosts on the LAN: `/imu/data` and `/tf` delivered nothing,
  and `/joint_states` and `/ultrasonic/range` came and went, while `/imu/data_raw` kept
  arriving. On the Pi, `hexapod.service` had been active since 2026-09-20 with `Linger=no` and
  **no `fastrtps_*` segment in `/dev/shm`**. logind's `RemoveIPC` had deleted them when an SSH
  session closed, which breaks same-host Fast DDS delivery: `imu_filter` stops getting
  `/imu/data_raw`, for one. Linger was enabled on the Pi on 2026-09-21, and
  `systemd/install.sh` now enables it too. **Open: the restart.** The segments come back
  only when the stack restarts, and the unit starts autonomy. So the restart waits until the
  robot is safe to explore. A manual `scripts/launch.sh` with the service stopped does not
  explore (OQ-24), so it can test the fix first. Resolves when, after a
  restart, `/imu/data` and `/tf` arrive and keep arriving through SSH sessions.

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
