# Test log

What was actually run, on what date, under what conditions — including negative results.
Entries before 2026-09-09 are not recorded; the commit messages of that period are the only
record, and they describe intent rather than outcome.

## Format

Each entry: date · what was tested · conditions · result · what changed as a result.

## Entries

### 2026-09-15 · First battery run of the native stack

**Conditions:** robot on the floor, 2× 18650, `hexapod.service` with `autonomy:=true`,
cold boot. LOAD rail 8.24 V, CTRL rail 7.94 V (`/battery/voltages`, LED ring green on
both). Owner present.

**Result:** the robot homed, stood and stepped in place; it did not explore. Three causes,
all in the log:

1. **Nav2 goals aborted within seconds.** Both frontier goals produced a first path
   (`controller_server: Received a goal`) and then died on the 1 Hz replan:
   `Timed out while waiting for action server to acknowledge goal request for
   compute_path_to_pose`. `bt_navigator.default_server_timeout` is 20 ms; the load average
   was 15 in the first minutes after boot (OQ-02). The 2026-09-09 bench run showed the same
   message once. The velocity mismatch of OQ-01 was never reached as a limit: the second
   goal was followed for 15 s and `/odom` moved 5 mm, consistent with 2.5 mm per cycle.
2. **A 24 min forward clock jump 45 s after boot.** The Pi 5 RTC is unbacked; `timesyncd`
   restored the recorded shutdown time (16:29:16) and stepped to NTP time (16:53:42) once
   the network came up. Consequences: `frontier_explorer` measures its timeout with
   `time.time()` and ended the exploration action with `Timeout reached` (300 s "elapsed")
   after two goals; the collision monitor rejected `/scan` for 4 s (timestamps 1391 s
   apart, robot told to stop); RTAB-Map logged extrapolation errors and `Did not receive
   data since 5 seconds`. The service has `After=network-online.target` only;
   `systemd-time-wait-sync.service` is disabled. The 2026-09-09 runs never saw this because
   the host had been up for days. Resolved the same day by DEC-21.
3. **RealSense `Depth stream start failure … Hardware Error`** logged once at start; the
   stream recovered (`/scan` at 3 Hz, `/map` 143×92 cells at 5 cm).

End state: `autonomy_manager` in `exploration_complete`, robot standing, mission inactive.
Servos held the stand for the whole run without a brownout on either rail.

**Second run, 17:03, after DEC-21:** no acknowledge timeouts; each goal was followed for
the full 30 s and then aborted by the progress checker (`Failed to make progress`, three
times). The robot stepped in place and turned left very slowly — the 0.3 rad/s
rotate-to-heading command as 3.6° per cycle, OQ-01 exactly as predicted. Stack stopped at
17:06 with the battery still full.

**Third start, 17:13, `autonomy:=false` for the movement calibration (DEC-22 code):** on
the stand the rear-left leg folded over its knee joint and rested on top of it (owner's
observation; the first such event recorded). Servos relaxed at 17:15 by `/pose_command
relax` for the leg to be repositioned by hand. Cause not known: the stand path is
unchanged by DEC-22. Homed and stood again cleanly.

**Movement calibration, same start:**
- Yaw sign (OQ-13): robot turned 90° anticlockwise by hand; `/imu/data` yaw read
  +77.6°. Sign correct; magnitude not checked further.
- Forward, `linear.x = 0.05` for 20 s, IMU fusion off: the robot walked forward
  (direction correct) but only 5 gait cycles ran — each took 4.21 s against the nominal
  1.0 s (controller warning). Odometry 0.25 m commanded; tape about 0.15 m, so
  provisional `stride_scale` ≈ 0.6 at that cycle rate; the floor is rubberised and the
  feet grip (owner), so the shortfall is in the gait or the servos, not slip. Load average was 1.0 (autonomy
  off), so the slow cycle is in the servo path, not CPU contention: the driver makes 80
  single-byte I2C writes per frame (0.16 ms each, measured) and the controller's reliable
  publisher waits on it — but profiling (`py-spy`, next run) showed the driver 5 % busy
  and the controller's gait thread mostly asleep, so the loss is in wake latency, not
  I2C.
- Turn, `angular.z = 0.3` for 10 s, fusion off: turned left (direction correct); 3
  cycles, `/joint_commands` at 30 Hz against the nominal 64. Odometry +51.6° commanded,
  IMU +36.2°, owner's estimate about 35°. Provisional `turn_scale` ≈ 0.7.
- Fix applied: both gait loops now sleep to absolute deadlines instead of a fixed
  `delay` per frame, the interpreter switch interval is 1 ms, the gait tick timer is
  50 ms.
- Retest of the forward run after the fix (stack restarted): 19 cycles in 20 s,
  `/joint_commands` at 67 Hz, no slow-cycle warning, odometry 0.95 m commanded — but
  three legs goose-stepped and three dragged. Cause: two conventions for the floor
  height. The stand leaves the feet at world z = 0 with the body raised, the post-walk
  reset put the feet at the body height (−30 mm, so the robot stood 60 mm tall after any
  walk), and the vendor-port gait lifted the odd tripod to an absolute height measured
  from the body reference while the even tripod lifted relative to the feet. A first
  correction (lift relative to each foot's current height) made the odd tripod climb
  40 mm per cycle, because those feet enter a cycle already lifted. Final form: a single
  `GROUND_Z = 0` used by the initial feet, the stand reset and the odd-leg lift.
- Forward again on that code: all six legs cycled normally (owner), 18 cycles in 20 s,
  `/joint_commands` at 59 Hz, odometry 0.90 m commanded, tape 0.74 m —
  **`stride_scale` = 0.82**.
- Turn again on that code, `angular.z = 0.3` for 10 s: 9 cycles, odometry +154.7°
  commanded, IMU +110.6°, owner's by-eye estimate 95–100° — **`turn_scale` = 0.72** from
  the IMU (0.70 on the earlier slow run; the by-eye figure would give 0.63). Both
  factors are now in `body_params.yaml`. The 18–28 % shortfall on a gripping floor is
  in the legs (servo deadband, link compliance, the rounded IK angles), not the floor.

**Full stack on the battery, 17:43, `hexapod.service` with autonomy:** homed, stood,
entered mapping mode and exploration; **the first frontier goal (0.71, 0.04) was
reached in 23 s** — the first autonomous navigation of this robot — and the explorer
sent the next goal (1.20, 0.61), reached at 17:45:45, then a third. **It walked into
obstacles**: the collision monitor logged no stop for the whole run, and the owner
turned the robot into free space by hand twice; it went back to the same obstacle each
time — RTAB-Map re-localised visually and corrected `map → odom`, so the manual moves
did not break the goal (gait odometry never saw them). Analysis, unverified on the
robot: the scan is a 10-pixel band at the camera's own height with `range_min` 0.2 m,
the D435i's minimum depth at its default profile is about 0.28 m (Intel's figure), and
the stop polygon reaches only 0.22 m ahead of `base_link` — an obstacle inside the
polygon is inside the camera's blind range, so the monitor can never fire on it
(OQ-03). Load average 23 while exploring (OQ-02). Battery 7.71 V LOAD, 7.65 V CTRL
after about 55 min on the cells, down from 8.24/7.94 at the first boot. Service
stopped at 17:47.

**What changed (evening):** DEC-22 completed with the measured factors; the frame loop,
the floor reference and the cycle timing fixes are part of it.

**What changed:** DEC-21 (time-sync ordering, monotonic clocks,
`default_server_timeout` 20 → 1000 ms); DEC-22 (SI `cmd_vel` on the vendor gait, OQ-01
closed); OQ-02 gains the boot-load measurement; OQ-16 opened.

### 2026-09-09 · State of the container before removal

**Conditions:** the Docker container that had run since 2026-01-06, image built 2025-12-31,
robot on USB power.

**Result:** nine hardware nodes up; the controller logged `Hardware init failed: 'GPIO
busy'` on every boot (servo power pin already held by `servo_driver`) and `No calibration
file, using defaults` (container-only path). The image predated RTAB-Map, Nav2 and the
autonomy package, and the compose file that launched it would have crash-looped on its next
start (it sourced an RTAB-Map overlay the image did not contain). Nothing above the drivers
had ever run on the robot.

**What changed:** DEC-07, DEC-09, DEC-17.

### 2026-09-09 · Host apt state

**Conditions:** `apt-get install` of the ROS 2 Jazzy packages on the robot's Ubuntu 24.04.

**Result:** 22 packages from the Raspberry Pi OS bookworm repository (an earlier Pi camera
attempt) blocked Nav2, RTAB-Map and RealSense with `libswresample4` / `libwayland-*`
version conflicts. Downgraded 11 to noble, removed `libavutil57`, `libssl3`, the
`libcamera*` and `rpicam-apps*` set; `initramfs-tools` and `pastebinit` left as Pi builds.
Every ROS package then installed from apt; only dlib compiled (about 20 minutes, twice
interrupted by power cycles). A `cc` wrapper in the user's local bin was also shadowing the
C compiler and broke the first CMake build.

**What changed:** [`operations.md`](operations.md#foreign-packages); the setup script warns
if `+rpt` packages are present.

### 2026-09-09 · Buzzer incidents — **two, both harmful**

**Conditions:** USB power. (1) `docker compose down` at about 15:05 killed the container's
buzzer node (SIGKILL after the 10 s stop timeout). (2) At about 16:04 a script holding
GPIO 17 low was killed by a `pkill -f` that matched its own command line.

**Result:** in both cases GPIO 17 floated and the shield buzzer sounded continuously until
the robot was unplugged. Between the two, a plain `sudo reboot` did not stop it. The noise
was painful for a family member.

**What changed:** DEC-10 — firmware `gpio=17=op,dl` and `gpio=4=op,dh`,
`hexapod-buzzer-guard.service`, `buzzer.enabled: false`. Verified afterwards: `gpioinfo`
shows GPIO 17 owned by `gpioset` from boot; four further stack restarts and one service
handover produced no sound.

### 2026-09-09 · RealSense D435i natively

**Conditions:** `realsense2_camera_node` 4.58.1 alone, with `realsense.yaml`.

**Result:** device found (serial `032622073916`, FW 5.17.0.10, USB 3.2). The
`depth_module.profile` / `rgb_camera.profile` keys were ignored (opened 848×480×30 and
1280×720×30); renamed to `depth_module.depth_profile` / `rgb_camera.color_profile` and
640×480×15 was honoured. Topics are `/camera/camera/...`, not `/camera/...` as the SLAM
launch assumed. No point cloud subscriber anywhere.

**What changed:** `realsense.yaml`, remaps in `realsense_slam.launch.py`, point cloud off.

### 2026-09-09 · DDS discovery on the host

**Conditions:** `ros2 topic pub` in one process, `ros2 topic list --no-daemon` in another.

**Result:** empty after 4 s, complete after 10 s. Node and topic lists from `--no-daemon`
queries stayed partial (15 of 19 nodes at 60 s). With the daemon running, lists were
complete. Interfaces present: WiFi, an overlay VPN interface, and a Docker bridge.

**What changed:** the working convention in `AGENTS.md`; verification uses the daemon.

### 2026-09-09 · Full native stack, first launch

**Conditions:** `scripts/launch.sh autonomy:=true`, USB power.

**Result:** 15 of 19 nodes reported (discovery), all processes alive. Defects found, in
order, each fixed and re-run:

1. `startup_sequence`: `Executor is already spinning` (`spin_once` inside a callback) and
   the auto-start timer re-fired every 2 s because the wrong timer was destroyed.
2. RTAB-Map: `TF of received image for camera 0 ... is not set` — no
   `base_link → camera` chain, because the head joints are revolute and nobody published
   their joint states.
3. `look_around` and `frontier_explorer`: `no running event loop` (`asyncio.sleep` inside
   an rclpy coroutine); the autonomy manager went to `error`.
4. RTAB-Map in localization mode against an empty database (the working DB file counted
   as a saved map); `/map` never published.
5. Nav2 not launched at all; explorer failed with `Nav2 action server not available`.
6. Nav2 on Jazzy: `nav2_navfn_planner/NavfnPlanner` not found (`::` form required);
   `collision_monitor` parameters uninitialised; `docks: []` aborted the docking server;
   `ID [ComputePathToPose] already registered` (explicit BT plugin list); empty behaviour
   tree from `default_nav_to_pose_bt_xml: ""`.
7. Collision monitor flapping stop/continue: `odom → base_link` stalled during every gait
   cycle (blocking callback on a single-threaded executor); 607 extrapolation errors in
   150 s.
8. Explorer counted every Nav2 abort as a reached frontier (206 "reached" in 30 s).

**What changed:** DEC-09, DEC-11, DEC-12, DEC-06, the asyncio fix, goal-status checking
and frontier blacklisting. Final run of the day: 36 nodes, `Managed nodes are active`,
state machine `waiting_for_startup → checking_map → mapping_mode → exploring`, `/map`
76×55 cells at 5 cm from the bench position, Nav2 planned and passed paths to the
controller, **0** collision-monitor stops and **0** extrapolation errors, Nav2 then
`Failed to make progress` (servos unpowered; OQ-01). Load average 10–12. Handed over to
`hexapod.service` with the same result.

### 2026-09-09 · IMU topic mismatch found by inspection

**Conditions:** code review during the documentation pass; not a runtime test.

**Result:** `imu_driver` publishes `/imu/data_raw` without orientation; the controller
subscribed to `/imu/data` and read a quaternion. Nothing published `/imu/data`, so the
odometry's IMU yaw fusion (DEC-04) had never received a message since it was written.

**What changed:** DEC-19 — `imu_filter_madgwick` added to the hardware launches.
**Untested on the robot**: OQ-13.

### 2026-09-16 · Explore start, dashboard crash, LOAD rail reading 2.5 V

**Conditions:** boot stack under `hexapod.service`, robot on the floor, cells charged
overnight, LOAD indicator LED red.

**Result:** with no saved map the state machine entered `exploring` on its own about 30 s
after boot and the explorer sent a frontier goal. `scripts/mission.sh start explore 600`
then **killed the dashboard**: the JSON integer `600` was assigned unchanged to the
`float32 timeout_sec` request field and the generated C conversion aborted
(`Assertion PyFloat_Check(field) failed`, exit -6). The launch does not respawn the
dashboard, so the mission API was gone until it was restarted by hand.

`/battery/voltages` over three samples: LOAD 2.47–2.53 V, CTRL 7.88 V. Two charged
18650s cannot read 2.5 V, so the LOAD rail is not seeing its cells (switch, cell
seating or contact; unverified which). Nav2 was commanding 0.3 rad/s, the IMU gyro
read about 0 and odometry had dead-reckoned to (3.0, −2.2) m: **the robot was
stationary while the map filled with commanded motion.**

**What changed:** the dashboard coerces `mission_type`, `timeout_sec` and
`return_home` to the service field types and returns 400 for a non-numeric timeout.
A second `start explore` with an integer timeout then answered
`accepted: true, message: Exploration goal rejected` because the explorer was already
active; `mission_active` stayed false. Untested: the same call with the explorer idle.

### 2026-09-18 · SSD moved to a USB 3 enclosure; forced TRIM hung the disk

**Conditions:** bench, USB power, `hexapod.service` running, AI HAT+ 2 not fitted.

**Result:** the 128 GB NVMe SSD, moved from its M.2 HAT to a USB 3 enclosure (Realtek
RTL9210B, UAS, 5 Gbps, `mq-deadline`), booted first time: root and boot mount by label.
No USB resets or I/O errors in the log. `BOOT_ORDER` changed `0xf146` → `0xf14`;
bootloader updated 2025-11-05 → 2025-12-08. `fstrim` failed with "discard operation is
not supported": the bridge reports UNMAP supported (LBPU=1, 20971520 blocks max) but
LBPME=0 in READ CAPACITY, so the kernel set `provisioning_mode=full`.

**Negative result:** with `provisioning_mode` forced to `unmap`, `fstrim -v /` **stopped
the disk responding**; the host accepted no new SSH sessions and was power-cycled by
hand. The exact command sequence was lost with the session. After the reboot: ext4 root
"clean", no journal replay logged; the FAT boot partition's dirty bit set and cleared
with `fsck.vfat -a` after a live unmount; the udev rule written seconds before the hang
was on disk as a zero-byte file and was removed; journald renamed one corrupt journal
file. No kernel message from the hang survived.

**What changed:** DEC-23, then DEC-24 the same day: the AI HAT+ 2 cannot be fitted
alongside the Freenove shield (it is powered through the GPIO header and cannot be
stacked on), so it goes to the tank bot and the SSD returns to the M.2 HAT. `BOOT_ORDER`
flashed back to `0xf146` before the move. **Untested:** the boot after the SSD returns
to PCIe, and `fstrim -v /` on it.

### 2026-09-19 · Sonar and head stack, USB power

**Conditions:** bench, USB power (servos, including the head's, unpowered), service
stopped, load average 4 before the run. D435i removed and the kit's camera and HC-SR04
refitted the same day (DEC-25).

**Ultrasonic, before any ROS code:** a one-off probe on the vendor pins (trigger GPIO 27,
echo GPIO 22) timing the echo from kernel line-event timestamps (lgpio alerts): **15
consecutive pings against a fixed target, 0.222–0.224 m**. The edge ticks are
`CLOCK_MONOTONIC`, which is what the driver back-dates each message's stamp with. This is
the measurement that undermines DEC-02's "software-timed echo is unreliable".

**Camera: not working.** `ov5647: probe of 10-0036 failed with error -121` (no I2C
acknowledge) at boot and on a re-bind; libcamera 0.7.2 then reports "no cameras
available" and `camera_ros` aborts. The overlay (`camera_auto_detect=0`,
`dtoverlay=ov5647,cam0`), the PiSP pipeline and an OV5647 tuning file are all present, so
the fault is physical ([OQ-23](open-questions.md)). Bench runs use `camera:=false`.

**Stack, drivers only** (`scripts/launch.sh autonomy:=false camera:=false`):
`/ultrasonic/range` at **15.00 Hz**; `base_link → ultrasonic_link` present and rotating
with the head (0.130, 0.005, 0.050 m at 10° pan); head joint states published by
`head_controller` alone; `/head_command` stepping in servo degrees. One `LookAround`
survey: **57 ranges in a single sweep.**

**Full stack** (`autonomy:=true camera:=false`): all lifecycle nodes active and the chain
ran unattended — `waiting_for_startup → look_around` (2 sweeps, **112 ranges**) →
`mapping_mode → exploring`, explorer reading `/global_costmap/costmap` and sending a
frontier goal at (0.41, 0.30). The map at that point: 240×240 at 5 cm, **281 free cells,
135 lethal, 894 inflated**, rest unknown. Nav2 then reported `Failed to make progress`,
as expected with no servo power. The dashboard answered with `sonar_range: 1.692`,
`map_available: true`, state `exploring`.

A second run, this time under `hexapod.service` after a rebuild: survey of **142 ranges**,
then the explorer ended immediately with "Only frontiers within min_goal_distance remain".
That is the correct answer to the map it had — with the head physically still, the sonar
fills one cone and everything unknown is against the robot's own body — but it is also a
reminder that the exploration end condition has only ever been exercised on a degenerate
map.

**What this does not prove:** the head servos never moved, so every ping was taken at one
physical bearing while the joint-state model swept. **The pipeline is verified; the map's
geometry is not** ([OQ-21](open-questions.md)). Also untested: the collision monitor
against a real obstacle, the camera, and anything involving walking.

**Defects found and fixed during the run:** `ultrasonic_driver` used `self.handle`, which
collides with rclpy's `Node.handle` (`AttributeError: handle cannot be modified after node
creation`); and `bt_navigator` refused to configure because the packaged
navigate-**through**-poses tree still required the `Spin` behaviour that DEC-27 removed —
that tree now has a repository copy too.

**Observed, not explained:** `/battery/voltages` read LOAD 0.06 V, CTRL 0.00 V on USB
power. The LOAD rail reading is consistent with 2026-09-16; CTRL reading 0.00 V is new
and unexplained. Load average with the full stack up: **19–23** on four cores, 23 % idle,
with the leg controller at 51 % of a core and `head_controller` at 30 % before its update
rate was lowered from 50 to 20 Hz ([OQ-02](open-questions.md)).

### 2026-09-22 · First cold boot under the family startup rule; the linger fix confirmed

**Conditions:** unplanned power cycle — the Pi had been off the network since about 22:00 on
2026-09-21 and came back at **22:16 UTC** on 2026-09-22; no owner action at the robot after
that. SBC on mains, **servo rails dead throughout**: `power_indicator` read LOAD 0.00 V and
CTRL 0.00 V (BLUE/USB), so no leg could move whatever was commanded. `hexapod.service`
enabled, started by systemd with `autonomy:=true`. Observed from the workstation over SSH.

**Result — the unit came up unattended and needed no intervention.** This is the first cold
boot since [OQ-24](open-questions.md) put the startup on the family rule:

- Started **22:16:39**, the same second `systemd-time-wait-sync` finished (`Result=success`),
  so the clock was right before the stack ran. `NRestarts=0`, still active when checked.
- 29 launch entities started. **The buzzer stayed silent and said so**: `Buzzer DISABLED
  (buzzer.enabled=false) - beep requests ignored`.
- The startup sequence ran to completion against the dead rails — rear LED yellow while
  waiting, warning, `home`, the 10 s place delay, `stand`, then `Phase 6: SAFE`.
- Autonomy ran its own sequence: `look_around` (head survey, 146 ranges) → `mapping_mode` →
  `exploring` → `exploration_complete` in 11 s, "Only frontiers within min_goal_distance
  remain". Nothing beyond the two pose commands above was sent to the legs.
- **`camera_ros` aborted 4 s in** (`std::runtime_error`, exit -6), and the kernel shows
  `ov5647 10-0036: probe of 10-0036 failed with error -121` — the unchanged
  [OQ-23](open-questions.md) signature, not a startup regression. The other 28 entities were
  unaffected and `ros2 launch` did not exit, so systemd had nothing to restart.

**The linger fix works ([OQ-25](open-questions.md) resolved).** `Linger=yes`, and `/dev/shm`
held **226 `fastrtps_*` entries** when measured after four SSH sessions had opened and closed
since boot — under the fault they were deleted when a session closed. From the workstation on
the LAN (`ROS_DOMAIN_ID=0`, `SUBNET`, 10-sample windows): `/imu/data` **108 Hz**,
`/imu/data_raw` **100 Hz**, `/tf` **50 Hz**, `/joint_states` **67 Hz**, `/ultrasonic/range`
**15 Hz**. `/imu/data` and `/tf` are the two that delivered nothing under the fault.

**Observed, not explained:** the *graph listing* from the workstation is incomplete while the
*data* flows. `ros2 node list --no-daemon` over 25 s returned **one of the robot's 33 nodes**
(`/autonomy_manager`) and `ros2 topic list` **8 of its 23**, while `ros2 topic hz` on the five
topics above delivered at full rate. On the robot itself the same two commands list all 33 and
23. Recorded at family level in
[wk-robotics `common.md`](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/common.md#ros-2-installs-are-familial)
— it is the mirror image of the 2026-09-21 observation there, which had names without data.

### 2026-09-28 · RPLIDAR C1 on the printed plate, and slam_toolbox on the bench

**Conditions:** plate printed and fitted the same day; USB power (both rails 0.00 V, read
off the ADS7830 directly), legs limp; robot on a desk, stationary. Owner present for the
yaw test.

**Result:**

1. **apt's `rplidar_ros` 2.1.0 cannot drive the C1.** SDK 1.12 read the serial number,
   firmware 1.02, hardware rev 18 and health 0, then `Cannot start scan: '80008002'` and
   `Failed to set scan mode`, at 460800 baud.
2. **Slamtec's `sllidar_ros2` works** (commit `3430009`, SDK 2.1.0, `sllidar_c1_launch.py`
   parameters): Standard mode, 5 kHz, **10.000 Hz** by `ros2 topic hz`, 720 beams, 628 with a
   range, 0.29–6.7 m.
3. **`slam_toolbox` 2.8.5 (online async) built a map** within seconds: 76 × 231 cells at
   5 cm, `map → laser_frame` resolving through gait odometry. One "queue is full" drop at
   start, none after. CPU at rest: `slam_toolbox` 4.5 %, `sllidar_node` 3.9 %, load average
   ~5 without Nav2.
4. **Laser yaw π**, from objects the owner placed and measured from the lidar centre:
   boxes 500 mm dead ahead → 0.494–0.50 m at 180°; wall 270 mm behind → 0.274 m at 0°;
   pole ~240 mm behind-left → 0.234 m at −50°, which rules out a mirrored scan. A
   photo-based guess of −90° was wrong. The first live check of the change (yaw 180°) was
   luck: see 6. After the fix below, three consecutive reads gave x 0.010, y 0, z 0.160,
   yaw 180°, once the owner's plan position (10 mm forward, centred) was added.
5. **A hand-started stack ignores SIGINT** (`setsid nohup … &`); SIGTERM to `ros2 launch`
   orphaned the nodes, which then exited on SIGTERM by PID. GPIO 17 stayed held by the
   guard throughout. Recorded in [`operations.md`](operations.md#development-loop).
6. **The cleanup missed two orphans**, because it matched only this repo's node paths:
   `robot_state_publisher` (publishing the original URDF, laser yaw 0) and `imu_filter`
   ran for ~50 min alongside `hexapod.service`, so `/tf_static` carried two conflicting
   `base_link → laser_frame` and `/imu/data` had two publishers. No harm on USB power (the
   controller fuses IMU yaw only while walking). SIGTERM by PID ended both; `/imu/data`
   back to one publisher.

**What changed:** DEC-31; `ros2_ws/deps.repos`; `lidar:=true` in `robot.launch.py`;
`laser_frame` in the URDF; `slam.launch.py` and `config/slam_params.yaml`. Not tested:
anything walking, a leg or the head in the scan plane.

### 2026-09-29 · Cold boot on the battery: the head surveyed, the legs never initialised

**Conditions:** cold boot on 2× 18650, `hexapod.service`, checkout `fba11e8` (the first
battery boot with `sllidar_node` in the stack). LOAD 7.47 V, CTRL 8.18 V at 12:08:01 UTC;
7.29 V and 8.06 V at 12:10:55 (dashboard `/status`). `throttled=0x0`, 59 °C, load average
12.7 at 2 min. Inspected over SSH from the workstation, read-only; the owner watched the
robot and reported the head moving and no walking.

**Result:**

1. **The `home` pose command was lost, so the controller stayed uninitialised.**
   `startup_sequence` published `home` on `/pose_command` at 12:07:53.65; `hexapod_controller`
   finished starting at 12:07:55.47, 1.8 s later. `stand` at 12:08:04.66 was refused:
   `Not initialized - send home first`. With `is_initialized` false `_gait_tick` returns
   before reading `/cmd_vel`, so nothing walks. The leg state was not observed from the
   workstation.
2. **Nothing noticed.** `startup_sequence` logged `Phase 6: SAFE - robot ready`, showed
   green and published `/robot/initialized`; it never checks the controller. The state
   machine ran `look_around` (survey: 31 of 34 ranges) → `mapping_mode` → `exploring`, and
   the explorer chose a frontier at (1.26, 0.18). Nav2 then logged `Failed to make progress`
   every 30 s (12:08:47, 12:09:17, 12:09:53, 12:10:24), one planner failure to that goal,
   and a failed `backup` recovery. Dashboard state at 12:10:55: `exploring`, progress 0.0.
3. **The race is not new and not the lidar's.** The journal on the robot holds five starts
   with both lines:

   | Start (UTC) | `home` sent relative to `controller started` | `stand` |
   |---|---|---|
   | 2026-09-19 11:34 | 4.3 s before | refused |
   | 2026-09-19 14:44 | 0.4 s before | accepted |
   | 2026-09-19 16:12 | 2.9 s before | refused |
   | 2026-09-23 10:04 | 0.9 s after | accepted |
   | 2026-09-29 12:07 | 1.8 s before | refused |

   The two earlier refusals were on USB power, where dead servos hid them.
4. **Also in the log:** `camera_node` died at start (exit −6, OQ-23); 64 `No echo pulse from
   HC-SR04` warnings in the first 2 min; one `Failed to read ADC: [Errno 121]` from
   `battery_monitor`.

**What changed:** opened [OQ-28](open-questions.md); the fix is the next entry.

### 2026-09-29 · The startup fix on the battery: it stood, walked, and hit a TV stand

**Conditions:** same session and battery, commit `a077db8` deployed by the
[development loop](operations.md#development-loop), `hexapod_hardware` and
`hexapod_controller` rebuilt. LOAD 7.76 V, CTRL 8.06 V before the restart. Robot on the
floor of a furnished room, owner present. Logs read over SSH; contact reported by the owner.

**Result:**

1. **`systemctl restart hexapod` ended in a reset.** The old stack relaxed the servos at
   12:16:13 and the new one began starting at 12:16:16. The kernel logged
   `hwmon hwmon4: Undervoltage detected!` at 12:16:20.83, the journal's last line; the Pi
   came back as a fresh boot with the stack starting at 12:17:35. Whether the owner touched
   the power switch was not asked; what drew the rail down is unknown
   ([OQ-29](open-questions.md)).
2. **The fix held on that boot.** The sequence began at 12:17:44.15 and waited 2.7 s for
   both subscribers (controller ready 12:17:47.19). `home` at 12:17:48.86 was executed by
   the controller 12 ms later and confirmed within 0.1 s; `stand` at 12:17:59.47 was
   accepted; `SAFE` at 12:18:00.97. One boot; the timeout paths (red LED) were not
   exercised.
3. **It walked.** `look_around` → `exploring` at 12:18:12. `/odom` x read 0.514 m, then
   0.558 m some 10–15 s later (~12:19:00). `Gait cycle took 1.76s against a nominal 1.00s`
   (logged once). The first frontier (1.29, 0.39) ended in `Goal failed` at 12:19:15 after
   two successful `backup` recoveries; the second (1.53, 0.81) logged `Failed to make
   progress` at 12:19:50.
4. **It walked into a TV stand** (owner; time not recorded). The collision monitor's first
   `Robot to stop due to PolygonStop` is at 12:19:20.83; from 12:19:31 it alternated
   between stop and 50 % slowdown several times a second, with the sonar reading
   0.29–0.31 m. Whether the first stop preceded the contact is unknown.
5. **The lidar did not take part.** `sllidar_node` published `/scan`; `slam_toolbox` was
   not running and nothing in Nav2 subscribes to `/scan`, as DEC-31 states. Costmaps and
   collision monitor saw the sonar only.

**What changed:** OQ-28 resolved; OQ-29 opened; OQ-03 and OQ-20 carry the collision. This
run is not the sonar battery run expected below: no map was checked against the room.

### 2026-09-29 · Head disabled: the robot stands and finds no frontier

**Conditions:** same session and battery, commit `b8f82b2`, after the owner heard the tilt
servo's gears jumping ([OQ-30](open-questions.md)). The stack was stopped at 12:21:33 UTC
(servos relaxed), the two packages rebuilt, and the service started at 12:23:07, not
restarted. LOAD 7.82 V, CTRL 8.00 V before the stop; 7.59 V and 7.94 V standing afterwards.

**Result:**

1. **Both switches took.** `head_controller` and `servo_driver` each logged the head as
   disabled; `look_around` was rejected. Whether the servos are silent is the owner's to
   confirm.
2. **The startup fix held a second time**, on a service start: `home` and `stand` accepted,
   `SAFE` at 12:23:35.65. No undervoltage was logged (OQ-29: one reset in three starts
   today).
3. **No exploration.** `frontier_explorer` reported `No frontiers detected` 2 ms after it
   began, and the state machine went to `exploration_complete` at 12:23:37.37. With no
   survey the map holds one sonar cone; the sonar read 0.147 m, its direction unknown with
   the head limp. The robot stands in place.

**What changed:** OQ-30 opened. Autonomous exploration now waits on the lidar as the range
source ([OQ-20](open-questions.md)) or on a repaired head.

### 2026-09-29 · Lidar and slam_toolbox in the boot stack: maps, plans nearby, reaches no frontier

**Conditions:** same session and battery, head disabled, robot on the floor of a furnished
room, owner present; the room was not described to the session. Four starts of the
service, each after a stop. LOAD 7.65–7.88 V and CTRL 7.88–7.94 V on the first three; load
average 9–16. Times UTC.

**Result:**

1. **12:30, `4c93b5c`: `slam_toolbox` ran on its defaults.** `Failed to compute odom pose`
   on every scan (271 in 40 s), no `/map`. `base_frame` read `base_footprint` (OQ-27):
   included from `navigation.launch.py`, `slam.launch.py`'s `params_file` argument
   resolved to Nav2's file. `/scan`, `/odom` and TF were healthy (10.1, 20.1, 20.0 Hz;
   received at most 0.39 s after their stamps). Fixed in `6fb3027`.
2. **12:32, `6fb3027`: a map, and no plan.** `/map` 74 × 153 cells within 13 s. NavFn
   failed every plan, including one to a clear point 0.6 m ahead, with `Failed to create a
   plan from potential when a legal potential was found`. Start cell cost 79–82 of 100.
3. **12:35, `dbcbff3`: Smac 2D also found no path.** `/map` and a live scan had the same
   outline at an offset: 146 occupied cells in the map, 296 cells hit by the last ten
   scans, 16 in common. `slam_toolbox` had taken its first scan before the robot was
   placed and stood. Fixed in `8bbc417` by resetting SLAM and both costmaps on `SAFE`.
4. **12:39, `8bbc417`: plans to nearby points succeed; frontiers do not.** Plans to
   (0.30, 0.15) and (0.35, 0.40) succeeded. Frontiers at (0.54, 1.52), (−0.88, −2.39) and
   (−1.26, −2.39) each failed with `no valid path found`, and the state machine went to
   `error` at 12:41:06 after three. The reachable pocket was 0.79 m²
   ([OQ-34](open-questions.md)). `backup` was refused each time: the nearest return was
   0.34 m behind the robot.
5. **The lidar did not see the robot standing** (OQ-31): nearest returns by 30° sector,
   clockwise from dead astern, 0.34, 0.38, 1.03, 0.73, 0.83, 1.11 m (ahead), 0.95, 1.22,
   0.85, 0.97, 0.85, 0.36 m.
6. **The collision monitor on `/scan`** logged one stop and release, 0.1 s apart, in the
   12:35 run and none in the others. It was not tested against an obstacle.
7. **The I2C bus stopped answering at 12:39:58** and had not recovered when the stack was
   stopped at 12:44 ([OQ-32](open-questions.md)). Whether the legs moved in the 12:39 run
   is unknown; its results above come from the lidar and the costmaps, which do not use
   that bus.
8. **No undervoltage** was logged on any of the four starts (OQ-29: one in seven today).

**What changed:** DEC-32; Smac 2D; the explorer's `initial_map_timeout_sec`;
`scripts/scan-near.py`; OQ-31 to OQ-34. **Not shown:** the robot walking on the lidar, a
frontier reached, the map against the room. The stack was left stopped.

### 2026-09-29 · The I2C bus freed without a power cycle; the room seen by the lidar alone

**Conditions:** same boot as the entry above (up since 12:16 UTC), on the battery, stack
stopped since 12:44, servo power disabled (GPIO 4 high), driven over SSH from the
workstation. Nothing moved a leg. Robot's checkout `489fbe9`, then `40a234b`. Times UTC.

**Result:**

1. **12:58, the bus was still dead.** `i2cget` to `0x40`, `0x41`, `0x48` and `0x68` each
   failed after 1.0 s with a kernel `controller timed out`.
2. **12:59, SDA was held low.** RP1 pad registers, 200 samples over 1 s: SDA 0, SCL 1,
   neither driven by the Pi, both pins on function 3 (I2C).
3. **The journal, read again, corrects the entry above** on three points: the arbitration
   loss at 12:39:57.19, `servo_driver`'s death at 12:40:16, and the 12:40:05 voltages
   ([OQ-32](open-questions.md)).
4. **13:02, `scripts/i2c-recover.py --recover` freed it with one SCL pulse.**
   `i2cdetect -y 1`: `40 41 48 68`. Both PCA9685s read `MODE1` `0x00`, `PRESCALE` `0x79`,
   and no channel held a live PWM. The IMU read `WHO_AM_I` `0x70`
   ([OQ-35](open-questions.md)). No kernel timeout followed. The script was run from
   `/tmp` before it was committed; `40a234b` is the same file.
5. **13:02, the battery.** Three reads 0.5 s apart: LOAD 7.59, 7.47, 7.47 V; CTRL 7.53,
   7.59, 7.59 V. `vcgencmd get_throttled` `0x0`.
6. **13:03, the `servo_driver` guard against a stub:** three `TimeoutError`s logged and
   counted, a good call passed through, a `ValueError` still raised. Not tested against a
   dead bus.
7. **13:04, `sllidar_node` alone, 53 scans, robot lying with its legs limp.** Nearest
   return by 30° sector of body bearing (0° ahead, + left), from −180°: 0.55, 0.58, 0.72,
   0.61, 0.52, 0.79, 1.87 (0° to +30°), 0.62, 0.54, 0.54, 0.62, 0.90 m. None nearer than
   0.50 m. The 12:39 run had 0.34–0.38 m in the three sectors astern: the robot or what
   stood behind it has moved since, which one is not known. Health `OK`, 10.0 Hz.
8. **`timeout` on `ros2 run` orphans the node.** The signal reaches the `ros2 run` wrapper;
   the node and the static transform publisher were left under PID 1, and had exited
   seconds later when looked for.

**What changed:** `scripts/i2c-recover.py`, `scripts/sync-check.sh`, the `servo_driver`
guard, [DEC-33](decisions.md), OQ-32 rewritten, OQ-35 opened. **Not shown:** why
arbitration was lost, the stack running on the recovered bus, anything walking.

### 2026-09-29 · The stack on the recovered bus: it stands at 30 and 80 mm, and an object behind holds it still

**Conditions:** same boot and battery, owner present and watching the dashboard, robot on
the floor where the owner had put it; the room was not described to the session. Two
starts of the service, each from stopped with the checkouts in sync (DEC-33): 13:14:05 at
`829bc83` (30 mm stand) and 13:28:09 at `e8ec3ed` (80 mm stand, DEC-34). LOAD 7.41–7.53 V,
CTRL 7.29–7.41 V standing; load average 9.6–10.1. Times UTC.

**Result:**

1. **The bus held through both runs.** No kernel I2C message, no `servo_driver` error,
   `Servos relaxed` from both nodes at the 13:27 stop. No undervoltage on either start
   (OQ-29).
2. **13:14, 30 mm: stood, `SAFE` at 13:14:32, reached no frontier.** Goals (1.02, 1.01),
   (−1.11, 1.15) and (−1.69, 0.23) each failed with `no valid path found`; `error` by
   13:16. `backup` ran once, 13:14:40–44, and odometry went from (0.02, −0.03) to
   (−0.09, 0.09); every later `backup` ended in `Collision Ahead`.
3. **13:15:58, `scripts/costmap-reach.py`:** the planner could reach 147 cells, 0.37 m²,
   none beside unknown space; the robot's own cell cost 96; the goals' cells cost −1, 99
   and −1. The costmap held 430 lethal and 3650 inscribed cells against 1237 passable.
   **`/map` alone, 13:17:** 278 occupied cells, the robot's cell free, 3.21 m² reachable
   over free cells with 284 of them beside unknown space.
4. **13:18, the scan in the body frame, standing:** straight edges only. One 0.55 m to the
   left, 1.3 m long; one 0.30 m astern running 0.75 m to the right, nearest 0.277 m at
   −140°. Lying at 13:04 the nearest return astern was 0.55 m: the robot had backed
   towards it.
5. **13:26:39, three headings** ([OQ-36](open-questions.md)): `map → base_link` −27.9°,
   `/odom` −48.9°, `/imu/data` +4.4°. 552 of 586 scan points lay on `/map`'s occupied
   cells as TF placed them; the best fit over ±90° and ±0.4 m was 571 points at +0.5°.
6. **The dashboard's new map and lidar panels** were run beside the stack on port 8081 at
   13:24 from `/tmp`, then deployed in `e8ec3ed`. The camera node died at both starts:
   `no cameras available` (OQ-23).
7. **13:28, 80 mm: `Standing (raising body 80.0mm)` at 13:28:34, `STAND position set` at
   13:28:35, `SAFE` at 13:28:36.** LOAD 7.41 V after it. How the stance looked and
   sounded is the owner's to say.
8. **The collision monitor held it from 13:28:38 and did not release.** A path to
   (−1.48, 0.08) was found; `Failed to make progress` every 30 s from 13:29:08.
   `scripts/scan-near.py`, 200 scans from 13:29:12: nearest 0.283 m at −140°, in every
   scan, which is the point (−0.22, −0.18) inside `PolygonStop`. The ranges astern were
   the same at 30 mm and at 80 mm, 0.28–0.36 m: **not a knee, and taller than 0.24 m.**
9. **Returns from the legs while walking (OQ-31): still not measured.** The robot did not
   walk at 80 mm, and no scan was recorded during the 0.2 m it backed at 30 mm.

10. **The LOAD batteries ran down with the robot standing still.** 7.41 V at 13:28:26, 7.12 V
    at 13:31:49, the stack still running and the robot held by the collision monitor.
    **The owner switched the robot off when the LED ring showed the LOAD batteries
    red**, which `power_indicator` shows below 6.5 V. When is not recorded: the
    robot did not answer at 22:26. The journal of that boot holds `power_indicator`'s
    readings every 10 s up to the switch-off; they are the discharge at the 80 mm stance
    and have not been read.

**How it was left:** switched off, not shut down, so the file systems were not unmounted
([`operations.md`](operations.md#troubleshooting), *After an unclean power-off*). The
LOAD batteries are discharged. The robot's checkout is at `9fa34b5`; what `origin/main` has
since is documentation. `hexapod.service` is enabled.

**What changed:** DEC-34; the dashboard; `scripts/costmap-reach.py`; OQ-34 and OQ-31
updated, OQ-36 opened; OQ-11 has its first case. **Not shown:** a frontier reached, the
robot walking at 80 mm, the map against the room.

### 2026-10-01 · The IMU tilted by hand: its axes lie 90° from the body's; the LOAD drain of 2026-09-29

**Conditions:** first boot since the 2026-09-29 switch-off, USB power, legs limp, the stack
running from the service; `/imu/data` read at 1 Hz by an ad hoc recorder in `/tmp` using
the controller's own quaternion-to-Euler formulas. The owner tilted the robot by hand in
the order asked: nose down about 20°, left side down about 20°, then lifted it, turned it
90° anticlockwise and set it down. Checkouts: robot `9fa34b5`, workstation ahead by
documentation only. Times UTC, 13:38–13:40.

**Result:**

1. **Both file systems were clean** after the switch-off (ext4 state `clean`; `fsck.vfat
   -n` found nothing).
2. **The 2026-09-29 boot's end, from its journal:** the robot stood from 13:28:36 and never
   moved; exploration ended `Timeout reached` at 13:39:54; the last journal line is
   14:32:23. `power_indicator` logs only colour changes: LOAD 7.47 V at 13:14:25, first
   YELLOW 6.59 V at 13:14:35 (the stand), first RED 6.41 V at 14:12:36, last 6.47 V at
   14:20:26. So **about 64 min standing at 80 mm took LOAD from ~7.4 V to ~6.4 V**, with
   no walking.
3. **Level, lying on the floor:** roll 0.0°, pitch −2.5°, yaw −3°; raw accel (+0.4, 0.0,
   +10.0) m/s².
4. **Nose down:** the controller's *roll* went to −28…−30°, pitch stayed −1°; raw accel y
   −5.0 m/s², x unchanged.
5. **Left side down:** the controller's *pitch* went to −29…−34°, roll stayed −2°; raw
   accel x +5.0…+5.8 m/s², y unchanged.
6. **Turned 90° anticlockwise:** yaw went from −3° to +50°, holding +49…+50° afterwards;
   raw gyro z was positive during the turn.

**Reading, if the moves were made in the order asked (the owner has not yet confirmed
the order):** the IMU's y axis points along the body's forward axis and its x axis to the
body's right — the chip sits turned 90° from `base_link`, while the URDF's `imu_joint`
says `rpy="0 0 0"` and `imu_filter_madgwick` and the controller use the quaternion as if
the frames agreed. The controller's `imu_roll` is therefore the body's pitch and
`imu_pitch` the body's roll, with nose down and left down both reading negative. The yaw
sign is right for ENU (anticlockwise positive), and the magnitude was 53° for a turn
asked as 90°: the turn's true size was not measured.

**What changed:** OQ-13 and OQ-37 updated.

## Next entries expected

From [`roadmap.md`](roadmap.md) milestone 1, all needing the battery and the owner:

- Whether the I2C bus fails again (OQ-32).
- A run in open floor on the lidar, started at least 0.6 m from anything: a frontier
  reached, and the map against the room (OQ-34).
- The three headings after a measured turn (OQ-36, OQ-13).
- `scripts/scan-near.py` while walking (OQ-31).
- Collision monitor behaviour against a real obstacle, now that the source is the lidar
  (OQ-03).
- `/imu/data` yaw sign when the robot is turned by hand (OQ-13).
- The stance at 80 mm: servo load, battery drain, the gait (DEC-34). Read the LOAD
  readings of the 2026-09-29 boot first: `journalctl -b -1 -u hexapod | grep 'LOAD:'`
  (the boot's number may differ).
- The file systems after the 2026-09-29 switch-off.
