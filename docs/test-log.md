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

## Next entries expected

From [`roadmap.md`](roadmap.md) milestone 1, all needing the battery:

- `/imu/data` present and yaw sign correct when the robot is turned by hand (OQ-13).
- Collision monitor behaviour against a real obstacle (OQ-03).
- CPU load after the dashboard and explorer changes (OQ-02).
- The movement calibration of [`operations.md`](operations.md#movement-calibration):
  yaw sign, forward and turn directions, the two slip factors (DEC-22).
- A battery run of the full stack after that: does Nav2 reach a frontier?
