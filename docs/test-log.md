# Test log

What was actually run, on what date, under what conditions — including negative results.
Entries before 2026-09-09 are not recorded; the commit messages of that period are the only
record, and they describe intent rather than outcome.

## Format

Each entry: date · what was tested · conditions · result · what changed as a result.

## Entries

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
- Measured walking speed per `cmd_vel` value, and the fix for OQ-01.
- Collision monitor behaviour against a real obstacle (OQ-03).
- CPU load after the dashboard and explorer changes (OQ-02).
