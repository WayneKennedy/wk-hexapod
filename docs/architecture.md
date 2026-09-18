# Architecture

Every bus, node, topic and frame on the robot, and where it sits in the family's tier
model. Package sources are under `ros2_ws/src/`.

## Where it sits in the family

The family's [two-tier split](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/common.md#compute-the-two-tier-split)
puts deterministic loops on a microcontroller and everything else on a Pi. **The hexapod
has only the second tier.** Every device is a direct peripheral of the Pi 5; the only work
done outside the Pi's CPU is PWM pulse generation inside the PCA9685 chips (and, until
DEC-25, stereo depth inside the RealSense).

| Family tier | On the hexapod |
|---|---|
| Reflex (MCU, ~1 kHz) | **Absent.** IK, gait timing, IMU polling and safety all run in Python on the Pi |
| Intent (Pi 5, ROS 2) | Everything: drivers, controller, SLAM, Nav2, autonomy, dashboard |
| Mission planning (off-robot) | The dashboard HTTP API driven from another machine; interim, unauthenticated ([OQ-04](open-questions.md)) |

Why it works anyway: a six-legged walker is statically stable. If the Pi stalls, the
PCA9685 chips keep emitting the last pulse widths and the robot stands still. A balancing
robot could not tolerate that, which is why this design is the family's counter-example and
not its pattern (DEC-08). The costs of the flat design show up as CPU load
([OQ-02](open-questions.md)), scheduler-sensitive loops (DEC-09 exists because a gait cycle
once starved the odometry publisher), and a buzzer that floats on when its process dies
([`hardware.md`](hardware.md#the-buzzer-hazard)). A reflex-tier retrofit is
[OQ-09](open-questions.md).

## Buses

| Device | Bus | Node | Rate |
|---|---|---|---|
| 2× PCA9685 servo drivers (0x41: channels 0–15, 0x40: 16–31) | I2C bus 1 | `servo_driver` | on command |
| Servo power enable | GPIO 4 (low = enabled) | `servo_driver` | on init/shutdown |
| MPU6050 IMU (0x68) | I2C bus 1 | `imu_driver` | 100 Hz |
| ADS7830 ADC (0x48) | I2C bus 1 | `battery_monitor` | 1 Hz |
| WS2812 strip, 7 LEDs (PCB V2) | SPI0 MOSI | `led_controller` | on command |
| Buzzer | GPIO 17 | `buzzer_controller` (disabled) / `hexapod-buzzer-guard.service` | held low |
| RealSense D435i — **removed 2026-09-18 (DEC-25); the code still expects it** | USB 3 | `realsense2_camera` (`/camera/camera`) | 15 fps colour + depth |
| OV5647 camera, HC-SR04 ultrasonic — **fitted back by DEC-25; no driver yet** | CSI; GPIO | none ([OQ-19](open-questions.md)) | — |

## Nodes and topics

### Hardware (`hexapod_hardware`)

| Node | Subscribes | Publishes / serves |
|---|---|---|
| `servo_driver` | `/joint_commands` (20 angles: 6 legs × coxa/femur/tibia, pan, tilt), `/leg_positions`, `/leg_command`, `/head_command`, `/servo_relax`, `/pose_command` (relax only) | `servo_driver/initialize` service |
| `imu_driver` | — | `/imu/data_raw` (`sensor_msgs/Imu`, no orientation) |
| `imu_filter` (`imu_filter_madgwick`, in `hexapod_bringup` launch) | `/imu/data_raw` | `/imu/data` with orientation quaternion, no TF |
| `battery_monitor` | — | `/battery/voltages` (LOAD and CTRL rails), diagnostics |
| `led_controller` | `/leds/zone` (`zone:r,g,b`; zones r1–r3, l1–l3, rear, left, right, front, back, mid, all) | — |
| `buzzer_controller` | `/buzzer/state` | `buzzer/beep` service. **Disabled by default** (DEC-10): requests are logged and dropped |
| `power_indicator` | `/battery/voltages` | `/leds/zone` (left = LOAD rail, right = CTRL rail; blue below 0.5 V means USB) |
| `startup_sequence` | — | `/leds/zone`, `/buzzer/state`, `/pose_command`, `/robot/initialized`; `/robot/safe_startup` service |

### Locomotion (`hexapod_controller`)

`hexapod_controller` owns body-centric IK: foot positions are held in the world frame while
the body translates and rotates, and every servo angle is solved from leg geometry (coxa 33,
femur 90, tibia 110 mm) plus per-leg calibration offsets. It **does not touch the servo
hardware** in the robot stack (DEC-09); it publishes calibrated angles and `servo_driver`
writes them.

| Direction | Topic / interface |
|---|---|
| In | `/cmd_vel` (`Twist`), `/pose_command` (`home`, `stand`, `relax`), `/imu/data`, body pose command |
| Out | `/joint_commands` (per gait sub-step), `/joint_states` (50 Hz, includes `head_pan_joint`/`head_tilt_joint`), `/odom` (20 Hz), TF `odom → base_link`, `/servo_relax` |
| Services | `hexapod/initialize` (home then stand), `hexapod/enable_balance`, `hexapod/reset_odometry` |
| Action | `hexapod/move_distance` (`hexapod_interfaces/MoveDistance`) |

Gait: the vendor's tripod gait (legs 0,2,4 alternate with 1,3,5), one cycle per second by
default, 40 mm step height. `/cmd_vel` is SI: a gait worker runs one cycle per tick on the
latest command while it is fresh (0.5 s) and non-zero, moving the body `v × cycle_time`
per cycle up to the gait's limits — 140 mm and 40° per cycle, so 0.14 m/s and 0.7 rad/s
at the default cycle time (DEC-22). Odometry integrates the displacement the gait
geometry commands, scaled by measured stride and turn factors, and blends IMU yaw with a
complementary filter (weight 0.98).

The node runs on a multithreaded executor. Gait callbacks are mutually exclusive so steps
never overlap; odometry, joint-state and IMU callbacks run in a reentrant group so TF keeps
flowing during a blocking gait cycle.

### Perception and SLAM (`hexapod_bringup/launch/realsense_slam.launch.py`)

**As the code stands; its input, the D435i, has left (DEC-25).** The replacement is
[OQ-19](open-questions.md).

- `realsense2_camera` at 640×480×15 colour and depth, depth aligned to colour, IMU streams
  enabled but unused, point cloud off, **TF off** — the URDF owns the camera frames (DEC-12).
  Topics carry the `/camera/camera/` prefix.
- `depthimage_to_laserscan` → `/scan` from the aligned depth image, 0.2–3.0 m, frame
  `camera_depth_frame`.
- `rtabmap` (RGB-D, odometry from `/odom`) → `/map` (5 cm grid), `map → odom` TF, database
  in `~/.ros/rtabmap.db` (mapping, wiped per run) or `~/.hexapod/maps/rtabmap.db`
  (localization; DEC-11).

### Navigation (`hexapod_bringup/launch/navigation.launch.py`)

Nav2's `navigation_launch.py` only — planner, controller (regulated pure pursuit),
smoother, behaviours, BT navigator, waypoint follower, velocity smoother, collision
monitor, docking server (no docks). Map and localization come from RTAB-Map (DEC-06). Both
costmaps take obstacles from `/scan`; the local costmap is a 2 m rolling window with a
0.15 m robot radius.

### Autonomy (`hexapod_autonomy`)

| Node | Role | Interfaces |
|---|---|---|
| `autonomy_manager` | State machine: `waiting_for_startup → checking_map → (look_around → localization_mode → waiting_for_mission) or mapping_mode → exploring → exploration_complete / error` | `/autonomy/state` (`AutonomyState`), `/robot/initialized`; calls `/rtabmap/set_mode_mapping` when localization fails |
| `slam_monitor` | RTAB-Map localization status from loop closures | `LocalizationStatus` |
| `look_around` | Head sweep, body rotation, or both, to gather features for localization | `LookAround` action; `/head_command`, `/cmd_vel` |
| `frontier_explorer` | Frontier detection on `/map`, closest-first, Nav2 `navigate_to_pose` goals, blacklists unreachable goals | `ExploreFrontiers` action |
| `mission_server` | External missions: `explore`, `navigate`, `patrol`, `return_home` | `/mission/start` (`StartMission`), `/mission/stop` (`StopMission`); `/mission/command` |
| `web_dashboard` (`hexapod_perception`) | Flask on port 8080: camera, depth and map streams, battery, faces, mission control | `POST /api/mission/start`, `POST /api/mission/stop`, `GET /api/autonomy/state`, `GET /status` |

`face_recognition_node` (`hexapod_perception`) consumes the RealSense colour stream and is
launched separately by `perception.launch.py`; it is not part of the boot stack.

## Frames

`map → odom` (RTAB-Map) → `base_link` (controller odometry) → fixed `base_footprint`,
`imu_link`, `laser_frame`; revolute `head_pan_joint` → `head_pan_link` → `head_tilt_joint`
→ `head_tilt_link` → fixed `camera_link` → `camera_depth_frame`,
`camera_depth_optical_frame`, `camera_color_frame`, `camera_color_optical_frame`,
`camera_imu_optical_frame`. All from `hexapod_bringup/urdf/hexapod.urdf` via
`robot_state_publisher`, fed by the controller's `/joint_states`.

## Launch structure

```
robot.launch.py                 (systemd: autonomy:=true)
├── robot_state_publisher, imu_driver, imu_filter, battery_monitor, servo_driver,
│   led_controller, buzzer_controller, power_indicator, startup_sequence, controller
└── autonomy.launch.py          (autonomy:=true)
    ├── realsense_slam.launch.py   (slam:=true; localization if a saved map exists)
    ├── navigation.launch.py       (nav:=true)
    ├── slam_monitor, look_around, frontier_explorer, mission_server, autonomy_manager
    └── web_dashboard              (dashboard:=true)
```

`hardware.launch.py` is the drivers alone; `controller.launch.py` is the controller alone
with direct servo access (`test_ros.sh`); `perception.launch.py` adds face recognition.

## Topic contract

The hexapod speaks the family
[contract](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/common.md#the-topic-contract):
`/cmd_vel`, `/joint_commands`, `/joint_states`, `/imu/data`, `/odom` and `/tf`. It has no
`/wheel_odom` (gait odometry is published as `/odom`) and no `/telemetry` (battery is
`/battery/voltages`).
