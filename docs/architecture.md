# Architecture

Every bus, node, topic and frame on the robot, and where it sits in the family's tier
model. Package sources are under `ros2_ws/src/`.

## Where it sits in the family

The family's [two-tier split](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/common.md#compute-the-two-tier-split)
puts deterministic loops on a microcontroller and everything else on a Pi. **The hexapod
has only the second tier.** Every device is a direct peripheral of the Pi 5; the only work
done outside the Pi's CPU is PWM pulse generation inside the PCA9685 chips: since the
RealSense left (DEC-25) nothing on the robot computes anything on the Pi's behalf.

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
| HC-SR04 ultrasonic (head) | GPIO 27 trigger, GPIO 22 echo | `ultrasonic_driver` | 15 Hz |
| OV5647 camera (head) | CSI CAM0, libcamera | `camera_ros` (`/camera/image_raw`) | on demand |
| RPLIDAR C1 (plate over the Pi stack, DEC-30) | USB serial, `/dev/ttyUSB0`, 460800 | `sllidar_node` (`/scan`, frame `laser_frame`) | 10 Hz |

## Nodes and topics

### Hardware (`hexapod_hardware`)

| Node | Subscribes | Publishes / serves |
|---|---|---|
| `servo_driver` | `/joint_commands` (20 angles: 6 legs × coxa/femur/tibia, pan, tilt; **pan and tilt are NaN**, meaning "not mine"), `/leg_positions`, `/leg_command`, `/head_command`, `/servo_relax`, `/pose_command` (relax only) | `servo_driver/initialize` service |
| `ultrasonic_driver` | — | `/ultrasonic/range` (`sensor_msgs/Range`, frame `ultrasonic_link`, 0.03–2 m). Echo timed from kernel edge timestamps; no echo within range is published as `max_range` |
| `imu_driver` | — | `/imu/data_raw` (`sensor_msgs/Imu`, no orientation) |
| `imu_filter` (`imu_filter_madgwick`, in `hexapod_bringup` launch) | `/imu/data_raw` | `/imu/data` with orientation quaternion, no TF |
| `battery_monitor` | — | `/battery/voltages` (LOAD and CTRL rails), diagnostics |
| `led_controller` | `/leds/zone` (`zone:r,g,b`; zones r1–r3, l1–l3, rear, left, right, front, back, mid, all) | — |
| `buzzer_controller` | `/buzzer/state` | `buzzer/beep` service. **Disabled by default** (DEC-10): requests are logged and dropped |
| `power_indicator` | `/battery/voltages` | `/leds/zone` (left = LOAD rail, right = CTRL rail; blue below 0.5 V means USB) |
| `startup_sequence` | `/hexapod/initialized` | `/leds/zone`, `/buzzer/state`, `/pose_command`, `/robot/initialized`; `/robot/safe_startup` service. Waits for `hexapod_controller` and `servo_driver` to subscribe to `/pose_command` before it starts, and for the controller to confirm `home`; red and no `/robot/initialized` otherwise ([OQ-28](open-questions.md)) |

### Looking (`hexapod_controller/head_controller`)

**The head is the robot's only steerable sense.** Both the camera and the ultrasonic sit
on the pan/tilt head, so where the robot can see is a head-servo decision. `head_controller`
is the single owner of those two servos (DEC-27): the leg controller sends NaN in the head
slots of `/joint_commands`, and nothing else publishes `/head_command`.

| Direction | Topic / interface |
|---|---|
| In | `/lookahead_point` (Nav2 pure-pursuit carrot), `/cmd_vel`, `/head/look_at` (`PointStamped`, any frame), `/pose_command`, `/servo_relax`, `/ultrasonic/range` |
| Out | `/head_command` (servo degrees), `/joint_states` (`head_pan_joint`, `head_tilt_joint`) |
| Action | `/look_around` (`LookAround`): full-width pan sweeps with the body still |

**The head is on** (owner, 2026-10-02; off 2026-09-29 to 10-02, [OQ-30](open-questions.md)):
`enabled` in `body_params.yaml` and `servos.head_enabled` in `hardware.yaml`, which must
agree. With either false nothing is published on `/head_command`, `/look_around` goals are
rejected, the head joints are published at zero, and `servo_driver` never writes the two
channels. Its range is limited in both nodes ([DEC-35](decisions.md)).

Behaviours, highest priority first: the **survey** (the `LookAround` action), a **look_at**
gaze held for a few seconds, and otherwise a continuous **scan** — the pan sweeps a ±30°
sector in 10° steps, centred on Nav2's carrot while it is fresh, else on the direction of
a turn, else straight ahead. So the head is already pointing where the body is about to go.

Head joint states follow a slew-rate **model** of the servo, not the command, because the
servos have no feedback. The model is what puts `ultrasonic_link` on the TF tree, so a
wrong `slew_rate` mis-places sonar readings ([OQ-21](open-questions.md)).

### Locomotion (`hexapod_controller`)

`hexapod_controller` owns body-centric IK: foot positions are held in the world frame while
the body translates and rotates, and every servo angle is solved from leg geometry (coxa 33,
femur 90, tibia 110 mm) plus per-leg calibration offsets. It **does not touch the servo
hardware** in the robot stack (DEC-09); it publishes calibrated angles and `servo_driver`
writes them.

| Direction | Topic / interface |
|---|---|
| In | `/cmd_vel` (`Twist`), `/pose_command` (`home`, `stand`, `relax`), `/imu/data`, body pose command |
| Out | `/joint_commands` (per gait sub-step; head slots NaN), `/joint_states` (50 Hz, legs only), `/odom` (20 Hz), TF `odom → base_link`, `/servo_relax`, `/hexapod/initialized` (`Bool`, 1 Hz and on change; true once `home` has run) |
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

### Mapping — lidar SLAM in the boot stack

Since 2026-09-29 ([DEC-32](decisions.md), superseding DEC-28):

- **`slam_toolbox` (online async) builds `/map` from `/scan` and gait odometry** and
  publishes `map → odom`. `navigation.launch.py` includes `slam.launch.py`; the parameters
  are `config/slam_params.yaml`, passed as `slam_params_file` (not `params_file`, which is
  Nav2's and is shared between the two launch files).
- **The map starts when the robot stands.** `slam_toolbox` takes its first scan at start,
  before the robot has been placed and has stood, so `autonomy_manager` calls
  `/slam_toolbox/reset` and clears both costmaps before it leaves `waiting_for_startup`.
- **The frontier explorer and the dashboard read `/global_costmap/costmap`**, which is
  `/map` plus live obstacles and inflation. Unknown cells stay unknown.
- **Nothing is saved and nothing is localized against** ([OQ-20](open-questions.md)).
- **The sonar feeds nothing**: `/ultrasonic/range` is published and on TF, but no costmap
  or collision-monitor source reads it (DEC-32); its use low and ahead is open (OQ-26).
- **The camera feeds no part of navigation.** `camera_ros` publishes `/camera/image_raw`
  for the dashboard and face recognition only ([OQ-22](open-questions.md)).

### Navigation (`hexapod_bringup/launch/navigation.launch.py`)

`slam.launch.py` and Nav2's `navigation_launch.py` — planner (Smac 2D), controller
(regulated pure pursuit), smoother, behaviours, BT navigator, waypoint follower, velocity
smoother, collision monitor, docking server (no docks). Nav2's map server and AMCL are not
started (DEC-06): there is nothing to load or localize against.

Both costmaps take obstacles from `/scan` through an `ObstacleLayer` that drops returns
nearer than 0.25 m and marks out to 3 m; the global one adds a `StaticLayer` on `/map`. The
local costmap is a 3 m rolling window. The robot radius is **0.24 m**, the reach of the
foot tips. The collision monitor watches `/scan`, with a stop polygon 0.32 m ahead and a
slowdown polygon at 0.50 m ([OQ-03](open-questions.md), [OQ-31](open-questions.md)).

**The behaviour trees are this repository's** (`hexapod_bringup/config/behavior_trees/`,
the `_sonar` files, unchanged by DEC-32): only the local costmap is cleared in recovery and
there is no `Spin`. `Spin` is not loaded in `behavior_server` either.

### Autonomy (`hexapod_autonomy`)

| Node | Role | Interfaces |
|---|---|---|
| `autonomy_manager` | State machine: `waiting_for_startup → look_around → mapping_mode → exploring → exploration_complete / error`. The head survey is the first act of every run; `checking_map` and `localization_mode` are unreachable while there is no saved map | `/autonomy/state` (`AutonomyState`), `/robot/initialized` |
| `frontier_explorer` | Frontier detection on `/global_costmap/costmap`, closest-first, Nav2 `navigate_to_pose` goals, blacklists unreachable goals, head survey on arrival, ignores frontiers nearer than `min_goal_distance` | `ExploreFrontiers` action |
| `mission_server` | External missions: `explore`, `navigate`, `patrol`, `return_home` | `/mission/start` (`StartMission`), `/mission/stop` (`StopMission`); `/mission/command` |
| `web_dashboard` (`hexapod_perception`) | Flask on port 8080: camera, **lidar view** (the current scan, body frame) and map streams (what each panel can be trusted for: [OQ-33](open-questions.md)), battery, faces, mission control | `POST /api/mission/start`, `POST /api/mission/stop`, `GET /api/autonomy/state`, `GET /status` |

`face_recognition_node` (`hexapod_perception`) consumes `/camera/image_raw` and is launched
separately by `perception.launch.py`; it is not part of the boot stack.

## Frames

`map → odom` (`slam_toolbox`) → `base_link` (controller odometry) → fixed `imu_link`;
`base_footprint` is **not** on this tree: the URDF makes it `base_link`'s parent, so it is a
separate root ([OQ-27](open-questions.md)); revolute `head_pan_joint` → `head_pan_link` →
`head_tilt_joint` → `head_tilt_link` → fixed `camera_link` → `camera_optical_frame`, and
fixed `ultrasonic_link`; fixed `laser_frame` off `base_link` (x +0.010 m, yaw π, z 0.16 m). All from `hexapod_bringup/urdf/hexapod.urdf` via
`robot_state_publisher`, fed by the leg controller's `/joint_states` (legs) and
`head_controller`'s (head). The head offsets in the URDF are estimates, not measured.

## Launch structure

```
robot.launch.py                 (systemd: autonomy:=true)
├── robot_state_publisher, imu_driver, imu_filter, ultrasonic_driver, camera_ros
│   (camera:=true), sllidar_node (lidar:=true), battery_monitor, servo_driver, led_controller, buzzer_controller,
│   power_indicator, startup_sequence, controller, head_controller
└── autonomy.launch.py          (autonomy:=true)
    ├── navigation.launch.py       (nav:=true; Nav2 plus the static map → odom)
    ├── frontier_explorer, mission_server, autonomy_manager
    └── web_dashboard              (dashboard:=true)
```

`hardware.launch.py` is the drivers alone; `controller.launch.py` is the leg controller
alone with direct servo access (`test_ros.sh`); `perception.launch.py` adds face
recognition; `slam.launch.py` is `slam_toolbox` alone, which `navigation.launch.py`
includes.

## Topic contract

The hexapod speaks the family
[contract](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/common.md#the-topic-contract):
`/cmd_vel`, `/joint_commands`, `/joint_states`, `/imu/data`, `/odom` and `/tf`. It has no
`/wheel_odom` (gait odometry is published as `/odom`) and no `/telemetry` (battery is
`/battery/voltages`). Two nodes publish `/joint_states`, legs and head, and
`robot_state_publisher` merges them by joint name.
