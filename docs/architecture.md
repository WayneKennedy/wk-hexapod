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
| `startup_sequence` | — | `/leds/zone`, `/buzzer/state`, `/pose_command`, `/robot/initialized`; `/robot/safe_startup` service |

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
| Out | `/joint_commands` (per gait sub-step; head slots NaN), `/joint_states` (50 Hz, legs only), `/odom` (20 Hz), TF `odom → base_link`, `/servo_relax` |
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

### Mapping — there is no SLAM

The D435i is gone (DEC-25), so there is no depth, no visual odometry and no loop closure.
What takes its place (DEC-28):

- **The map is Nav2's global costmap**, a fixed 12 × 12 m grid at 5 cm whose only source is
  `nav2_costmap_2d::RangeSensorLayer` fed from `/ultrasonic/range`. The layer holds a
  probability per cell and only calls a cell free or occupied once readings cross its
  thresholds, so unseen space stays unknown and the frontier explorer has something to aim
  at. It is published as `/global_costmap/costmap`.
- **`map` is odometry.** `map → odom` is a static identity published by
  `navigation.launch.py`. Gait odometry drift is therefore map drift, uncorrected, and a
  map is only meaningful within one run: nothing is saved and nothing is localized against
  ([OQ-20](open-questions.md)).
- **The camera feeds no part of navigation.** `camera_ros` publishes `/camera/image_raw`
  for the dashboard and face recognition only ([OQ-22](open-questions.md)).

### Navigation (`hexapod_bringup/launch/navigation.launch.py`)

Nav2's `navigation_launch.py` only — planner, controller (regulated pure pursuit),
smoother, behaviours, BT navigator, waypoint follower, velocity smoother, collision
monitor, docking server (no docks). Nav2's map server and AMCL are not started (DEC-06):
there is nothing to load or localize against.

Both costmaps take obstacles from the sonar through a `RangeSensorLayer`; the local costmap
is a 3 m rolling window and the global one is the map above. The robot radius is **0.24 m**,
the reach of the foot tips, not the 0.15 m body radius used until 2026-09-19. The collision
monitor watches `/ultrasonic/range` directly, with a stop polygon 0.32 m ahead and a
slowdown polygon at 0.50 m ([OQ-03](open-questions.md)).

**The behaviour trees are this repository's** (`hexapod_bringup/config/behavior_trees/`),
not Nav2's packaged defaults, for two reasons: the defaults clear the global costmap in
recovery, which here would erase the map, and their `Spin` recovery turns eighteen servos
to look around, which is the head's job. `Spin` is not loaded in `behavior_server` either.

### Autonomy (`hexapod_autonomy`)

| Node | Role | Interfaces |
|---|---|---|
| `autonomy_manager` | State machine: `waiting_for_startup → look_around → mapping_mode → exploring → exploration_complete / error`. The head survey is the first act of every run; `checking_map` and `localization_mode` are unreachable while there is no saved map | `/autonomy/state` (`AutonomyState`), `/robot/initialized` |
| `frontier_explorer` | Frontier detection on `/global_costmap/costmap`, closest-first, Nav2 `navigate_to_pose` goals, blacklists unreachable goals, head survey on arrival, ignores frontiers nearer than `min_goal_distance` | `ExploreFrontiers` action |
| `mission_server` | External missions: `explore`, `navigate`, `patrol`, `return_home` | `/mission/start` (`StartMission`), `/mission/stop` (`StopMission`); `/mission/command` |
| `web_dashboard` (`hexapod_perception`) | Flask on port 8080: camera, **sonar fan** and map streams, battery, faces, mission control | `POST /api/mission/start`, `POST /api/mission/stop`, `GET /api/autonomy/state`, `GET /status` |

`face_recognition_node` (`hexapod_perception`) consumes `/camera/image_raw` and is launched
separately by `perception.launch.py`; it is not part of the boot stack.

## Frames

`map → odom` (static identity) → `base_link` (controller odometry) → fixed
`base_footprint`, `imu_link`; revolute `head_pan_joint` → `head_pan_link` →
`head_tilt_joint` → `head_tilt_link` → fixed `camera_link` → `camera_optical_frame`, and
fixed `ultrasonic_link`. All from `hexapod_bringup/urdf/hexapod.urdf` via
`robot_state_publisher`, fed by the leg controller's `/joint_states` (legs) and
`head_controller`'s (head). The head offsets in the URDF are estimates, not measured.

## Launch structure

```
robot.launch.py                 (systemd: autonomy:=true)
├── robot_state_publisher, imu_driver, imu_filter, ultrasonic_driver, camera_ros
│   (camera:=true), battery_monitor, servo_driver, led_controller, buzzer_controller,
│   power_indicator, startup_sequence, controller, head_controller
└── autonomy.launch.py          (autonomy:=true)
    ├── navigation.launch.py       (nav:=true; Nav2 plus the static map → odom)
    ├── frontier_explorer, mission_server, autonomy_manager
    └── web_dashboard              (dashboard:=true)
```

`hardware.launch.py` is the drivers alone; `controller.launch.py` is the leg controller
alone with direct servo access (`test_ros.sh`); `perception.launch.py` adds face
recognition.

## Topic contract

The hexapod speaks the family
[contract](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/common.md#the-topic-contract):
`/cmd_vel`, `/joint_commands`, `/joint_states`, `/imu/data`, `/odom` and `/tf`. It has no
`/wheel_odom` (gait odometry is published as `/odom`) and no `/telemetry` (battery is
`/battery/voltages`). Two nodes publish `/joint_states`, legs and head, and
`robot_state_publisher` merges them by joint name.
