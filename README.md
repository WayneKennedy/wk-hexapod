# wk-hexapod

ROS 2 based autonomous hexapod robot, built on Freenove Big Hexapod hardware and
running natively on Ubuntu Server 24.04 / ROS 2 Jazzy on a Raspberry Pi 5.

## Reference

This project is based on the [Freenove Big Hexapod Robot Kit for Raspberry Pi](https://github.com/Freenove/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi) (FNK0052).

Working reference code is in `../fn-hexapod/Code/Server/`:
- `servo.py` - PCA9685 servo control (confirmed working)
- `home.py` - Calibrated home position (confirmed working)
- `stand.py` - Smooth stand sequence (confirmed working)
- `pca9685.py` - Low-level PWM driver
- `control.py` - Full gait control and IK

## Hardware

- Raspberry Pi 5 (8GB), host name `spid`
- Freenove shield: 2x PCA9685 (20 servos: 18 leg + 2 head pan/tilt), ADS7830 battery ADC,
  MPU6050 IMU, WS2812 LED strip (SPI), buzzer (GPIO 17), servo power enable (GPIO 4)
- Intel RealSense D435i on the pan/tilt head (RGB-D + IMU). Replaces the original
  Pi Camera and ultrasonic sensor.

### Power modes

| Mode | Power | What works |
|------|-------|------------|
| Battery | 2x 18650 | Everything |
| USB | USB-C | Sensors, LEDs, RealSense. No servos (PCA9685 is battery powered) |

### Buzzer warning

The shield buzzer is extremely loud, and it sounds whenever GPIO 17 is left
floating, which happens after any process that claimed the pin exits. Two
safeguards keep it silent:

1. `/boot/firmware/config.txt` drives GPIO 17 low from firmware boot (`gpio=17=op,dl`).
2. `systemd/hexapod-buzzer-guard.service` holds GPIO 17 low at all times, independently
   of the ROS stack.

`buzzer.enabled` in `hexapod_hardware/config/hardware.yaml` is `false` by default, so beep
requests are logged and dropped. To use the buzzer, set it to `true` and stop the guard
service. Do not run the buzzer node against the pin while the guard is active.

## Goals

- Autonomous exploration and mapping of the local area (RTAB-Map SLAM + Nav2)
- Missions supplied by an approved external mission planner take priority over
  autonomous exploration. Today that is an HTTP API on the robot (see Remote missions);
  authentication and a formal planner interface are still to be defined.
- Wander mode with return-to-home capability

## Autonomy roadmap

End state: the robot boots, accepts high-level mission requests ("explore", "go to
kitchen"), and ROS handles planning and execution.

```
┌─────────────────┐     ┌─────────────┐     ┌────────────────┐     ┌──────────────┐
│ Mission Request │────▶│    Nav2     │────▶│  /cmd_vel      │────▶│  Controller  │
│ "go to kitchen" │     │ (planning)  │     │ (velocity)     │     │ (legs move)  │
└─────────────────┘     └─────────────┘     └────────────────┘     └──────────────┘
                              ▲                                           │
                              │              ┌────────────────┐           │
                              └──────────────│    /odom       │◀──────────┘
                                             └────────────────┘
```

- Phase 1 Locomotion: DONE. IK, tripod gait, cmd_vel, home/stand/relax, initialize service.
- Phase 2 Odometry: DONE. Gait integration, IMU fusion, `/odom` and TF, MoveDistance action.
- Phase 3 Perception: DONE. RealSense D435i, depth to LaserScan, camera frames in URDF.
  Face recognition subscribes to the RealSense colour stream.
- Phase 4 SLAM: IN PROGRESS. RTAB-Map configured for RGB-D. Needs tuning on the robot and
  map save/load for persistent navigation.
- Phase 5 Navigation: IN PROGRESS. Nav2 params tuned for slow motion. Planner tuning,
  semantic waypoints and return-to-home still open.
- Phase 6 Autonomous operation: IN PROGRESS. `hexapod_autonomy` provides the state
  machine (startup, localize, explore), frontier exploration, look-around, mission
  server and web dashboard. Battery-aware return and the approved planner interface
  are open.

## Setup (native)

```bash
git clone <repo-url> ~/Code/wk-hexapod
cd ~/Code/wk-hexapod
sudo ./scripts/ubuntu-setup.sh     # ROS 2 Jazzy, Nav2, RTAB-Map, RealSense, Python deps, build
sudo reboot                        # if config.txt or groups changed
./systemd/install.sh               # buzzer guard + auto-start on boot
```

The setup script installs everything from Ubuntu and ROS apt repositories. Only
`face_recognition` (dlib) is built from source, about 20 minutes on a Pi 5.

If the host previously had Raspberry Pi OS (bookworm) packages installed, they block the
ROS packages. See `docs/ubuntu-hardware-setup.md`, "Foreign packages".

## Running

```bash
scripts/launch.sh                       # full stack, autonomy on (what the service runs)
scripts/launch.sh autonomy:=false       # hardware drivers + controller only
scripts/launch.sh hardware.launch.py    # any hexapod_bringup launch file

sudo systemctl start|stop|status hexapod
journalctl -u hexapod -f
```

Boot sequence with `autonomy:=true`: hardware drivers, startup sequence (LED warning,
home, stand), RealSense + RTAB-Map, autonomy nodes, and the web dashboard on port 8080.
Without a saved map the robot waits `mission_timeout` seconds for an external mission,
then explores.

## Remote missions

The web dashboard (`hexapod_perception/web_dashboard`) serves a mission API on port 8080.
It is reachable on the LAN and over Tailscale, and needs nothing but curl on the
client, so a Claude CLI session on another machine can drive the robot:

```bash
scripts/mission.sh -H spid state                  # autonomy state
scripts/mission.sh -H spid start explore 600      # explore for up to 600 s
scripts/mission.sh -H spid stop --return-home
scripts/mission.sh -H spid status                 # battery, faces, mission
export HEXAPOD_HOST=spid                          # default host for the script
```

Underlying endpoints: `POST /api/mission/start` `{mission_type, timeout_sec}`,
`POST /api/mission/stop` `{return_home}`, `GET /api/autonomy/state`, `GET /status`.
Mission types: `explore`, `patrol`, `navigate`, `return_home`.

Clients with ROS 2 installed can instead call the services directly
(`/mission/start`, `/mission/stop`) with `ROS_DOMAIN_ID=0` on the same subnet.

There is no authentication yet. Treat the API as trusted-LAN only until the approved
planner interface is defined.

## Locomotion controller

Body-centric control: foot positions are defined relative to the body origin and
inverse kinematics calculates all servo angles.

- `body_position[x, y, z]`, `body_rotation[roll, pitch, yaw]`
- Tripod gait: legs (0,2,4) and (1,3,5) alternate. World Y is forward, X is strafe.

Servo ownership: the controller computes calibrated joint angles and publishes them on
`/joint_commands`; `hexapod_hardware/servo_driver` owns the PCA9685 chips and the servo
power GPIO. Set `hardware.direct:=true` on the controller only when running it standalone
without the servo driver (what `test_ros.sh` does).

Calibration is read from, in order: the `hardware.calibration_file` parameter,
`~/.hexapod/servo_calibration.txt`, then the copy installed with `hexapod_hardware`.

**Topics:** `/cmd_vel`, `/pose_command` (home, stand, relax), `/odom`, `/tf`, `/joint_commands`.
**Services:** `/hexapod/initialize`, `/hexapod/reset_odometry`.
**Actions:** `/hexapod/move_distance`.

## Project structure

```
wk-hexapod/
├── ros2_ws/src/
│   ├── hexapod_hardware/     # servo, IMU, battery, LED, buzzer, power indicator, startup sequence
│   ├── hexapod_controller/   # IK, gait, odometry, MoveDistance action; standalone tests
│   ├── hexapod_perception/   # face recognition, web dashboard (legacy Pi Camera node)
│   ├── hexapod_autonomy/     # state machine, frontier explorer, look-around, mission server
│   ├── hexapod_interfaces/   # custom msg/srv/action definitions
│   └── hexapod_bringup/      # launch files, RealSense/RTAB-Map/Nav2 config, URDF
├── config/                   # servo calibration
├── scripts/                  # ubuntu-setup.sh, launch.sh, mission.sh
├── systemd/                  # hexapod.service, hexapod-buzzer-guard.service, install.sh
└── docs/
```

## Testing

Standalone hardware tests (battery required, stop the service first):

```bash
sudo systemctl stop hexapod
python3 ros2_ws/src/hexapod_controller/test_init.py    # home and stand
python3 ros2_ws/src/hexapod_controller/test_walk.py    # walk forward/backward
ros2_ws/src/hexapod_controller/test_ros.sh             # ROS controller: init, walk, home, relax
```

Development loop:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 topic echo /imu/data_raw
ros2 topic echo /battery/voltages
```

Python nodes are symlink-installed, so edits take effect on restart without rebuilding.
Rebuild after changing `hexapod_interfaces` or any `setup.py`.

## License

Apache 2.0
