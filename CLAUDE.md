# Project Context

## CRITICAL: This is a ROS 2 Port of Working Code

**Working reference code is in `../fn-hexapod/Code/Server/` (sibling repo).**

Key confirmed-working files:
- `servo.py` - PCA9685 servo control
- `home.py` - Calibrated home position
- `stand.py` - Smooth stand sequence
- `pca9685.py` - Low-level PWM driver
- `control.py` - Full gait control and IK

When implementing ANY hardware driver or feature:
1. **ALWAYS check `../fn-hexapod/Code/Server/` first** - the working code is there
2. Port the reference implementation to ROS 2 patterns
3. Do not reinvent - the Freenove code works on this exact hardware

## Physical Hardware

This runs on a **real, physical Freenove Big Hexapod Kit (FNK0052)** - not simulation.

- **Host**: `spid` (Raspberry Pi 5, 8GB)
- **OS**: Ubuntu Server 24.04
- **ROS**: ROS 2 Jazzy, installed natively from apt (no Docker)
- **Runtime**: `scripts/launch.sh`, run at boot by `systemd/hexapod.service`

## CRITICAL: Buzzer hazard

The shield buzzer (GPIO 17) is painfully loud and sounds whenever the pin is left
floating, i.e. after ANY process that claimed it exits or is killed. This has
distressed people in the house. Rules:

- Never claim GPIO 17 from ad-hoc scripts, and never kill a process holding it.
- `hexapod-buzzer-guard.service` holds the pin low at all times. Leave it running.
- `buzzer.enabled` in `hexapod_hardware/config/hardware.yaml` stays `false` unless Wayne asks.
- Do not read the pin with `gpioget` (it reconfigures the line as input).

### Power Modes

| Mode | Power Source | What Works |
|------|--------------|------------|
| **Battery** | 2x 18650 cells | Everything - servos, sensors, LEDs, camera |
| **USB** | USB-C power | Direct peripherals only - camera, IMU, ultrasonic, LEDs. **No servos** (PCA9685 powered by battery) |

## Hardware Components

- 20 servos (18 leg + 2 head pan/tilt) via 2x PCA9685 (I2C 0x40/0x41) - **battery mode only**
- Servo power enable on GPIO 4 (low = enabled)
- Intel RealSense D435i on the pan/tilt head (USB 3). Topics are `/camera/camera/...`
  because realsense-ros prefixes namespace and node name.
- MPU6050 IMU (I2C 0x68)
- ADS7830 ADC for battery monitoring (I2C 0x48)
- WS2812 LEDs (SPI)
- Buzzer on GPIO 17 (see hazard above)
- The original OV5647 camera and HC-SR04 ultrasonic are no longer fitted.

## Project Goals

- Autonomous exploration and mapping of the local area (RTAB-Map SLAM + Nav2)
- Missions from an approved external mission planner override exploration. Interim
  planner: the dashboard HTTP API on port 8080 (`scripts/mission.sh`), driven from
  Claude CLI sessions on another machine. Authentication is still to be defined.
- Wander mode with return-to-home capability

## Working on this machine

- ROS is native: `source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash`.
- DDS discovery takes about 10 s; `ros2 topic list` right after a launch shows nothing.
- `pkill -f <pattern>` kills the calling shell if the pattern appears in its own
  command line. Kill by PID or `pkill -x`.
- Ubuntu apt vs Raspberry Pi OS packages: never add the Pi (bookworm) repo on this host.

## Key Directories

- `../fn-hexapod/Code/Server/` - **Working Freenove Python code - start here**
- `ros2_ws/src/hexapod_hardware/` - ROS 2 hardware interface nodes
- `ros2_ws/src/hexapod_controller/` - IK, gait, odometry
- `ros2_ws/src/hexapod_autonomy/` - state machine, exploration, mission server
- `ros2_ws/src/hexapod_bringup/` - Launch files and config
- `ros2_ws/src/hexapod_perception/` - Face recognition, web dashboard
- `config/` - Hardware calibration
- `scripts/`, `systemd/` - setup, launch, service units
