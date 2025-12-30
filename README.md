# wk-hexapi

ROS 2 based autonomous hexapod robot, built on Freenove Big Hexapod hardware.

## Reference

This project is based on the [Freenove Big Hexapod Robot Kit for Raspberry Pi](https://github.com/Freenove/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi) (FNK0052).

The original repository contains:
- `Tutorial.pdf` - Comprehensive setup and usage guide
- `Calibration_Graph.pdf` - Servo calibration reference
- `About_Battery.pdf` - Battery specifications
- `Datasheet/` - Component datasheets (PCA9685, MPU6050, ADS7830)
- `Code/Server-pi5/` - Pi 5 specific server code

The `reference/` directory in this repo contains code copied from the Freenove repository for reference during ROS 2 development.

## Hardware

- Raspberry Pi 5 (8GB)
- 20 servos (18 leg + 2 head pan/tilt) via PCA9685
- OV5647 camera (on pan/tilt head)
- Ultrasonic sensor (on pan/tilt head)
- MPU6050 IMU
- ADS7830 ADC for dual battery monitoring
- WS2812 LEDs
- Buzzer

## Goals

- ROS 2 Jazzy on Ubuntu Server 24.04
- Autonomous mapping and navigation (SLAM + Nav2)
- Wander mode with return-to-home capability

## Project Structure

```
wk-hexapi/
├── ros2_ws/src/
│   ├── hexapod_hardware/     # Hardware interface nodes
│   │   ├── imu_driver        # MPU6050 IMU publisher
│   │   ├── servo_driver      # PCA9685 servo control
│   │   ├── battery_monitor   # ADS7830 battery status
│   │   └── led_controller    # WS2812 LED control
│   └── hexapod_bringup/      # Launch files and config
├── reference/                # Original Freenove code
├── config/                   # Hardware calibration
├── docker/                   # Docker support files
├── scripts/                  # Setup scripts
└── docs/                     # Documentation
```

## Quick Start (Docker - Recommended)

```bash
# Clone the repo
git clone <repo-url> ~/Code/wk-hexapi
cd ~/Code/wk-hexapi

# Run hardware setup first (if not already done)
sudo ./scripts/ubuntu-setup.sh --skip-reboot

# Build and start container
docker compose build
docker compose up -d

# Enter container
docker compose exec hexapod bash

# Inside container: build ROS 2 workspace
colcon build --symlink-install
source install/setup.bash

# Launch hardware drivers
ros2 launch hexapod_bringup hardware.launch.py
```

## Development Workflow

```bash
# Start development shell
docker compose --profile dev run --rm dev

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Test IMU
ros2 topic echo /imu/data_raw

# Test battery monitor
ros2 topic echo /battery/voltages
```

## Native Setup (Alternative)

```bash
# Run setup script
sudo ./scripts/ubuntu-setup.sh

# Activate venv
source .venv/bin/activate

# Install ROS 2 Jazzy (see docs.ros.org)
```

## License

Apache 2.0
