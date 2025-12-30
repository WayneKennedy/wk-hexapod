# Project Context

## CRITICAL: This is a ROS 2 Port of Working Code

**The `reference/` directory contains fully functional Python code from Freenove.**

When implementing ANY hardware driver or feature:
1. **ALWAYS check `reference/` first** - the working code is there
2. Port the reference implementation to ROS 2 patterns
3. Do not reinvent - the Freenove code works on this exact hardware

## Physical Hardware

This runs on a **real, physical Freenove Big Hexapod Kit (FNK0052)** - not simulation.

- **Host**: `spid` (Raspberry Pi 5, 8GB)
- **OS**: Ubuntu Server 24.04
- **ROS**: ROS 2 Jazzy
- **Runtime**: Docker container with hardware passthrough

### Power Modes

| Mode | Power Source | What Works |
|------|--------------|------------|
| **Battery** | 2x 18650 cells | Everything - servos, sensors, LEDs, camera |
| **USB** | USB-C power | Direct peripherals only - camera, IMU, ultrasonic, LEDs. **No servos** (PCA9685 powered by battery) |

## Hardware Components

- 20 servos (18 leg + 2 head pan/tilt) via PCA9685 - **battery mode only**
- OV5647 camera on pan/tilt head
- HC-SR04 ultrasonic sensor on pan/tilt head (GPIO 27/22)
- MPU6050 IMU (I2C 0x68)
- ADS7830 ADC for battery monitoring (I2C 0x48)
- WS2812 LEDs (SPI)
- Buzzer

## Project Goals

- Autonomous mapping and navigation (SLAM + Nav2)
- Wander mode with return-to-home capability

## Key Directories

- `reference/` - **Working Freenove Python code - start here**
- `ros2_ws/src/hexapod_hardware/` - ROS 2 hardware interface nodes
- `ros2_ws/src/hexapod_bringup/` - Launch files and config
- `ros2_ws/src/hexapod_perception/` - Camera and vision nodes
- `config/` - Hardware calibration
