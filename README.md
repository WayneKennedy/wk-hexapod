# wk-hexapod

ROS 2 based autonomous hexapod robot, built on Freenove Big Hexapod hardware.

## Reference

This project is based on the [Freenove Big Hexapod Robot Kit for Raspberry Pi](https://github.com/Freenove/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi) (FNK0052).

Working reference code is in `../fn-hexapod/Code/Server/`:
- `servo.py` - PCA9685 servo control (confirmed working)
- `home.py` - Calibrated home position (confirmed working)
- `stand.py` - Smooth stand sequence (confirmed working)
- `pca9685.py` - Low-level PWM driver
- `control.py` - Full gait control and IK

The Freenove repository also contains:
- `Tutorial.pdf` - Comprehensive setup and usage guide
- `Calibration_Graph.pdf` - Servo calibration reference
- `Datasheet/` - Component datasheets (PCA9685, MPU6050, ADS7830)

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

## Autonomy Roadmap

End state: Robot boots autonomously, accepts high-level mission requests ("walk forward 150mm", "go to kitchen"), and ROS handles planning and execution.

```
┌─────────────────┐     ┌─────────────┐     ┌────────────────┐     ┌──────────────┐
│ Mission Request │────▶│    Nav2     │────▶│  /cmd_vel      │────▶│  Controller  │
│ "go to kitchen" │     │ (planning)  │     │ (velocity)     │     │ (legs move)  │
└─────────────────┘     └─────────────┘     └────────────────┘     └──────────────┘
                              ▲                                           │
                              │              ┌────────────────┐           │
                              └──────────────│    /odom       │◀──────────┘
                                             │ (position)     │
                                             └────────────────┘
```

### Phase 1: Locomotion (DONE)
- [x] Hexapod controller with IK and tripod gait
- [x] cmd_vel subscriber for velocity commands
- [x] Home, stand, relax poses
- [x] Initialize service for safe startup

### Phase 2: Odometry (IN PROGRESS)
- [x] Integrate gait cycles to estimate displacement
- [x] Publish `/odom` (nav_msgs/Odometry) with position and velocity
- [x] Publish TF transform: `odom` → `base_link`
- [ ] Fuse with IMU for rotation accuracy
- [ ] Add `MoveDistance` action server for goal-based movement

### Phase 3: Perception
- [ ] Configure camera for depth estimation or visual odometry
- [ ] Ultrasonic sensor for close obstacle detection
- [ ] Publish sensor data for costmap integration

### Phase 4: SLAM
- [ ] Configure slam_toolbox or rtabmap
- [ ] Build occupancy grid from sensors
- [ ] Save/load maps for persistent navigation

### Phase 5: Navigation (Nav2)
- [ ] Configure Nav2 with hexapod-specific parameters
- [ ] Tune local/global planners for hexapod motion
- [ ] Add semantic waypoints ("kitchen", "charging station")
- [ ] Implement return-to-home behavior

### Phase 6: Autonomous Operation
- [ ] Auto-start on boot via systemd
- [ ] Mission queue for accepting external requests
- [ ] Battery-aware behavior (return to charge)
- [ ] Wander mode with exploration

## Locomotion Controller

The hexapod uses body-centric control where foot positions are defined relative to the body origin, and inverse kinematics calculates all servo angles.

**Control model:**
- `body_position[x, y, z]` - Body translation (Z negative = raise body)
- `body_rotation[roll, pitch, yaw]` - Body orientation
- Feet maintain fixed positions in world frame while body moves

**Gaits:**
- Tripod gait: Even legs (0,2,4) and odd legs (1,3,5) alternate
- World Y axis is forward/back, X axis is left/right (strafe)

**ROS 2 Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands for walking
- `/pose_command` (std_msgs/String) - Pose commands: "home", "stand", "relax"
- `/odom` (nav_msgs/Odometry) - Odometry estimate from gait integration
- `/tf` - Transform: `odom` → `base_link`

**ROS 2 Services:**
- `/hexapod/initialize` (std_srvs/Trigger) - Home then stand sequence
- `/hexapod/reset_odometry` (std_srvs/Trigger) - Reset odometry to origin

## Project Structure

```
wk-hexapod/
├── ros2_ws/src/
│   ├── hexapod_controller/   # Main locomotion controller
│   │   ├── controller.py     # ROS 2 node with IK, gait, balance
│   │   ├── test_init.py      # Standalone home/stand test
│   │   ├── test_walk.py      # Standalone walk test
│   │   └── test_ros.sh       # ROS-based integration test
│   ├── hexapod_hardware/     # Hardware interface nodes
│   │   ├── imu_driver        # MPU6050 IMU publisher
│   │   ├── servo_driver      # PCA9685 servo control
│   │   ├── range_finder      # HC-SR04 ultrasonic sensor
│   │   ├── battery_monitor   # ADS7830 battery status
│   │   └── led_controller    # WS2812 LED control
│   ├── hexapod_perception/   # Vision and perception
│   │   ├── camera_node       # Pi Camera publisher
│   │   └── face_detector     # Face recognition node
│   └── hexapod_bringup/      # Launch files and config
├── config/                   # Hardware calibration
├── docker/                   # Docker support files
├── scripts/                  # Setup scripts
└── docs/                     # Documentation

../fn-hexapod/                # Sibling repo with working Freenove code
```

## Quick Start (Docker - Recommended)

```bash
# Clone the repo
git clone <repo-url> ~/Code/wk-hexapod
cd ~/Code/wk-hexapod

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

## Testing

**Standalone hardware tests** (no ROS required):
```bash
# Test home and stand (battery power required)
python3 ros2_ws/src/hexapod_controller/test_init.py

# Test walking forward/backward
python3 ros2_ws/src/hexapod_controller/test_walk.py
```

**ROS integration test** (inside Docker):
```bash
docker compose run --rm dev ./ros2_ws/src/hexapod_controller/test_ros.sh
```

This builds the workspace, starts the controller node, and runs through:
1. Initialize (home -> stand)
2. Walk forward 150mm
3. Walk backward 150mm
4. Home and relax

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
