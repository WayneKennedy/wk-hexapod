# Operations

Install, run, save maps, send missions, test, and fix the things that go wrong. Written
for the robot's own Pi 5 on Ubuntu Server 24.04.

## Install

```bash
git clone git@github.com:WayneKennedy/wk-hexapod.git ~/Code/wk-hexapod
# Vendor reference: sparse clone of Freenove's upstream, Code/Server only (~70 KB, not 477 MB).
# Pinned commit and drift check: docs/references.md.
git clone --filter=blob:none --no-checkout --depth 1 \
  https://github.com/Freenove/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi.git ~/Code/freenove-hexapod
git -C ~/Code/freenove-hexapod sparse-checkout set Code/Server
git -C ~/Code/freenove-hexapod checkout master
cd ~/Code/wk-hexapod
sudo ./scripts/ubuntu-setup.sh        # idempotent; --dry-run previews
sudo reboot                           # config.txt and group changes
./systemd/install.sh                  # buzzer guard + hexapod.service, enabled at boot
```

The setup script: enables I2C at 400 kHz and SPI, writes the safe GPIO defaults
([`hardware.md`](hardware.md#the-buzzer-hazard)), adds the ROS 2 apt source, installs
ROS 2 Jazzy plus Nav2, RTAB-Map, `realsense2_camera`, `imu_filter_madgwick`,
`depthimage_to_laserscan` and the Python hardware libraries from apt, installs the
librealsense udev rules, adds the user to the hardware groups, pip-installs
`requirements.txt` (only `face_recognition`/dlib builds from source, about 20 minutes),
initialises rosdep, and runs `colcon build --symlink-install`.

Python dependencies go into the system interpreter (`--break-system-packages`) because
ROS 2 nodes run under `/usr/bin/python3`. There is no virtualenv.

### Foreign packages

If Raspberry Pi OS (bookworm) apt sources were ever enabled on the host, packages such as
`libswresample4`, `libwayland-*`, `libssl3`, `libcamera*` and `linux-libc-dev` may be the
`+rpt`/`deb12` builds. They block ROS packages with errors like:

```
libswresample-dev : Depends: libswresample4 (= 7:6.1.1-3ubuntu5) but 8:5.1.8-0+deb12u1+rpt1 is to be installed
```

Find them with `dpkg-query -W -f='${Package}\t${Version}\n' | grep -E 'rpt|deb12'`,
disable the Pi source, downgrade each to its noble version
(`apt-get install --allow-downgrades pkg=<version from apt-cache madison>`), and remove
bookworm-only packages (`libavutil57`, `libssl3`, `libcamera*`, `rpicam-apps*`).
`initramfs-tools` from the Pi repo does not conflict and can stay. This was done on the
robot on 2026-09-09 ([`test-log.md`](test-log.md)).

## Run

```bash
sudo systemctl start|stop|status hexapod      # the boot stack
journalctl -u hexapod -f
scripts/launch.sh                              # same stack by hand (stop the service first)
scripts/launch.sh autonomy:=false              # drivers + controller only
scripts/launch.sh hardware.launch.py           # any hexapod_bringup launch file
```

`scripts/launch.sh` sources ROS and the workspace, sets `ROS_DOMAIN_ID=0`, Fast DDS and
the `lgpio` pin factory, and execs `ros2 launch hexapod_bringup <file>`, adding
`autonomy:=true` for `robot.launch.py` unless told otherwise.

**What the boot stack does** ([`architecture.md`](architecture.md#launch-structure)):
drivers start; the startup sequence shows red on the rear LED for 2 s, snaps the legs to
home, shows cyan for 10 s so the robot can be placed on the floor, then stands and shows
green. RealSense, RTAB-Map, Nav2 and the autonomy nodes come up alongside. With no saved
map the state machine goes straight to mapping and frontier exploration. With a saved map
it localizes, then waits `mission_timeout` (60 s) for a mission before exploring.

**On the battery this means the robot stands and walks off on its own about 30 s after
boot.** Start with `autonomy:=false`, or stop the service, when that is not wanted.

Services: `hexapod-buzzer-guard.service` (holds GPIO 17 low; leave it enabled) and
`hexapod.service` (runs `scripts/launch.sh autonomy:=true` as the owning user, stops the
nodes with SIGINT so servos relax, and leaves servo power disabled afterwards). It starts
only after `time-sync.target`: `systemd-time-wait-sync.service` is enabled with a 90 s
bound, so a boot with no network starts the stack 90 s late on the restored clock
(DEC-21).

## Maps

RTAB-Map works in `~/.ros/rtabmap.db`, wiped at the start of every mapping run. A map is
only used for localization once deliberately saved (DEC-11):

```bash
sudo systemctl stop hexapod      # RTAB-Map flushes the database on shutdown
scripts/save-map.sh              # copies it to ~/.hexapod/maps/rtabmap.db, keeps a .bak
sudo systemctl start hexapod     # now boots in localization mode
```

Delete `~/.hexapod/maps/rtabmap.db` to map from scratch. If localization fails after the
look-around sweep, the autonomy manager switches RTAB-Map back to mapping in place.

## Remote missions

The dashboard on port 8080 is the interim planner interface (DEC-14). Any machine with
`curl` on the same network can drive it; the repository copy of `scripts/mission.sh` works
from anywhere:

```bash
export HEXAPOD_HOST=<robot address>           # or pass -H <host[:port]> each time
scripts/mission.sh state                       # autonomy state
scripts/mission.sh start explore 600           # explore for up to 600 s
scripts/mission.sh start return_home
scripts/mission.sh stop --return-home
scripts/mission.sh status                      # battery, faces, mission, full status
```

Endpoints: `POST /api/mission/start` `{mission_type, timeout_sec}`,
`POST /api/mission/stop` `{return_home}`, `GET /api/autonomy/state`, `GET /status`.
Mission types: `explore`, `navigate`, `patrol`, `return_home`. Clients with ROS 2 on the
same subnet can call the `/mission/start` and `/mission/stop` services directly with
`ROS_DOMAIN_ID=0`. **There is no authentication** ([OQ-04](open-questions.md)).

## Tests

Battery required for anything that moves. Stop the service first.

```bash
sudo systemctl stop hexapod
python3 ros2_ws/src/hexapod_controller/test_init.py    # home then stand, standalone Python
python3 ros2_ws/src/hexapod_controller/test_walk.py    # walk forward and back, standalone
ros2_ws/src/hexapod_controller/test_ros.sh             # controller alone with direct servo access: init, walk, home, relax
```

### Movement calibration

Battery, a floor with 1.5 m clear, owner present, service stopped. `/cmd_vel` is SI
(DEC-22); odometry integrates what the gait commands, so the actual/commanded ratio is
measured against `/odom`, not against a cycle count.

```bash
sudo systemctl stop hexapod
scripts/launch.sh autonomy:=false &      # homes, then stands about 15 s later
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
ODOM='ros2 topic echo /odom --once --field pose.pose'
# 1. Yaw sign (OQ-13): lift the robot, turn it 90° anticlockwise, set it down.
#    /imu/data orientation.z must be about +0.7 (odometry only takes the IMU
#    yaw while walking). Negative means the IMU yaw is inverted.
ros2 topic echo /imu/data --once --field orientation
# Odometry yaw follows the IMU while walking, so for the two runs below turn
# the fusion off: odom then reports exactly what the gait commanded.
ros2 param set /hexapod_controller odometry.imu_fusion false
# 2. Forward 1 m: mark a foot tip, then
$ODOM; timeout 20 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"; $ODOM
#    stride_scale = tape distance / odom distance. The robot must go forward.
# 3. Turn: mark the heading, then
$ODOM; timeout 10 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}"; $ODOM
#    turn_scale = measured angle / odom yaw change (the IMU yaw change is a
#    fair measure of the angle). Positive z must turn left.
ros2 param set /hexapod_controller odometry.imu_fusion true
```

Put the two factors in `hexapod_controller/config/body_params.yaml`, record the run in
[`test-log.md`](test-log.md), restart the service. A wrong direction in step 2 or 3 is a
sign error in the controller's `_walk_cycle`, not a calibration value.

Sensor checks that need no battery:

```bash
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
ros2 topic echo /imu/data --once
ros2 topic echo /battery/voltages --once
ros2 topic hz /scan
curl -s localhost:8080/api/autonomy/state
```

## Development loop

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
```

Python nodes are symlink-installed: restart the service after editing. Rebuild after
changing `hexapod_interfaces`, any `setup.py`, launch files, or config files (they are
`data_files`). Record what you ran in [`test-log.md`](test-log.md).

## Troubleshooting

- **`ros2 topic list` is empty right after launch.** DDS discovery takes about 10 s here.
  Use `ros2 daemon start` and query again; `--no-daemon` gives partial lists.
- **Nav2 "Failed to make progress".** Expected on USB power (servos dead). On battery,
  run the movement calibration above; before DEC-22 this was the velocity mismatch.
- **Collision monitor flaps "stop / continue".** Transform lag between `/scan` and
  `odom → base_link`; fixed by DEC-09 on 2026-09-09. If it recurs, check load
  ([OQ-02](open-questions.md)).
- **RTAB-Map "TF of received image ... not set".** A few at start-up are normal. If they
  persist, `robot_state_publisher` is not getting head joint states from the controller.
- **The buzzer sounds.** Something released GPIO 17. `systemctl status hexapod-buzzer-guard`
  and [`hardware.md`](hardware.md#the-buzzer-hazard).
- **A tool loops or hangs under load.** `ros2 topic hz` has been seen to hang at load
  averages above 8; read the journal instead.
