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
([`hardware.md`](hardware.md#the-buzzer-hazard)) and the OV5647 camera overlay, adds the
ROS 2 apt source, installs ROS 2 Jazzy plus Nav2, `camera_ros` (which brings libcamera
with the Pi 5 PiSP pipeline), `imu_filter_madgwick` and the Python hardware libraries
from apt, reports whether the camera probed, adds the user to the hardware groups,
pip-installs `requirements.txt` (only `face_recognition`/dlib builds from source, about
20 minutes), initialises rosdep, and runs `colcon build --symlink-install`.

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
scripts/launch.sh autonomy:=true               # the boot stack by hand (stop the service first)
scripts/launch.sh                              # drivers + controllers only
scripts/launch.sh autonomy:=true camera:=false # skip the camera (it does not probe: OQ-23)
scripts/launch.sh hardware.launch.py           # any hexapod_bringup launch file
```

`scripts/launch.sh` sets `ROS_DOMAIN_ID=0`, Fast DDS, discovery range `SUBNET` and the
`lgpio` pin factory, *then* sources ROS and the workspace, and execs `ros2 launch
hexapod_bringup <file>`. It adds no arguments. Since 2026-09-21 a manual run is drivers and
controllers only; only `hexapod.service` passes `autonomy:=true`.

**What the boot stack does** ([`architecture.md`](architecture.md#launch-structure)):
drivers start; the startup sequence shows red on the rear LED for 2 s, snaps the legs to
home, shows cyan for 10 s so the robot can be placed on the floor, then stands and shows
green. Nav2 and the autonomy nodes come up alongside. The head then makes a two-sweep
survey standing still (`look_around`), which seeds the map, and the state machine goes to
mapping and frontier exploration. After an external mission ends it waits
`mission_timeout` (60 s) for another before exploring again.

**On the battery this means the robot stands and walks off on its own about 30 s after
boot.** Stop the service, or `sudo systemctl disable hexapod` before a reboot, when that is
not wanted.

Services: `hexapod-buzzer-guard.service` (holds GPIO 17 low; leave it enabled) and
`hexapod.service` (runs `scripts/launch.sh autonomy:=true` as the owning user, stops the
nodes with SIGINT to `ros2 launch` alone (`KillMode=mixed`) so servos relax, and leaves
servo power disabled afterwards). Both follow
[*Robot startup is familial*](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/common.md#robot-startup-is-familial). It starts
only after `time-sync.target`: `systemd-time-wait-sync.service` is enabled with a 90 s
bound, so a boot with no network starts the stack 90 s late on the restored clock
(DEC-21).

## Maps

**Maps are not saved and cannot be reloaded** (DEC-28). The map is Nav2's global costmap,
built from the head's sonar sweeps in a frame that is odometry, so it drifts with the gait
and means nothing on the next boot. `scripts/save-map.sh` and the RTAB-Map database are
gone; what replaces them is undecided ([OQ-20](open-questions.md)).

To watch the map while the robot runs: the dashboard's map panel on port 8080, or

```bash
ros2 topic echo /global_costmap/costmap --once --field info
```

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
ros2_ws/src/hexapod_controller/test_ros.sh             # leg controller alone with direct servo access: init, walk, home, relax
```

### Head calibration

Battery, owner present, service stopped. Nothing here is verified yet
([OQ-21](open-questions.md)); the values live in
`hexapod_controller/config/body_params.yaml` under `head_controller`.

```bash
sudo systemctl stop hexapod
scripts/launch.sh autonomy:=false camera:=false &    # homes, then stands about 15 s later
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
# 1. Pan sign. The head scans on its own; watch it and compare with the joint state.
#    head_pan_joint must be POSITIVE when the head points to the robot's LEFT.
#    If it is inverted, flip head_controller.pan_direction to -1.0.
ros2 topic echo /joint_states --once
# 2. Travel limits. Step to each limit and check nothing fouls; widen or narrow
#    pan_limit_left / pan_limit_right, staying inside the vendor's 50-180 servo range.
ros2 action send_goal /look_around hexapod_interfaces/action/LookAround "{sweeps: 1}"
# 3. Slew rate. Time a full limit-to-limit sweep: 2 * (pan_limit_left + pan_limit_right)
#    degrees over the sweep's duration, minus the dwells, is the rate. Set slew_rate.
# 4. Bearing check. Put a box 1 m ahead and to one side, open the dashboard's sonar
#    fan (port 8080) and confirm the echo appears at the right bearing.
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
ros2 topic hz /ultrasonic/range          # 15 Hz
ros2 run tf2_ros tf2_echo base_link ultrasonic_link   # swings with the head
curl -s localhost:8080/status            # includes the live sonar range
```

## Development loop

Authoring happens on the workstation, and the robot is driven over SSH ([DEC-29](decisions.md)).

```bash
# workstation: commit and push, then on the robot
ssh <robot> 'cd ~/Code/wk-hexapod && [ -z "$(git status --porcelain)" ] && git fetch && git reset --hard origin/main'
ssh <robot> 'cd ~/Code/wk-hexapod/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install'
ssh <robot> 'sudo systemctl restart hexapod'     # one call; inspect in another
```

- **Restart in one SSH call and inspect in another.** A cleanup that kills by pattern also
  matches an SSH session whose command line names a node.
- **Starting `scripts/launch.sh` in the background** (`setsid nohup ... &`): `$!` is the
  wrapper shell, not `ros2 launch`, so killing it does nothing, and a relaunch starts a
  second stack. Two stacks fight over I2C and GPIO 4: the second `servo_driver` cannot claim
  servo power, and the robot behaves inexplicably (2026-09-15, twice). Find the real process
  with `ps -eo pid,args | grep '^ *[0-9]* /usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch
  hexapod_bringup'`, SIGINT it, and confirm no node processes remain before relaunching.
  Prefer `hexapod.service`.

Python nodes are symlink-installed: restart the service after editing. Rebuild after
changing `hexapod_interfaces`, any `setup.py`, launch files, or config files (they are
`data_files`). Record what you ran in [`test-log.md`](test-log.md).

## Troubleshooting

- **`ros2 topic list` is empty right after launch.** DDS discovery takes about 10 s here.
  Use `ros2 daemon start` and query again; `--no-daemon` gives partial lists.
- **Nav2 "Failed to make progress".** Expected on USB power (servos dead). On battery,
  run the movement calibration above; before DEC-22 this was the velocity mismatch.
- **No `/ultrasonic/range`.** The driver claims GPIO 27 and 22; another process holding
  them, or a missing `lgpio`, is logged at start-up. `ros2 topic hz /ultrasonic/range`
  should read 15 Hz. Repeated "No echo pulse from HC-SR04" means the sensor is not
  answering at all — check the head wiring, not the software.
- **`camera_ros` exits immediately.** The OV5647 is not probing ([OQ-23](open-questions.md));
  use `camera:=false` until the ribbon is sorted out.
- **The head does not move.** It is only driven between a home/stand and a relax, like the
  legs, and it needs the battery: the servo rail is dead on USB power. Check
  `ros2 topic echo /head_command` — if commands are flowing, the fault is power or wiring.
- **The map does not grow.** It only grows where the sonar has looked. Confirm the head is
  scanning (`/head_command`), that `base_link → ultrasonic_link` resolves, and that
  readings are arriving; a stationary robot with a still head maps one cone and no more.
- **Collision monitor flaps "stop / continue".** Transform lag between the range source
  and `odom → base_link`; fixed by DEC-09 on 2026-09-09. If it recurs, check load
  ([OQ-02](open-questions.md)).
- **The buzzer sounds.** Something released GPIO 17. `systemctl status hexapod-buzzer-guard`
  and [`hardware.md`](hardware.md#the-buzzer-hazard).
- **A tool loops or hangs under load.** `ros2 topic hz` has been seen to hang at load
  averages above 8; read the journal instead.
- **After an unclean power-off.** Root is ext4 and journals; check
  `tune2fs -l /dev/nvme0n1p2 | grep state`. The FAT boot partition sets a dirty bit:
  confirm with `fsck.vfat -n /dev/nvme0n1p1`, then `umount /boot/firmware`,
  `fsck.vfat -a /dev/nvme0n1p1`, `mount /boot/firmware`.
- **Boot order.** `rpi-eeprom-config` shows `BOOT_ORDER=0xf146`: NVMe, then USB mass
  storage, then SD, read right to left (DEC-24). Root and boot mount by label, so the
  SSD boots from either bus without edits.
- **The SSD is ever put in a USB enclosure again.** Do not force
  `provisioning_mode=unmap` on a Realtek RTL9210 bridge; that hung the disk and the host
  on 2026-09-18 ([`test-log.md`](test-log.md)).
