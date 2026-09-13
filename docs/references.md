# References

Upstream code this robot depends on, the vendor reference it was ported from, and the
versions verified on the robot on 2026-09-09.

## Vendor reference

- **Freenove Big Hexapod Robot Kit for Raspberry Pi** —
  [Freenove/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi](https://github.com/Freenove/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi).
  Tutorial, calibration graph, and datasheets for PCA9685, MPU6050 and ADS7830.
- **Local checkout: `../freenove-hexapod`**, a sparse clone of the above holding only
  `Code/Server/` (the recipe is in [`operations.md`](operations.md)). The full upstream is
  477 MB of tutorial PDFs and desktop-client binaries; the ten files that matter total
  about 70 KB. Upstream `master` was at commit `b7d228cc870b` (2026-03-07) when this was
  written; re-check the drift table below when it moves ([DEC-20](decisions.md)).
  Confirmed-working files in `Code/Server/`:

  | File | What it proves |
  |---|---|
  | `servo.py`, `pca9685.py` | Channel-to-chip mapping (0x41 first), pulse range, relax |
  | `home.py`, `stand.py` | The calibrated home pose and the smooth stand sequence |
  | `control.py` | Body-centric IK, tripod and wave gaits, balance PID, calibration maths |
  | `imu.py`, `adc.py`, `buzzer.py`, `spi_ledpixel.py`, `ultrasonic.py` | Bus and pin usage for every shield peripheral |

  Drift check on 2026-09-09, against the 2025-11-28 state the robot was ported from:
  only `control.py` had changed, by `np.mat` → `np.asmatrix` (numpy 2); the rest was
  byte-identical. This repo's controller does not use `np.mat`.

  **Licence:** upstream is CC BY-NC-SA 3.0 (`LICENSE.txt` there). This repo is Apache-2.0.
  The reference is read, not vendored, for that reason ([DEC-20](decisions.md),
  [OQ-15](open-questions.md)).

## ROS 2 packages (apt, Jazzy, arm64) — versions installed 2026-09-09

| Package | Version | Role |
|---|---|---|
| `ros-jazzy-ros-base` | 0.11.0 | ROS 2 Jazzy |
| `ros-jazzy-realsense2-camera` / `librealsense2` | 4.58.1 / 2.58.1 | D435i driver. Topics prefixed `/camera/camera/`; profile params are `depth_module.depth_profile` and `rgb_camera.color_profile` |
| `ros-jazzy-rtabmap-ros` | 0.22.1 | RGB-D SLAM. `queue_size` renamed `sync_queue_size` |
| `ros-jazzy-navigation2`, `nav2-bringup` | 1.3.12 | Nav2. Jazzy specifics recorded in `nav2_params.yaml` |
| `ros-jazzy-depthimage-to-laserscan` | 2.5.1 | `/scan` from depth |
| `ros-jazzy-imu-filter-madgwick` | 2.1.5 | `/imu/data` orientation |
| `ros-jazzy-robot-state-publisher` | 3.3.4 | URDF → TF |
| `ros-jazzy-foxglove-bridge` | 3.4.1 | Installed, not launched |

Python from apt: `python3-gpiozero` 2.0.1, `python3-lgpio`, `python3-spidev`,
`python3-smbus`, `python3-numpy` 1.26, `python3-opencv` 4.6, `python3-flask` 3.0. From
pip into the system interpreter: `rpi-ws281x` 5.0.0, `mpu6050-raspberrypi` 1.2,
`face_recognition` (dlib 20.0.1, built from source).

## Documentation drawn on

- [Raspberry Pi `config.txt` GPIO control](https://www.raspberrypi.com/documentation/computers/config_txt.html#gpio-control)
  — the `gpio=` directive used for the safe boot defaults.
- [librealsense udev rules](https://github.com/IntelRealSense/librealsense/blob/master/config/99-realsense-libusb.rules)
  — installed by the setup script.
- [micro-ROS board support](https://github.com/micro-ROS/micro_ros_arduino) — relevant
  only to [OQ-09](open-questions.md).

## Family

- [wk-robotics](https://github.com/WayneKennedy/wk-robotics) — index, compute tiers,
  topic contract, perception placement, power integrity, conventions.
- [wk-devastator](https://github.com/WayneKennedy/wk-robotics/tree/main/projects/devastator) — intended second
  consumer of this robot's SLAM and Nav2 configuration; its `docs/` are the format this
  repository's documentation follows.
