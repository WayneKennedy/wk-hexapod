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
  | `home.py`, `stand.py` | The calibrated home pose and the smooth stand sequence. **Not upstream files** (corrected 2026-09-21): they were written in the old `fn-hexapod` snapshot (commit `926f538`, with a `CLAUDE.md` and the calibration now in `config/servo_calibration.txt`). The snapshot was deleted from the robot on 2026-09-21 per DEC-20. A full git bundle is kept outside any repo on the always-on workstation, at `~/Archive/fn-hexapod-2026-09-21.bundle` |
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
| `ros-jazzy-navigation2`, `nav2-bringup` | 1.3.12 | Nav2. Jazzy specifics recorded in `nav2_params.yaml`. `RangeSensorLayer` is the map's only source (DEC-28); the collision monitor's `range` source reads the same topic |
| `ros-jazzy-camera-ros` / `ros-jazzy-libcamera` | 0.7.0 / 0.7.2 | OV5647 driver (added 2026-09-19, DEC-25). The ROS build of libcamera carries the Raspberry Pi **PiSP** pipeline, `libpisp` and an `ov5647.json` tuning file, which Ubuntu's own `libcamera0.2` (0.2.0) does not: **no source build is needed on a Pi 5** |
| `ros-jazzy-imu-filter-madgwick` | 2.1.5 | `/imu/data` orientation |
| `ros-jazzy-robot-state-publisher` | 3.3.4 | URDF → TF |
| `ros-jazzy-foxglove-bridge` | 3.4.1 | Installed, not launched |

Python from apt: `python3-gpiozero` 2.0.1, `python3-lgpio` (its edge timestamps are
`CLOCK_MONOTONIC`; the ultrasonic driver times echoes with them), `python3-spidev`,
`python3-smbus`, `python3-numpy` 1.26, `python3-opencv` 4.6, `python3-flask` 3.0. From
pip into the system interpreter: `rpi-ws281x` 5.0.0, `mpu6050-raspberrypi` 1.2,
`face_recognition` (dlib 20.0.1, built from source).

## Documentation drawn on

- [Raspberry Pi `config.txt` GPIO control](https://www.raspberrypi.com/documentation/computers/config_txt.html#gpio-control)
  — the `gpio=` directive used for the safe boot defaults, and `dtoverlay=ov5647,cam0`.
- [Nav2 `RangeSensorLayer`](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/plugins/range_sensor_layer.cpp)
  — read for DEC-28: it keeps unseen cells unknown, and only clears on a no-echo reading
  when that reading equals `max_range`, which is why the driver reports it that way.
- [micro-ROS board support](https://github.com/micro-ROS/micro_ros_arduino) — relevant
  only to [OQ-09](open-questions.md).

## Family

- [wk-robotics](https://github.com/WayneKennedy/wk-robotics) — index, compute tiers,
  topic contract, perception placement, power integrity, conventions.
- [wk-devastator](https://github.com/WayneKennedy/wk-robotics/tree/main/projects/devastator) — intended second
  consumer of this robot's SLAM and Nav2 configuration; its `docs/` are the format this
  repository's documentation follows.
