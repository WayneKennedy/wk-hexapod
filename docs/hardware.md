# Hardware

The kit, what replaced what, the pin and address map, power, calibration, and the buzzer
hazard. Everything here was read from the running robot or the kit's code; dates are
given where a value was measured.

## The kit

Freenove Big Hexapod Robot Kit for Raspberry Pi, FNK0052, with the **V2.0 shield** (SPI
LED connector; the V1.0 PWM-on-GPIO18 variant does not work on a Pi 5). Vendor
documentation and datasheets for the shield's three chips (PCA9685, MPU6050, ADS7830) are
in the [Freenove repository](https://github.com/Freenove/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi).
They are not copied here.

Compute: **Raspberry Pi 5, 8 GB**, Ubuntu Server 24.04, ROS 2 Jazzy. Storage: a 128 GB
NVMe SSD on an M.2 HAT on the PCIe connector; root and boot mount by label. Bootloader
`BOOT_ORDER=0xf146` (NVMe, USB, SD; read right to left), release 2025-12-08. The SSD
spent 2026-09-18 in a USB 3 enclosure to free the PCIe connector for an AI HAT+ 2; the
HAT went to the tank bot instead (DEC-24, [`test-log.md`](test-log.md)).

The shield is a breakout, not a controller: it carries no microcontroller. Every device
below is a direct peripheral of the Pi ([`architecture.md`](architecture.md#buses)).

## Head sensors

The head is the kit's own again. The RealSense D435i was **removed on 2026-09-19** and
reassigned to another robot; the **OV5647 Pi camera and the HC-SR04 ultrasonic sensor
were refitted** to the pan/tilt head (DEC-25). What the robot has to sense with:

| Sensor | Interface | What it gives | What it does not |
|---|---|---|---|
| OV5647 Pi camera | CSI, CAM0, `dtoverlay=ov5647,cam0`, libcamera → `camera_ros` | Colour images, `/camera/image_raw` | No depth, no scale, no odometry |
| HC-SR04 ultrasonic | GPIO 27 trigger, GPIO 22 echo | One range, 0.03–2 m, 15 Hz, in a ~15° cone wherever the head points | No bearing within the cone; misses angled and soft surfaces |

Neither is fixed to the body: both sit on the pan/tilt head, so **where the robot can see
is a head-servo decision**, made by `head_controller` ([`architecture.md`](architecture.md)).

**Slamtec RPLIDAR C1 2D lidar, in hand and dry-fitted 2026-09-26** on the body, not the head —
RobotShop #1499979, £47.76 ex VAT, product code RB-Rpk-35; one of two, the other is in
[wk-inventory stock](https://github.com/WayneKennedy/wk-inventory/blob/main/docs/stock.md).
Datasheet facts and the Jazzy driver status are in
[wk-robotics `common.md`](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/common.md#depth-which-kind-for-which-task);
what it is for, and what is open until it arrives, is [OQ-19](open-questions.md). Bench
facts still to record: mass as delivered, current over USB (datasheet: 230 mA typical at 5 V,
so USB power suffices).

**Dry fit, 2026-09-26 (owner's photo):** an old Pi VESA mount plate — Pi hole pattern
underneath, honeycomb fill — on four brass standoffs above the Freenove shield, the lidar
sitting loose on it (VESA holes do not match the C1's 43 mm square; the plate needs four
M2.5 holes, ≤ 4 mm into the lidar). **The replacement plate is designed:**
[wk-robotics `tools/lidar-mount/`](https://github.com/WayneKennedy/wk-robotics/tree/main/tools/lidar-mount)
(2026-09-26) — HAT outline on the Pi holes, 10 mm bosses, M2.5 × 8 from below, cable slot;
scan plane 39.8 mm above the plate top. **Printed and fitted 2026-09-28** (owner: "success",
with a photo). In that photo the USB lead leaves the back of the lidar and drops beside the stack,
looping at about plate height, below the scan window; not measured. Seen in the 2026-09-26 dry-fit photo, for OQ-19: the USB lead was coiled
beside the lidar *in* the scan plane (30 mm above the base) — route it down through the plate;
the pan/tilt head tops out near the lidar's mid-height, so whether it crosses the plane
depends on tilt — 2026-10-02: tilted 15–20° above level, the head's top edge stood level with
the lidar's base, beside it (owner's photo, "too close for comfort"); not measured further; a raised leg reaches the plane in swing, so a body-radius range
mask on the scan is cheap insurance.

**Bench facts, 2026-09-28, USB power, legs limp** ([`test-log.md`](test-log.md)):

- **Driver:** `sllidar_ros2` (SDK 2.1.0; DEC-31). The C1 reports firmware 1.02, hardware
  rev 18, health OK; Standard mode, 5 kHz, **10.0 Hz, 720 points at 0.5°**, `/dev/ttyUSB0`
  (CP2102N). apt's `rplidar_ros` 2.1.0 does not run it.
- **Height:** scan plane ~155 mm above the belly, belly ~35 mm off the floor at the 30 mm
  stand (owner's measurements, 2026-09-28), so ~190 mm above the floor then. **At the
  50 mm stand ([DEC-34](decisions.md)) that is ~210 mm, by arithmetic, not measured.**
  `laser_frame` z = 0.16 m above `base_link`. Nothing lower than the scan plane is seen
  when the body is level.
- **Plan position:** lidar centre 10 mm forward of the body centre, centred side to side
  (owner, 2026-09-28): `laser_frame` x = +0.010 m, y = 0.
- **Yaw π.** The arrow on the C1's cap faces the robot's front, and the driver's 0° points
  the other way. Found with objects at owner-measured positions from the lidar centre:
  boxes 500 mm ahead read 0.494–0.50 m at 180°; a wall 270 mm behind read 0.274 m at 0°;
  a pole ~240 mm behind-left read 0.234 m at −50°, which also shows the scan is not
  mirrored.
- **No self-hits at rest:** the nearest return with nothing placed near the robot was
  0.29 m. `slam_params.yaml` drops returns under 0.25 m. Not checked: a leg in swing, and
  the head at full tilt.

**Echo timing.** DEC-02 removed the ultrasonic partly because a software-timed echo on a
non-real-time kernel was unreliable. The driver now times the echo from the kernel's
line-event timestamps (lgpio alerts) rather than Python wake-ups. Measured 2026-09-19 on
USB power, load average ~4, against a fixed target: **15 consecutive pings, 0.222–0.224 m**
([`test-log.md`](test-log.md)).

**The camera is not working yet** (2026-09-19): the sensor does not acknowledge on I2C, so
`ov5647: probe of 10-0036 failed with error -121` and libcamera reports no cameras. The
ribbon is the suspect — seating, contact orientation, or the wrong connector on the Pi 5
(CAM/DISP 0 versus 1). Everything above it in the stack is installed and configured.

## Pin and address map

| Function | Interface | Detail |
|---|---|---|
| Servo drivers | I2C bus 1, `0x41` and `0x40` | 0x41 serves channels 0–15, 0x40 channels 16–31 (the reference code's ordering, kept). 50 Hz, 500–2500 µs |
| Head pan / tilt | 0x41 channels **1 / 0** (the vendor's code has 0 / 1; found 2026-10-02) | **Pan straight ahead = 103°** (by eye), higher turns left; **tilt level = 65°**, 18° down at 45°, travel ends at 40° (wiring, bracket); measured 2026-10-02. **Pan ±60° (43–163°), clear of cables and bracket; tilt locked to 45–65°, never up** ([DEC-35](decisions.md)). **Tilt drives down only with CTRL on** ([OQ-30](open-questions.md)). **Both disabled since 2026-09-29: the tilt servo's gears slip** ([OQ-30](open-questions.md)) |
| Ultrasonic | GPIO 27 (trigger), GPIO 22 (echo) | HC-SR04 on the head; echo timed from kernel edge timestamps |
| Pi camera | CSI CAM0 | OV5647; `camera_auto_detect=0` and `dtoverlay=ov5647,cam0` in `config.txt` |
| Legs | leg 1 RF 15,14,13 · leg 2 RM 12,11,10 · leg 3 RR 9,8,**31** · leg 4 LR 22,23,**27** · leg 5 LM 19,20,21 · leg 6 LF 16,17,18 | coxa, femur, tibia; legs 3 and 4 have non-contiguous tibia channels |
| Servo power enable | GPIO 4 | **Low = enabled.** Firmware boots it high (off); `servo_driver` drives it low on start; the service leaves it high on stop |
| Body IMU | I2C bus 1, `0x68` | MPU6050 per the kit; it answers `WHO_AM_I` `0x70` ([OQ-35](open-questions.md)). ±2 g, ±250 °/s, polled at 100 Hz. **The chip sits turned 90° from the body** (its y forward, its x to the right; tilted by hand 2026-10-01): `imu_driver` rotates its axes into the body's (`imu.mounting_yaw_deg`), so `/imu/data_raw` and everything after it are in body axes |
| Battery ADC | I2C bus 1, `0x48` | ADS7830; channel 0 = LOAD rail (servos), channel 4 = CTRL rail (Pi) |
| LED strip | SPI0 MOSI (GPIO 10), `/dev/spidev0.0` | 7× WS2812, GRB, 50 % brightness |
| Buzzer | GPIO 17 | See the hazard below |
| I2C clock | `dtparam=i2c_arm=on,i2c_arm_baudrate=400000` | 100 kHz makes the robot walk visibly slowly (Freenove) |
| I2C bus recovery | `dtoverlay=hexapod-i2c1-recovery` | GPIO 3/2 as `scl-gpios`/`sda-gpios`: the kernel clocks a held SDA free on a timeout ([OQ-32](open-questions.md)) |

Verify the bus with `i2cdetect -y 1`: expect `40 41 48 68`. If every address times out,
see [`operations.md`](operations.md#troubleshooting) and [OQ-32](open-questions.md).

## Leg geometry

Link lengths coxa **33**, femur **90**, tibia **110 mm**. Leg mounting angles
`[54, 0, −54, −126, 180, 126]°` and offsets `[94, 85, 94, 94, 85, 94] mm` from the body
origin; default foot positions at `(±137.1, ±189.4)` and `(±225, 0)` mm, all from the reference
`control.py`. The stand raises the body 50 mm above the home pose
([DEC-34](decisions.md)); the reference stands at 25 mm and its app offers 10–50. See
`hexapod_controller/config/body_params.yaml` for the tunable subset.

## Power

| Mode | Source | Works | Does not |
|---|---|---|---|
| Battery | 2× 18650 through the shield: LOAD rail (servos), CTRL rail (Pi) | Everything | — |
| USB | USB-C into the Pi | Pi, I2C sensors, LEDs, ultrasonic, camera, all software | **Servos**, including the head's. The PCA9685 logic answers on I2C but the servo rail is dead |

`power_indicator` maps the two rails onto the LED strip: below 0.5 V reads as USB (blue),
7.0 V and above green, 6.5–7.0 V yellow, below 6.5 V red. There is **no low-voltage cutoff
and no watchdog** below the Pi ([OQ-11](open-questions.md)).

## Calibration

The kit's per-leg foot-position calibration (`point.txt` format: one line per leg, tab
separated x y z) is stored at `ros2_ws/src/hexapod_hardware/config/servo_calibration.txt`
and installed with the package. Both `servo_driver` and the controller resolve it in this
order (DEC-17): the `*.calibration_file` parameter, `~/.hexapod/servo_calibration.txt`,
the installed copy. Values on this robot:

```
-11	10	16
-6	7	9
15	5	18
8	13	10
-4	12	6
20	5	9
```

## The buzzer hazard

The shield buzzer is driven by an NPN stage from GPIO 17 and is **painfully loud**. When
the pin is left floating it sounds continuously. On this kernel a GPIO line reverts to a
floating input the moment the process that requested it exits or is killed — observed
twice on 2026-09-09, after `docker compose down` and after a pin-holding script was
killed ([`test-log.md`](test-log.md)). Both times the robot had to be unplugged.

Three independent safeguards are in place (DEC-10):

1. **Firmware.** `/boot/firmware/config.txt` carries `gpio=17=op,dl` (buzzer low) and
   `gpio=4=op,dh` (servo power off) so both pins are driven before any software runs.
2. **Guard service.** `systemd/hexapod-buzzer-guard.service` runs
   `gpioset --mode=signal gpiochip4 17=0` from `sysinit.target` and holds the line for the
   life of the system.
3. **Software default.** `buzzer.enabled: false` in `hexapod_hardware/config/hardware.yaml`.
   `buzzer_controller` never touches the pin; beep requests are logged and dropped. The
   startup sequence warns with LEDs only.

To use the buzzer deliberately: set `buzzer.enabled: true` **and** stop and disable the
guard service. Never read GPIO 17 with `gpioget` (it turns the line into an input) and
never claim it from a script that can be killed.
