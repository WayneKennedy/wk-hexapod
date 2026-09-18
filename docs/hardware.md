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

## Sensor swap

| Kit part | Replaced by | Why |
|---|---|---|
| OV5647 Pi camera on the pan/tilt head | Intel RealSense D435i on the same head | RGB + depth + IMU in one USB 3 device; depth computed in the camera (DEC-02) |
| HC-SR04 ultrasonic on the head (GPIO 27/22) | `/scan` derived from RealSense depth | Software-timed echo on a non-real-time kernel was unreliable; a single range point cannot feed a costmap |

The D435i on the robot (2026-09-09): serial `032622073916`, firmware **5.17.0.10**,
enumerates as USB 3.2. Its IMU calibration is not available on this unit (the driver warns
and uses defaults). Streams in use: colour and aligned depth at **640×480, 15 fps**; the
camera's gyro and accel are enabled but nothing consumes them ([OQ-07](open-questions.md)).

## Pin and address map

| Function | Interface | Detail |
|---|---|---|
| Servo drivers | I2C bus 1, `0x41` and `0x40` | 0x41 serves channels 0–15, 0x40 channels 16–31 (the reference code's ordering, kept). 50 Hz, 500–2500 µs |
| Head pan / tilt | 0x41 channels 0 / 1 | 90° = centred |
| Legs | leg 1 RF 15,14,13 · leg 2 RM 12,11,10 · leg 3 RR 9,8,**31** · leg 4 LR 22,23,**27** · leg 5 LM 19,20,21 · leg 6 LF 16,17,18 | coxa, femur, tibia; legs 3 and 4 have non-contiguous tibia channels |
| Servo power enable | GPIO 4 | **Low = enabled.** Firmware boots it high (off); `servo_driver` drives it low on start; the service leaves it high on stop |
| Body IMU | I2C bus 1, `0x68` | MPU6050, ±2 g, ±250 °/s, polled at 100 Hz |
| Battery ADC | I2C bus 1, `0x48` | ADS7830; channel 0 = LOAD rail (servos), channel 4 = CTRL rail (Pi) |
| LED strip | SPI0 MOSI (GPIO 10), `/dev/spidev0.0` | 7× WS2812, GRB, 50 % brightness |
| Buzzer | GPIO 17 | See the hazard below |
| I2C clock | `dtparam=i2c_arm=on,i2c_arm_baudrate=400000` | 100 kHz makes the robot walk visibly slowly (Freenove) |

Verify the bus with `i2cdetect -y 1`: expect `40 41 48 68`.

## Leg geometry

Link lengths coxa **33**, femur **90**, tibia **110 mm**. Leg mounting angles
`[54, 0, −54, −126, 180, 126]°` and offsets `[94, 85, 94, 94, 85, 94] mm` from the body
origin; default foot positions at `(±137.1, ±189.4)` and `(±225, 0)` mm; body height 30 mm
above the home pose. All from the reference `control.py`; see
`hexapod_controller/config/body_params.yaml` for the tunable subset.

## Power

| Mode | Source | Works | Does not |
|---|---|---|---|
| Battery | 2× 18650 through the shield: LOAD rail (servos), CTRL rail (Pi) | Everything | — |
| USB | USB-C into the Pi | Pi, I2C sensors, LEDs, RealSense, all software | **Servos.** The PCA9685 logic answers on I2C but the servo rail is dead |

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
