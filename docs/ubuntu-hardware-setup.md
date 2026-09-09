# Ubuntu Server Hardware Setup for Hexapod

This document covers hardware configuration for Raspberry Pi 5 running Ubuntu Server 24.04,
cross-referenced with Freenove Tutorial Chapter 1 and community findings.

## Automated Setup

Use the setup script for automated configuration (config.txt, ROS 2 Jazzy and all
dependencies from apt, pip packages, udev rules, groups, workspace build):

```bash
sudo ./scripts/ubuntu-setup.sh
```

Run with `--dry-run` to preview changes without applying them.

## Safe GPIO defaults (buzzer)

The shield buzzer on GPIO 17 sounds continuously whenever the pin floats, which is the
state after any process that claimed it exits. `config.txt` therefore carries:

```
gpio=17=op,dl    # buzzer output low from firmware boot
gpio=4=op,dh     # servo power disabled until servo_driver enables it
```

and `systemd/hexapod-buzzer-guard.service` holds GPIO 17 low while the system runs.
Do not read the pin with `gpioget`; it turns the line into an input.

## Foreign packages

If Raspberry Pi OS (bookworm) apt sources were ever enabled on this host, packages such
as `libswresample4`, `libwayland-*`, `libssl3`, `libcamera*` and `linux-libc-dev` may be
the `+rpt`/`deb12` builds. They block ROS packages with errors like
`libswresample-dev : Depends: libswresample4 (= 7:6.1.1-3ubuntu5) but 8:5.1.8-0+deb12u1+rpt1 is to be installed`.

Find them with:

```bash
dpkg-query -W -f='${Package}\t${Version}\n' | grep -E 'rpt|deb12'
```

Fix: disable the Pi repo, then downgrade each to the noble version with
`apt-get install --allow-downgrades pkg=<noble version>` (from `apt-cache madison pkg`),
and remove bookworm-only packages (`libavutil57`, `libssl3`, `libcamera*`, `rpicam-apps*`).
`initramfs-tools` from the Pi repo can stay; it does not conflict.

## Manual Configuration

Check your config at `/boot/firmware/config.txt`.

## Required Configuration

### 1. I2C and SPI (Already enabled in default Ubuntu)

```
dtparam=i2c_arm=on
dtparam=spi=on
```

### 2. I2C Baud Rate (MISSING - required for fast servo response)

Add to the `dtparam=i2c_arm=on` line:

```
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
```

Per Freenove: "Default is 100000. We change to 400000 to speed up servo response.
If baud rate is 100,000, the robot walks slowly."

### 3. Camera

The Pi Camera is no longer fitted; the Intel RealSense D435i on USB 3 provides RGB, depth
and IMU. It needs the librealsense udev rules (installed by the setup script to
`/etc/udev/rules.d/99-realsense-libusb.rules`) so the ROS node can open it as a normal user.
Any leftover `dtoverlay=ov5647` line in config.txt is harmless.

### 4. Verify I2C is working

```bash
# Check kernel modules
lsmod | grep i2c

# Install tools
sudo apt install i2c-tools

# Scan for devices (PCA9685 should be at 0x40 and 0x41)
sudo i2cdetect -y 1
```

Expected output with hexapod connected:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
...
40: 40 41 -- -- -- -- -- -- 48 -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
```
- 0x40, 0x41 = PCA9685 PWM drivers
- 0x48 = ADS7830 ADC
- 0x68 = MPU6050 IMU

## Python Library Differences

### GPIO Library

**Raspberry Pi OS**: RPi.GPIO
**Ubuntu Server (kernel 5.11+)**: LGPIO

```bash
sudo apt install python3-lgpio
```

The reference code uses `gpiozero` which should work with LGPIO backend.

### I2C Library

```bash
sudo apt install python3-smbus
```

### PCA9685 Servo Driver

Old Adafruit library is deprecated. Options:

**Option A: Use reference code's custom implementation** (in `../fn-hexapod/Code/Server/pca9685.py`)

**Option B: Use new Adafruit CircuitPython library**
```bash
pip3 install adafruit-circuitpython-pca9685
```

### Camera (picamera2)

Not in Ubuntu archive. Install from PPA:

```bash
sudo apt install ffmpeg
sudo add-apt-repository ppa:r41k0u/python3-simplejpeg
sudo apt install python3-picamera2
```

### WS2812 LEDs

**PCB Connector Versions:**
- V1.0: Uses PWM on GPIO18 - **Not compatible with Pi 5**
- V2.0: Uses SPI on GPIO10 - **Required for Pi 5**

The hexapod uses PCB V2.0 with Pi 5, so LEDs are driven via SPI (`../fn-hexapod/Code/Server/spi_ledpixel.py`).

Configuration in `../fn-hexapod/Code/Server/params.json`:
```json
{
    "Pcb_Version": 2,
    "Pi_Version": 2
}
```

Requirements:
- `dtparam=spi=on` in config.txt (already set)
- `spidev` Python library (installed via requirements.txt)

```bash
pip3 install rpi_ws281x spidev
```

## User Permissions

```bash
sudo usermod -aG i2c,spi,dialout,gpio $USER
# Logout and login for groups to take effect
```

## Test Commands

```bash
# Check I2C devices
sudo i2cdetect -y 1

# Check SPI
ls /dev/spidev*

# Check camera
libcamera-hello --list-cameras

# Check GPIO access
ls -la /dev/gpiomem
```

## References

- [Freenove Big Hexapod GitHub](https://github.com/Freenove/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi)
- [Ubuntu GPIO Tutorial](https://ubuntu.com/tutorials/gpio-on-raspberry-pi)
- [Canonical Camera Documentation](https://canonical-ubuntu-hardware-support.readthedocs-hosted.com/boards/how-to/rpi-camera/)
- [Raspberry Pi config.txt Documentation](https://www.raspberrypi.com/documentation/computers/config_txt.html)
