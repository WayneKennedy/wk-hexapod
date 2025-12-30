# Ubuntu Server Hardware Setup for Hexapod

This document covers hardware configuration for Raspberry Pi 5 running Ubuntu Server 24.04,
cross-referenced with Freenove Tutorial Chapter 1 and community findings.

## Automated Setup

Use the setup script for automated configuration:

```bash
sudo ./scripts/ubuntu-setup.sh --camera-model ov5647 --camera-port cam0
```

Run with `--dry-run` to preview changes without applying them.

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

### 3. Camera Configuration

Two options:

**Option A: Auto-detect (current)**
```
camera_auto_detect=1
```

**Option B: Explicit overlay (Freenove method)**
```
camera_auto_detect=0
dtoverlay=ov5647,cam0
```

Note: Pi 5 has two camera ports (cam0, cam1). Check physical connection.

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

**Option A: Use reference code's custom implementation** (in `reference/pca9685.py`)

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

The hexapod uses PCB V2.0 with Pi 5, so LEDs are driven via SPI (`spi_ledpixel.py`).

Configuration in `config/params.json`:
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
