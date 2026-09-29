#!/usr/bin/env python3
"""Free I2C bus 1 when a device holds SDA low (OQ-32), without a power cycle.

Run as root on the robot, with the stack stopped:

    sudo python3 scripts/i2c-recover.py            # report the lines, change nothing
    sudo python3 scripts/i2c-recover.py --recover  # clock SCL until SDA is released

--recover unbinds the kernel's I2C controller, clocks SCL by hand (GPIO 3) one pulse
at a time until SDA (GPIO 2) reads high, sends a STOP, and binds the controller
again, which returns both pins to their I2C function. This is the bus-clear
procedure of the I2C specification (NXP UM10204, 3.1.16).

It refuses to run while hexapod.service is active or while servo power is enabled
(GPIO 4 low). It claims GPIO 2 and 3 only. It never touches GPIO 17 (the buzzer).
"""
import argparse
import mmap
import os
import struct
import subprocess
import sys
import time

import lgpio

GPIOCHIP = 4                      # RP1 on the Pi 5
SDA, SCL, SERVO_POWER = 2, 3, 4
I2C_FUNCSEL = 3                   # RP1 alternate function a3: I2C1 on GPIO 2 and 3
CONTROLLER = '1f00074000.i2c'
DRIVER = '/sys/bus/platform/drivers/i2c_designware'
RP1_IO_BANK0 = 0x1f000d0000
HALF_PERIOD = 0.001               # s; 500 Hz
MAX_PULSES = 16


class Pads:
    """Read-only view of the RP1 pad registers: reading reconfigures nothing."""

    def __init__(self):
        fd = os.open('/dev/mem', os.O_RDONLY | os.O_SYNC)
        self.io = mmap.mmap(fd, 0x1000, mmap.MAP_SHARED, mmap.PROT_READ,
                            offset=RP1_IO_BANK0)

    def _reg(self, offset):
        return struct.unpack_from('<I', self.io, offset)[0]

    def level(self, pin):
        return (self._reg(pin * 8) >> 17) & 1      # STATUS.INFROMPAD

    def funcsel(self, pin):
        return self._reg(pin * 8 + 4) & 0x1f       # CTRL.FUNCSEL

    def report(self, label):
        print(f'{label}: SDA={self.level(SDA)} SCL={self.level(SCL)} '
              f'funcsel SDA={self.funcsel(SDA)} SCL={self.funcsel(SCL)} '
              f'(I2C is {I2C_FUNCSEL})')


def release(handle, pin):
    """Let the pull-up take the line high."""
    lgpio.gpio_claim_input(handle, pin)


def pull_low(handle, pin):
    lgpio.gpio_claim_output(handle, pin, 0)


def rebind(action):
    with open(f'{DRIVER}/{action}', 'w') as f:
        f.write(CONTROLLER)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--recover', action='store_true',
                        help='clock SCL until SDA is released, then rebind the controller')
    args = parser.parse_args()

    if os.geteuid() != 0:
        sys.exit('Run as root: /dev/mem and the driver bind files need it.')

    pads = Pads()
    pads.report('Before')
    if not args.recover:
        return

    service = subprocess.run(['systemctl', 'is-active', 'hexapod'],
                             capture_output=True, text=True).stdout.strip()
    if service != 'inactive':
        sys.exit(f'hexapod.service is {service}: stop it first.')
    if pads.level(SERVO_POWER) == 0:
        sys.exit('Servo power is enabled (GPIO 4 low): refusing to touch the bus.')

    rebind('unbind')
    handle = lgpio.gpiochip_open(GPIOCHIP)
    try:
        release(handle, SDA)
        release(handle, SCL)
        time.sleep(HALF_PERIOD)
        if pads.level(SCL) == 0:
            print('SCL is held low by a device: clocking cannot free it. '
                  'Power the robot off and on.')
        pulses = 0
        while pads.level(SDA) == 0 and pulses < MAX_PULSES:
            pull_low(handle, SCL)
            time.sleep(HALF_PERIOD)
            release(handle, SCL)
            time.sleep(HALF_PERIOD)
            pulses += 1
        freed = pads.level(SDA) == 1
        print(f'SCL pulses sent: {pulses}; SDA {"released" if freed else "STILL LOW"}')
        if freed:
            # STOP: SDA rises while SCL is high. Ends whatever transfer the
            # device thought it was in, discarding any partial byte.
            pull_low(handle, SCL)
            time.sleep(HALF_PERIOD)
            pull_low(handle, SDA)
            time.sleep(HALF_PERIOD)
            release(handle, SCL)
            time.sleep(HALF_PERIOD)
            release(handle, SDA)
            time.sleep(HALF_PERIOD)
    finally:
        lgpio.gpio_free(handle, SDA)
        lgpio.gpio_free(handle, SCL)
        lgpio.gpiochip_close(handle)
        rebind('bind')

    time.sleep(0.2)
    pads.report('After')
    if pads.funcsel(SDA) != I2C_FUNCSEL or pads.funcsel(SCL) != I2C_FUNCSEL:
        sys.exit('The pins did not return to their I2C function: reboot the Pi.')
    if not freed:
        sys.exit('SDA is still held low: power the robot off and on.')
    print('Now check the bus: i2cdetect -y 1 (expect 40 41 48 68)')


if __name__ == '__main__':
    main()
