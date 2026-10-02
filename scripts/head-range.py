#!/usr/bin/env python3
"""Drive the head's pan and tilt servos by hand, to measure their range (OQ-21, OQ-26, OQ-30).

Battery (LOAD) on, the owner watching, hexapod.service stopped:

    python3 scripts/head-range.py &              # holds servo power while it runs
    echo 'tilt 90' > /tmp/head-range             # one command per line, see below
    tail -f /tmp/head-range.log

Commands: `pan <deg>` and `tilt <deg>` move to a servo angle (0-180, the scale
servo_driver uses: 500-2500 us; 90 is head_controller's centre), ramping 1 deg
per 20 ms from the last commanded angle. The first command for a servo jumps:
where a limp servo rests is unknown. `relax <pan|tilt>` stops that servo's pulses;
`quit` ends. On start every one of the 32 channels is relaxed, so the legs stay
limp. On exit, by `quit`, a signal or 15 min without a command, every channel is
relaxed and servo power (GPIO 4, low = enabled) is switched off.

Needs the i2c and gpio groups. Claims GPIO 4 only; never touches GPIO 17 (the buzzer).
"""
import os
import select
import signal
import sys
import time

import lgpio
import smbus

FIFO, LOG = '/tmp/head-range', '/tmp/head-range.log'
GPIOCHIP, SERVO_POWER = 4, 4
BUS, ADDR_0_15, ADDR_16_31 = 1, 0x41, 0x40
CHANNEL = {'pan': 1, 'tilt': 0}         # this robot's wiring (hardware.yaml, OQ-21)
STEP_DEG, STEP_SEC = 1.0, 0.02
IDLE_SEC = 15 * 60


def log(msg):
    line = f'{time.strftime("%H:%M:%S")} {msg}'
    print(line, flush=True)
    with open(LOG, 'a') as f:
        f.write(line + '\n')


class Pca9685:
    def __init__(self, bus, addr):
        self.bus, self.addr = bus, addr
        self.bus.write_byte_data(addr, 0x00, 0x00)              # MODE1: awake
        prescale = int(25e6 / (4096 * 50) - 1 + 0.5)            # 50 Hz
        mode = self.bus.read_byte_data(addr, 0x00)
        self.bus.write_byte_data(addr, 0x00, (mode & 0x7F) | 0x10)
        self.bus.write_byte_data(addr, 0xFE, prescale)
        self.bus.write_byte_data(addr, 0x00, mode)
        time.sleep(0.005)
        self.bus.write_byte_data(addr, 0x00, mode | 0x80)

    def set(self, ch, on, off):
        base = 0x06 + 4 * ch
        for i, v in enumerate((on & 0xFF, on >> 8, off & 0xFF, off >> 8)):
            self.bus.write_byte_data(self.addr, base + i, v)

    def angle(self, ch, deg):
        us = 500 + 2000 * deg / 180
        self.set(ch, 0, int(us / 20000 * 4095))

    def relax(self, ch):
        self.set(ch, 4096, 4096)


def main():
    bus = smbus.SMBus(BUS)
    head, legs = Pca9685(bus, ADDR_0_15), Pca9685(bus, ADDR_16_31)
    for ch in range(16):
        head.relax(ch)
        legs.relax(ch)
    gpio = lgpio.gpiochip_open(GPIOCHIP)
    angle = {}

    def shutdown(*_):
        for ch in range(16):
            try:
                head.relax(ch)
                legs.relax(ch)
            except OSError as e:
                log(f'relax failed: {e}')
        lgpio.gpio_write(gpio, SERVO_POWER, 1)
        lgpio.gpio_free(gpio, SERVO_POWER)
        lgpio.gpiochip_close(gpio)
        log('all channels relaxed, servo power off')
        sys.exit(0)

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    if os.path.exists(FIFO):
        os.unlink(FIFO)
    os.mkfifo(FIFO)
    fd = os.open(FIFO, os.O_RDWR | os.O_NONBLOCK)    # RDWR: never sees EOF
    lgpio.gpio_claim_output(gpio, SERVO_POWER, 0)
    log(f'servo power on; all channels relaxed; commands to {FIFO}')

    buf, last = b'', time.monotonic()
    while True:
        ready, _, _ = select.select([fd], [], [], 10)
        if not ready:
            if time.monotonic() - last > IDLE_SEC:
                log('idle 15 min')
                shutdown()
            continue
        buf += os.read(fd, 1024)
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            words = line.decode().split()
            last = time.monotonic()
            try:
                if words == ['quit']:
                    shutdown()
                elif len(words) == 2 and words[0] == 'relax' and words[1] in CHANNEL:
                    head.relax(CHANNEL[words[1]])
                    angle.pop(words[1], None)
                    log(f'{words[1]} relaxed')
                elif len(words) == 2 and words[0] in CHANNEL:
                    name, target = words[0], max(0.0, min(180.0, float(words[1])))
                    ch, a = CHANNEL[name], angle.get(name, target)
                    while abs(target - a) > STEP_DEG:
                        a += STEP_DEG if target > a else -STEP_DEG
                        head.angle(ch, a)
                        time.sleep(STEP_SEC)
                    head.angle(ch, target)
                    angle[name] = target
                    log(f'{name} {target:.0f}')
                else:
                    log(f'not understood: {line.decode().strip()}')
            except (OSError, ValueError) as e:
                log(f'{" ".join(words)}: {e}')


if __name__ == '__main__':
    main()
