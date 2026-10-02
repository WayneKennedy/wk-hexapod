#!/usr/bin/env python3
"""Test the kernel's I2C bus recovery (OQ-32): hold SDA low as a stuck device would.

    sudo python3 scripts/i2c-fault-test.py

Forces GPIO 2 (SDA) low through the RP1 pad's output override, with the pin left on
its I2C function, then waits up to 10 s for the override to clear. The next transfer
on the bus (the stack's IMU at 100 Hz, or an i2cget) fails with "lost arbitration" and
"controller timed out", the 2026-09-29 signature. The recovery set up by
boot/hexapod-i2c1-recovery.dts switches the pins to GPIO, which resets the override:
the line is released and the bus works again. Without the overlay nothing clears it,
and this script clears the override itself after 10 s. Needs root (/dev/mem). Touches
GPIO 2's control register only.
"""
import mmap, os, struct, time
BASE, CTRL = 0x1f000d0000, 2 * 8 + 4
fd = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
m = mmap.mmap(fd, 0x1000, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=BASE)
rd = lambda o: struct.unpack_from('<I', m, o)[0]
sda = lambda: (rd(2 * 8) >> 17) & 1
c = rd(CTRL)
print(f'before: ctrl={c:#010x} SDA={sda()}')
struct.pack_into('<I', m, CTRL, (c & ~0xf000) | (2 << 12) | (3 << 14))
t0 = time.monotonic()
print(f'held:   ctrl={rd(CTRL):#010x} SDA={sda()}')
while time.monotonic() - t0 < 10:
    if (rd(CTRL) >> 12) & 0xf == 0 and sda():
        print(f'released after {time.monotonic() - t0:.3f} s: ctrl={rd(CTRL):#010x} SDA={sda()}')
        break
    time.sleep(0.001)
else:
    c = rd(CTRL)
    struct.pack_into('<I', m, CTRL, c & ~0xf000)
    print(f'NOT released in 10 s; override cleared by hand: ctrl={rd(CTRL):#010x} SDA={sda()}')
