#!/usr/bin/env python3
"""
Test: Initialize (home -> stand) using body-centric controller
With debug output.
"""

import math
import os
import time
import smbus


class PCA9685:
    """PCA9685 PWM driver"""
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, bus, address=0x40):
        self.bus = smbus.SMBus(bus)
        self.address = address
        self.bus.write_byte_data(self.address, self.MODE1, 0x00)

    def set_pwm_freq(self, freq):
        prescale = int(25000000.0 / (4096.0 * freq) - 1 + 0.5)
        old_mode = self.bus.read_byte_data(self.address, self.MODE1)
        self.bus.write_byte_data(self.address, self.MODE1, (old_mode & 0x7F) | 0x10)
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode | 0x80)

    def set_pwm(self, channel, on, off):
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel, on & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 1, on >> 8)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 2, off & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 3, off >> 8)

    def set_pwm_off(self, channel):
        self.set_pwm(channel, 4096, 4096)


class HexapodInit:
    """Test body-centric home and stand"""

    LEG_CHANNELS = [
        [15, 14, 13],  # Leg 0 (RF)
        [12, 11, 10],  # Leg 1 (RM)
        [9, 8, 31],    # Leg 2 (RR)
        [22, 23, 27],  # Leg 3 (LR)
        [19, 20, 21],  # Leg 4 (LM)
        [16, 17, 18],  # Leg 5 (LF)
    ]

    L1, L2, L3 = 33.0, 90.0, 110.0
    LEG_ANGLES = [54.0, 0.0, -54.0, -126.0, 180.0, 126.0]
    LEG_OFFSETS = [94.0, 85.0, 94.0, 94.0, 85.0, 94.0]
    HEIGHT_OFFSET = 14.0

    FOOT_XY = [
        (137.1, 189.4),
        (225.0, 0.0),
        (137.1, -189.4),
        (-137.1, -189.4),
        (-225.0, 0.0),
        (-137.1, 189.4),
    ]

    def __init__(self):
        print("Initializing PCA9685...")
        self.pwm_41 = PCA9685(1, 0x41)
        self.pwm_41.set_pwm_freq(50)
        self.pwm_40 = PCA9685(1, 0x40)
        self.pwm_40.set_pwm_freq(50)
        time.sleep(0.1)

        self.body_z = 0.0

        self.calibration = self._load_calibration()
        self.cal_angles = self._calc_calibration()

        print(f"Calibration offsets: {self.cal_angles}")
        print("Ready")

    def _load_calibration(self):
        here = os.path.dirname(os.path.abspath(__file__))
        paths = [
            os.path.expanduser('~/.hexapod/servo_calibration.txt'),
            os.path.join(here, '..', 'hexapod_hardware', 'config', 'servo_calibration.txt'),
            os.path.join(here, '..', '..', '..', '..', 'fn-hexapod', 'Code', 'Server', 'point.txt'),
        ]
        for path in paths:
            try:
                with open(path, 'r') as f:
                    data = [list(map(int, line.strip().split('\t')))
                            for line in f if line.strip()]
                    print(f"Calibration: {path}")
                    return data
            except:
                pass
        return [[140, 0, 0] for _ in range(6)]

    def _calc_calibration(self):
        cal_angles = []
        for i in range(6):
            cal = self._ik(-self.calibration[i][2], self.calibration[i][0], self.calibration[i][1])
            default = self._ik(0, 140, 0)
            cal_angles.append([cal[j] - default[j] for j in range(3)])
        return cal_angles

    def _clamp(self, val, lo, hi):
        return max(lo, min(hi, val))

    def _ik(self, x, y, z):
        a = math.pi / 2 - math.atan2(z, y)
        x_4 = self.L1 * math.sin(a)
        x_5 = self.L1 * math.cos(a)
        l23 = math.sqrt((z - x_5)**2 + (y - x_4)**2 + x**2)

        w = self._clamp(x / l23, -1, 1)
        v = self._clamp((self.L2**2 + l23**2 - self.L3**2) / (2 * self.L2 * l23), -1, 1)
        u = self._clamp((self.L2**2 + self.L3**2 - l23**2) / (2 * self.L3 * self.L2), -1, 1)

        b = math.asin(round(w, 2)) - math.acos(round(v, 2))
        c = math.pi - math.acos(round(u, 2))

        return [round(math.degrees(a)), round(math.degrees(b)), round(math.degrees(c))]

    def _world_to_leg(self, foot_x, foot_y, foot_z, leg_idx):
        angle = math.radians(self.LEG_ANGLES[leg_idx])
        offset = self.LEG_OFFSETS[leg_idx]

        x = foot_x * math.cos(angle) + foot_y * math.sin(angle) - offset
        y = -foot_x * math.sin(angle) + foot_y * math.cos(angle)
        z = foot_z - self.HEIGHT_OFFSET

        return x, y, z

    def _set_servo(self, channel, angle):
        angle = self._clamp(angle, 0, 180)
        duty = (angle / 180.0) * 2000 + 500
        pwm_val = int(duty / 20000.0 * 4095)
        if channel < 16:
            self.pwm_41.set_pwm(channel, 0, pwm_val)
        else:
            self.pwm_40.set_pwm(channel - 16, 0, pwm_val)

    def _update_all_legs(self, debug=False):
        for i in range(6):
            foot_x, foot_y = self.FOOT_XY[i]
            foot_z = self.body_z

            lx, ly, lz = self._world_to_leg(foot_x, foot_y, foot_z, i)
            raw = self._ik(-lz, lx, ly)

            if i < 3:
                a0 = self._clamp(raw[0] + self.cal_angles[i][0], 0, 180)
                a1 = self._clamp(90 - (raw[1] + self.cal_angles[i][1]), 0, 180)
                a2 = self._clamp(raw[2] + self.cal_angles[i][2], 0, 180)
            else:
                a0 = self._clamp(raw[0] + self.cal_angles[i][0], 0, 180)
                a1 = self._clamp(90 + raw[1] + self.cal_angles[i][1], 0, 180)
                a2 = self._clamp(180 - (raw[2] + self.cal_angles[i][2]), 0, 180)

            if debug:
                print(f"Leg {i+1}: local=({lx:.1f},{ly:.1f},{lz:.1f}) raw={raw} -> coxa={a0}, femur={a1}, tibia={a2}")

            ch = self.LEG_CHANNELS[i]
            self._set_servo(ch[0], a0)
            self._set_servo(ch[1], a1)
            self._set_servo(ch[2], a2)

    def home(self):
        print("HOME (body_z = 0)...")
        self.body_z = 0.0
        self._update_all_legs(debug=True)
        print("HOME done")

    def stand(self, height=30.0, duration=1.0):
        print(f"STAND (body_z = -{height})...")
        steps = 50
        start_z = self.body_z
        end_z = -height

        for step in range(steps + 1):
            t = step / steps
            t = t * t * (3 - 2 * t)
            self.body_z = start_z + t * (end_z - start_z)
            self._update_all_legs()
            time.sleep(duration / steps)

        print(f"Final body_z = {self.body_z}")
        self._update_all_legs(debug=True)
        print("STAND done")

    def relax(self):
        for i in range(16):
            self.pwm_40.set_pwm_off(i)
            self.pwm_41.set_pwm_off(i)
        print("Relaxed")


def main():
    print("=" * 40)
    print("Body-Centric Initialize Test (DEBUG)")
    print("=" * 40)

    robot = HexapodInit()

    print("\n>>> HOME in 2 seconds...")
    time.sleep(2)
    robot.home()

    print("\n>>> STAND in 2 seconds...")
    time.sleep(2)
    robot.stand(30.0)

    print("\n>>> Holding for 3 seconds...")
    time.sleep(3)

    print("\n>>> RELAX...")
    robot.relax()

    print("\n=== TEST COMPLETE ===")


if __name__ == '__main__':
    main()
