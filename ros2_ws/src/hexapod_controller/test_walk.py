#!/usr/bin/env python3
"""
Test: Walk forward 150mm, then backward 150mm
Uses body-centric controller with tripod gait.
"""

import math
import time
import smbus


class PCA9685:
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


class HexapodWalk:
    LEG_CHANNELS = [
        [15, 14, 13], [12, 11, 10], [9, 8, 31],
        [22, 23, 27], [19, 20, 21], [16, 17, 18],
    ]

    L1, L2, L3 = 33.0, 90.0, 110.0
    LEG_ANGLES = [54.0, 0.0, -54.0, -126.0, 180.0, 126.0]
    LEG_OFFSETS = [94.0, 85.0, 94.0, 94.0, 85.0, 94.0]
    HEIGHT_OFFSET = 14.0

    FOOT_XY = [
        (137.1, 189.4), (225.0, 0.0), (137.1, -189.4),
        (-137.1, -189.4), (-225.0, 0.0), (-137.1, 189.4),
    ]

    def __init__(self):
        print("Initializing...")
        self.pwm_41 = PCA9685(1, 0x41)
        self.pwm_41.set_pwm_freq(50)
        self.pwm_40 = PCA9685(1, 0x40)
        self.pwm_40.set_pwm_freq(50)
        time.sleep(0.1)

        self.body_z = 0.0
        # Foot positions [x, y, z] - will be modified during gait
        self.feet = [[fx, fy, 0.0] for fx, fy in self.FOOT_XY]

        self.calibration = self._load_calibration()
        self.cal_angles = self._calc_calibration()
        print("Ready")

    def _load_calibration(self):
        paths = [
            '/home/wkenn/Code/wk-hexapod/ros2_ws/src/hexapod_hardware/config/servo_calibration.txt',
            '/home/wkenn/Code/fn-hexapod/Code/Server/point.txt',
        ]
        for path in paths:
            try:
                with open(path, 'r') as f:
                    return [list(map(int, line.strip().split('\t'))) for line in f if line.strip()]
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

    def _update_servos(self):
        for i in range(6):
            lx, ly, lz = self._world_to_leg(self.feet[i][0], self.feet[i][1], self.feet[i][2], i)
            raw = self._ik(-lz, lx, ly)
            if i < 3:
                a0 = self._clamp(raw[0] + self.cal_angles[i][0], 0, 180)
                a1 = self._clamp(90 - (raw[1] + self.cal_angles[i][1]), 0, 180)
                a2 = self._clamp(raw[2] + self.cal_angles[i][2], 0, 180)
            else:
                a0 = self._clamp(raw[0] + self.cal_angles[i][0], 0, 180)
                a1 = self._clamp(90 + raw[1] + self.cal_angles[i][1], 0, 180)
                a2 = self._clamp(180 - (raw[2] + self.cal_angles[i][2]), 0, 180)
            ch = self.LEG_CHANNELS[i]
            self._set_servo(ch[0], a0)
            self._set_servo(ch[1], a1)
            self._set_servo(ch[2], a2)

    def home(self):
        print("HOME...")
        self.body_z = 0.0
        for i, (fx, fy) in enumerate(self.FOOT_XY):
            self.feet[i] = [fx, fy, self.body_z]
        self._update_servos()

    def stand(self, height=30.0, duration=1.0):
        print(f"STAND ({height}mm)...")
        steps = 50
        start_z = self.body_z
        end_z = -height
        for step in range(steps + 1):
            t = step / steps
            t = t * t * (3 - 2 * t)
            self.body_z = start_z + t * (end_z - start_z)
            for i, (fx, fy) in enumerate(self.FOOT_XY):
                self.feet[i] = [fx, fy, self.body_z]
            self._update_servos()
            time.sleep(duration / steps)

    def walk(self, distance_mm, step_size=25, step_height=40.0, cycle_time=0.8):
        direction = "FORWARD" if distance_mm > 0 else "BACKWARD"
        print(f"WALK {direction} {abs(distance_mm)}mm...")

        cycles = max(1, int(abs(distance_mm) / step_size))
        x_step = step_size if distance_mm > 0 else -step_size

        for cycle in range(cycles):
            print(f"  Step {cycle+1}/{cycles}")
            self._tripod_cycle(x_step, step_height, cycle_time)

        # Reset to neutral stance
        for i, (fx, fy) in enumerate(self.FOOT_XY):
            self.feet[i] = [fx, fy, self.body_z]
        self._update_servos()
        print(f"WALK {direction} done")

    def _tripod_cycle(self, y_move, step_height, cycle_time):
        """One tripod gait cycle - even legs (0,2,4) and odd legs (1,3,5) alternate
        Y axis is forward/back in world frame.
        """
        frames = 64
        delay = cycle_time / frames

        # Store starting positions
        start_feet = [f[:] for f in self.feet]

        for frame in range(frames):
            phase = frame / frames

            for i in range(6):
                is_odd = (i % 2 == 1)

                # Odd legs move first half, even legs move second half
                if is_odd:
                    leg_phase = phase * 2 if phase < 0.5 else 1.0
                else:
                    leg_phase = 0.0 if phase < 0.5 else (phase - 0.5) * 2

                # Calculate foot position (Y is forward/back)
                if leg_phase < 1.0:
                    # Swing phase - leg in air moving forward
                    swing = math.sin(leg_phase * math.pi)
                    self.feet[i][1] = start_feet[i][1] + y_move * (leg_phase - 0.5)
                    self.feet[i][2] = self.body_z + step_height * swing
                else:
                    # Stance complete
                    self.feet[i][1] = start_feet[i][1] + y_move * 0.5
                    self.feet[i][2] = self.body_z

                # Push phase for grounded legs
                if (is_odd and phase >= 0.5) or (not is_odd and phase < 0.5):
                    push_phase = (phase - 0.5) if is_odd else phase
                    self.feet[i][1] = start_feet[i][1] - y_move * push_phase * 2

            self._update_servos()
            time.sleep(delay)

        # Update start positions for next cycle
        for i, (fx, fy) in enumerate(self.FOOT_XY):
            self.feet[i] = [fx, fy, self.body_z]

    def relax(self):
        for i in range(16):
            self.pwm_40.set_pwm_off(i)
            self.pwm_41.set_pwm_off(i)
        print("Relaxed")


def main():
    print("=" * 40)
    print("Walk Test: Forward 150mm, Back 150mm")
    print("=" * 40)

    robot = HexapodWalk()

    print("\n>>> HOME in 2s...")
    time.sleep(2)
    robot.home()

    print("\n>>> STAND in 2s...")
    time.sleep(2)
    robot.stand(30.0)
    time.sleep(1)

    print("\n>>> WALK FORWARD 150mm...")
    robot.walk(150)
    time.sleep(1)

    print("\n>>> WALK BACKWARD 150mm...")
    robot.walk(-150)
    time.sleep(1)

    print("\n>>> HOME before relax...")
    robot.home()
    time.sleep(1)

    print("\n>>> RELAX...")
    robot.relax()

    print("\n=== TEST COMPLETE ===")


if __name__ == '__main__':
    main()
