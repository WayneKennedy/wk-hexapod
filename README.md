# wk-hexapod

**A ROS 2 autonomous hexapod on Freenove Big Hexapod hardware**, running natively on a
Raspberry Pi 5. It walks with an analytic inverse-kinematics tripod gait, maps with an
Intel RealSense D435i and RTAB-Map, navigates with Nav2, and explores frontiers on its own
unless an external mission planner gives it something better to do.

**Status:** the native stack is verified end to end on the bench (2026-09-09, USB power,
servos unpowered): drivers, SLAM producing a map, Nav2 active, exploration sending goals,
mission API answering. The first battery run of this stack is the next step. See
[`docs/roadmap.md`](docs/roadmap.md) for direction, [`docs/decisions.md`](docs/decisions.md)
for what is settled, and [`docs/open-questions.md`](docs/open-questions.md) for what is not.

## What it is

- **Body:** Freenove Big Hexapod (FNK0052) — 18 leg servos plus a 2-servo pan/tilt head,
  all hobby PWM servos on two PCA9685 drivers, driven straight off the Pi. There is no
  microcontroller: it is the family's counter-example to the two-tier compute rule, viable
  because a six-legged walker is statically stable.
- **Sensing:** RealSense D435i (RGB, depth, IMU) on the head, replacing the kit's Pi camera
  and ultrasonic sensor; MPU6050 body IMU; ADS7830 dual-battery ADC.
- **Software:** ROS 2 Jazzy on Ubuntu Server 24.04, all from apt. Six packages: hardware
  drivers, locomotion controller, interfaces, bring-up, perception, autonomy.
- **Behaviour:** boots, stands, maps or localizes, waits briefly for a mission, then
  explores. Missions arrive over an HTTP API from any machine with `curl`.

## Documents

| File | Contents |
|---|---|
| [`AGENTS.md`](AGENTS.md) | Onboarding for any AI assistant or contributor — **start here** |
| [`docs/concept.md`](docs/concept.md) | What it is, what it is for, and the history |
| [`docs/architecture.md`](docs/architecture.md) | Buses, nodes, topics, frames; the tier model |
| [`docs/hardware.md`](docs/hardware.md) | The kit, the sensor swap, pin map, power, calibration, the buzzer hazard |
| [`docs/operations.md`](docs/operations.md) | Install, run, service, maps, remote missions, tests |
| [`docs/decisions.md`](docs/decisions.md) | Banked decisions — the durable *why* |
| [`docs/open-questions.md`](docs/open-questions.md) | Unresolved. Never state these as settled |
| [`docs/roadmap.md`](docs/roadmap.md) | Milestones |
| [`docs/references.md`](docs/references.md) | Upstream code, vendor reference, versions |
| [`docs/test-log.md`](docs/test-log.md) | What was actually run |

## Quick start

```bash
sudo ./scripts/ubuntu-setup.sh     # ROS 2 Jazzy, Nav2, RTAB-Map, RealSense, Python deps, build
sudo reboot
./systemd/install.sh               # buzzer guard + auto-start on boot
scripts/mission.sh -H <robot> state
```

Details, including what runs at boot and how to save a map, are in
[`docs/operations.md`](docs/operations.md).

## Family

This robot is one of several. The index, and everything true of more than one of them —
the compute tiers, the topic contract, perception placement, power integrity — is in
[wk-robotics](https://github.com/WayneKennedy/wk-robotics). Facts that belong there are
linked, never copied. The kit's vendor code lives in
[fn-hexapod](https://github.com/WayneKennedy/fn-hexapod).

## Licence

Apache 2.0 ([`LICENSE`](LICENSE)). This repository contains software and documentation
only; the hardware is Freenove's. See DEC-15.
