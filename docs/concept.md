# Concept

What the hexapod is, what it is for, and how it got here.

## What it is

A Freenove Big Hexapod kit (FNK0052) whose vendor Python stack has been replaced by a
ROS 2 Jazzy stack running natively on the kit's Raspberry Pi 5. The mechanics, servos,
shield electronics and body IMU are the kit's. The head sensing is not: an Intel RealSense
D435i replaces the Pi camera and the HC-SR04 ultrasonic sensor, giving RGB, depth and a
second IMU in one USB 3 device ([`hardware.md`](hardware.md)).

Everything on the robot hangs directly off the Pi: servos on two I2C PWM chips, IMU and
battery ADC on I2C, LEDs on SPI, buzzer and servo-power enable on GPIO, the camera on USB.
There is no microcontroller and therefore no reflex tier
([`architecture.md`](architecture.md#where-it-sits-in-the-family)).

## What it is for

1. **Autonomous exploration and mapping of the local area.** The robot boots, stands,
   builds or reloads a map with RTAB-Map, and explores frontiers with Nav2 on its own.
2. **Missions from an approved external planner take priority.** When a planner is
   reachable it directs the robot; when it is not, the robot falls back to exploring. Today
   the planner interface is an HTTP API on the robot driven from another machine
   ([`operations.md`](operations.md#remote-missions)); who is "approved" is
   [OQ-04](open-questions.md).
3. **Wander mode with return-to-home** — not yet built ([`roadmap.md`](roadmap.md)).

Within the family it is the **intent-tier reference**: the only robot with a working SLAM,
navigation and mission stack, and the first candidate node for the fleet direction recorded
in [wk-robotics](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/ideas.md#physical-ai-and-the-hive-mind).
[wk-devastator](https://github.com/WayneKennedy/wk-robotics/tree/main/projects/devastator) is the intended second
consumer of its navigation configuration.

## History

- **2025-12-29 → 12-31.** Repository created from the vendor code; ROS 2 Jazzy in a Docker
  container on the Pi; hardware drivers, then the body-centric IK controller, tripod gait,
  odometry with IMU yaw fusion, a `MoveDistance` action, URDF, Nav2 and SLAM configuration,
  and the RealSense swap — all in three days.
- **2026-01 → 03.** RTAB-Map, a safe startup sequence, LED power indicator, a web dashboard,
  face recognition, and the `hexapod_autonomy` package (state machine, frontier explorer,
  look-around, mission server). The container image was last built on 2025-12-31 and never
  rebuilt, so none of the SLAM, Nav2 or autonomy work ever ran on the robot.
- **2026-09-09.** Docker removed; everything installed natively from apt (DEC-07). The
  native bring-up exposed and fixed a chain of defects that the stale container had hidden
  — a servo-power GPIO claimed twice, calibration paths that only existed in the container,
  a startup sequence that could not run on current rclpy, asyncio sleeps inside rclpy
  coroutines, RTAB-Map forced into localization against an empty database, and a Nav2
  configuration written for an older release. Two incidents with the shield buzzer during
  the same session drove DEC-10. By the end of the day the full stack ran end to end on
  the bench ([`test-log.md`](test-log.md)).
