# Roadmap

Each milestone ends with a robot that does something it could not do before, verified on
the hardware and recorded in [`test-log.md`](test-log.md). Direction, not commitment:
the owner's stated goal is autonomous exploration with an external planner able to
override it ([`concept.md`](concept.md#what-it-is-for)).

## Milestone 0 — Native stack runs end to end *(done 2026-09-09, on the bench)*

Docker gone, every package from apt, all six ROS packages building, the whole boot stack
under `systemd`: drivers, startup sequence, RealSense, RTAB-Map mapping, Nav2 active,
frontier exploration sending goals, mission API answering. Verified on USB power with the
servos unpowered.

**Exit met:** 36 nodes up, a 5 cm occupancy grid produced from the camera, Nav2 planning
paths to frontiers. Not met on purpose: nothing walked.

## Milestone 1 — It explores a room *(current, restarted on the lidar)*

The movement calibration was done on 2026-09-15 and the robot reached a frontier on the
battery. The sensor changes of 2026-09-19 (DEC-25) and 2026-09-29 (DEC-32) put the
perception half back to the start twice: everything below is on the lidar, with the head
disabled (OQ-30).

1. The I2C bus answers again (OQ-32).
2. A frontier reached on the lidar, in open floor (OQ-34).
3. The lidar does not see the robot itself while it walks (OQ-31).
4. Tune the collision monitor and costmaps against real obstacles (OQ-03).
5. Verify the IMU filter conventions with the robot turning (OQ-13).
6. Bring the CPU load down to where the loops keep their rates (OQ-02).
7. The dashboard shows what is true (OQ-33).

**Exit:** the robot maps a room unattended, on the lidar, and Nav2 reaches frontiers in it.

## Milestone 2 — It localizes and takes missions

The route is the lidar (DEC-32). What is missing is a map that survives the run:
`slam_toolbox` can serialize a map and localize against it, and nothing here does yet
(OQ-20).

1. Save a map and localize against it on the next boot; then: boot, know where you are,
   wait for a mission.
2. `navigate` and `return_home` missions from a remote machine via `scripts/mission.sh`.
3. Decide what an approved planner is and how it authenticates (OQ-04).

**Exit:** from another machine, send the robot to a map coordinate and bring it home.

## Milestone 3 — It looks after itself

1. Battery-aware behaviour: return home and relax below a threshold (OQ-11).
2. Wander mode: explore, return, repeat, without a mission.
3. Named places ("kitchen") as mission targets (OQ-05).

**Exit:** it wanders for a battery's worth, comes home, and stops safely.

## Milestone 4 — A fleet node

The family's [hive-mind direction](https://github.com/WayneKennedy/wk-robotics/blob/main/docs/ideas.md#physical-ai-and-the-hive-mind):
this robot and [wk-devastator](https://github.com/WayneKennedy/wk-robotics/tree/main/projects/devastator) on one
topic contract, reporting to an off-robot planner, and both still useful when it is
unreachable. Depends on that robot existing, and on the transport question the family has
not decided (Zenoh or DDS).

## Not on the roadmap

- A reflex-tier MCU (OQ-09) — argued for by the family pattern, not by any failure yet
  seen on this robot that cannot be fixed in software.
- Face recognition as a goal (OQ-06).
