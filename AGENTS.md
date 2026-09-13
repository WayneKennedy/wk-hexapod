# wk-hexapod — agent / contributor onboarding

**Read this first.** It is the entry point for any AI assistant or human working in
this repository, and it is written to be complete on a first read with no prior
context. It is **provider-neutral**: `CLAUDE.md` and `GEMINI.md` do nothing but
point here.

## The 4Cs — the standard every artefact meets

Every artefact — docs, source, config, commit messages — must be:

1. **Correct** — fact-based. No speculation unless labelled as such. "Unknown" and
   "unverified" are valid answers; confident guesses are not.
2. **Complete** — nothing essential missing.
3. **Coherent** — everything fits together; no contradictions.
4. **Concise** — nothing superfluous.

All four hold at once: completeness never excuses bloat; brevity never excuses gaps;
and none of the other three count if the content is wrong.

## What this project is

**A ROS 2 Jazzy autonomous hexapod on Freenove Big Hexapod (FNK0052) hardware**, running
natively on the robot's Raspberry Pi 5 under Ubuntu Server 24.04. The kit's Pi camera and
ultrasonic sensor are replaced by an Intel RealSense D435i. The goal is autonomous
exploration and mapping of the local area, with missions from an external planner taking
priority over exploration. Full intent: [`docs/concept.md`](docs/concept.md).

**It is a working robot, not a design.** Locomotion, odometry, SLAM, Nav2 and frontier
exploration all run end to end on the bench ([`docs/test-log.md`](docs/test-log.md)).
The first battery-powered run of the native stack has not happened yet.

## Where things live

- [`docs/concept.md`](docs/concept.md) — what it is, what it is for, and its history.
- [`docs/architecture.md`](docs/architecture.md) — every bus, node, topic and frame;
  where the hexapod sits in the family's tier model.
- [`docs/hardware.md`](docs/hardware.md) — the kit, the sensor swap, GPIO and I2C map,
  power modes, calibration, **the buzzer hazard**.
- [`docs/operations.md`](docs/operations.md) — install, run, service, maps, remote
  missions, tests, troubleshooting.
- [`docs/decisions.md`](docs/decisions.md) — **banked decisions** (the durable *why*).
- [`docs/open-questions.md`](docs/open-questions.md) — **pending decisions**.
- [`docs/roadmap.md`](docs/roadmap.md) — milestones.
- [`docs/references.md`](docs/references.md) — upstream code, the vendor reference,
  package versions.
- [`docs/test-log.md`](docs/test-log.md) — what was actually run, when, with what result.
- `ros2_ws/src/` — the six ROS 2 packages; `scripts/` and `systemd/` — install, launch,
  service units.

## This repo does not stand alone in one respect

Facts true of **more than one** robot live in
[wk-robotics](https://github.com/WayneKennedy/wk-robotics) — the compute tiers, the
topic contract, perception placement, power integrity, the GPU workstation, licensing
conventions. **Link to them; never copy them.** Facts true of *this* robot alone live here
and nowhere else.

The kit's vendor code is Freenove's upstream repository, expected as a sparse sibling
checkout at `../freenove-hexapod` (the clone recipe and pinned commit are in
[`docs/references.md`](docs/references.md)). Its `Code/Server/` files are the
confirmed-working reference for servo, home, stand and gait behaviour. When touching a
hardware driver, read the reference first; do not reinvent. The vendor code is
CC BY-NC-SA 3.0 and this repo is Apache-2.0, so **read and re-derive, do not copy**
([OQ-14](docs/open-questions.md)).

## Hazard: the buzzer

The shield buzzer on **GPIO 17** is painfully loud and sounds whenever the pin floats —
which is the state after **any** process that claimed it exits or is killed. It has
distressed people in the house. Rules, all of which are enforced by the setup
([`docs/hardware.md`](docs/hardware.md#the-buzzer-hazard)):

- `hexapod-buzzer-guard.service` holds GPIO 17 low at all times. Leave it running.
- `buzzer.enabled` in `hexapod_hardware/config/hardware.yaml` stays `false` unless the
  owner decides otherwise.
- Never claim GPIO 17 from an ad-hoc script and never kill a process holding a GPIO line.
- Never read the pin with `gpioget`; it reconfigures the line as an input.

## Working conventions

- **No project fact lives only in chat.** Record durable decisions in `decisions.md`;
  put anything unresolved in `open-questions.md`. Move items between them as they resolve.
- **Distinguish decided from open.** Never state an open question as settled.
- **Verified beats plausible.** Measured numbers carry the date and the conditions.
  `test-log.md` records what was actually run, including negative results.
- **Docs are written AI-first** — dense, factual, cross-referenced, greppable.
- **This repository is public.** No credentials, host names, network addresses or
  overlay-network identifiers. The robot is "the robot"; reach it through the
  `HEXAPOD_HOST` variable described in `docs/operations.md`.
- **The repository is the memory.** Per-harness memory holds pointers only.

## Working on the robot itself

- ROS is native: `source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash`,
  or use `scripts/launch.sh`. The stack normally runs under `systemd` (`hexapod.service`);
  stop it before launching anything by hand.
- Python nodes are symlink-installed: edits take effect on restart. Rebuild after changing
  `hexapod_interfaces`, any `setup.py`, or any file listed in a package's `data_files`.
- DDS discovery takes about 10 s. `ros2 topic list --no-daemon` right after a launch is
  empty or partial; use the daemon and wait.
- `pkill -f <pattern>` kills the calling shell if the pattern appears in its own command
  line. Kill by PID.
- Never add the Raspberry Pi OS (bookworm) apt repository to this host
  ([`docs/operations.md`](docs/operations.md#foreign-packages)).
- The robot is usually on USB power during development: sensors and LEDs work, servos do
  not. Anything that moves a leg needs the battery and the owner present.

## Status

**Native stack verified on the bench, 2026-09-09, USB power.** All 36 nodes run under the
service: drivers, startup sequence, RealSense, RTAB-Map in mapping mode producing an
occupancy grid, Nav2 active, frontier exploration sending goals, dashboard mission API
answering. Nav2 reports "failed to make progress" because the servos are unpowered and
because the controller's velocity scaling does not match Nav2's commands
([OQ-01](docs/open-questions.md)). The Pi runs at a load average around 10 with
everything up ([OQ-02](docs/open-questions.md)).

**Frontier:** the first battery run of the native stack — velocity scaling, collision
monitor and costmap tuning on the floor ([`docs/roadmap.md`](docs/roadmap.md)).
