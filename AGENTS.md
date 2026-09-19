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
natively on the robot's Raspberry Pi 5 under Ubuntu Server 24.04. The goal is autonomous
exploration and mapping of the local area, with missions from an external planner taking
priority over exploration. It is the family's **baseline intent-tier reference**, and its
hardware ceiling is the kit's: no accelerator, no bus servos (DEC-26). Full intent:
[`docs/concept.md`](docs/concept.md).

**Its sensing changed on 2026-09-19** (DEC-25): the Intel RealSense D435i went to the family's
Orin Nano and the kit's OV5647 camera and HC-SR04 ultrasonic went back on the pan/tilt head.
So there is **no depth and no SLAM**: one 15° sonar cone on a servo is the only range
sensor, the head does the looking instead of the body (DEC-27), and the map is Nav2's
sonar-fed global costmap anchored to odometry, lasting one run (DEC-28). Read those three
decisions before changing anything in perception or navigation.

**It is a working robot, not a design.** Locomotion, odometry, Nav2 and frontier
exploration run end to end ([`docs/test-log.md`](docs/test-log.md)). On 2026-09-15, after
a day of floor fixes (DEC-21, DEC-22), **the robot reached a frontier goal autonomously on
the battery for the first time** — with the D435i it no longer has.

## Where things live

- [`docs/concept.md`](docs/concept.md) — what it is, what it is for, and its history.
- [`docs/architecture.md`](docs/architecture.md) — every bus, node, topic and frame;
  where the hexapod sits in the family's tier model.
- [`docs/hardware.md`](docs/hardware.md) — the kit, the head sensors, GPIO and I2C map,
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
([OQ-15](docs/open-questions.md)).

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
- **Check what is owned before suggesting a purchase.** Read the private
  [wk-inventory `docs/stock.md`](https://github.com/WayneKennedy/wk-inventory/blob/main/docs/stock.md)
  and search the owner's invoices, and say what was found. Full rule and the owner's goal
  (fewer unused parts, more finished projects):
  [wk-inventory `AGENTS.md`](https://github.com/WayneKennedy/wk-inventory/blob/main/AGENTS.md#before-anything-is-bought).

## Working on the robot itself

**No assistant runs on the robot** ([DEC-29](docs/decisions.md), owner, 2026-09-21).
Sessions run on the always-on workstation and operate the robot over SSH, as for the
family's other ROS 2 hosts. Author code in the workstation's checkout and push it. Then
update the robot's checkout from `origin/main` and test over SSH: the
[development loop](docs/operations.md#development-loop). A change that moves a leg needs
the owner present (below).

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
  not. Anything that moves a leg **or the head** needs the battery and the owner present,
  which is why the head's calibration is still open ([OQ-21](docs/open-questions.md)).

## Status

**Sonar stack verified on the bench, 2026-09-19, USB power.** The whole chain ran
unattended: head survey → sonar map → frontier goal, with Nav2 active and the dashboard
answering ([`docs/test-log.md`](docs/test-log.md)). Nav2 reported "failed to make
progress" because the servos are dead on USB power. **The head servos could not move
either, so the map's geometry is unverified** — the pipeline is proven, the map is not.
The camera does not probe ([OQ-23](docs/open-questions.md)); bench runs use
`camera:=false`. Load average 19–23 on four cores with everything up
([OQ-02](docs/open-questions.md)).

**Frontier:** the head calibration in `docs/operations.md` (pan sign, limits, slew rate —
battery and owner needed), then a survey that matches a real room, then the collision
monitor against real obstacles ([`docs/roadmap.md`](docs/roadmap.md)). Deciding how the
robot will know where it is without SLAM ([OQ-20](docs/open-questions.md)) blocks
milestone 2.
