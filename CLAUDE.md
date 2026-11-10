# Claude Code

> **The project record moved to Basic Memory on 2026-11-10** (project `main`, folder `robotics/hexapod/`; start at the
> *Hexapod* note). `README.md` and `docs/` — concept, architecture, decisions, open questions, roadmap, hardware,
> test log, references, operations — are canonical there; the copies here were deleted on 2026-11-10 (git history keeps them). This repo keeps what is about the code
> and artefacts: `AGENTS.md`/`CLAUDE.md`/`GEMINI.md`, `ros2_ws/`, `scripts/`, `systemd/`, `boot/`, `requirements.txt`.


The instructions for this repository are provider-neutral and live in `AGENTS.md`. Read it
in full — it is the whole onboarding.

@AGENTS.md

## Claude-specific

- Claude's auto-memory holds **pointers only** — never a project fact, measurement or
  decision. See *Working conventions* in `AGENTS.md`.
- Prefer prose and a led recommendation over an `AskUserQuestion` poll.
