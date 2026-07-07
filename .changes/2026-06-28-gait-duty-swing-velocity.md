# 2026-06-28 — Gait duty override and swing touchdown velocity

**Type:** fix

**Scope:** gait

**Branch / Context:** feat/toroidal-tire-geometry

## Changes

- Added optional stance-duty override support through `GaitGenerator3D`, `LaunchController`, hardware CSV generation, CLI, and TUI paths.
- Preserved the remote Walk CoM stability margin option while merging the duty override flow.
- Fixed swing touchdown horizontal velocity to follow the planned body-frame swing displacement instead of using the opposite stance-contact velocity.
- Added regression coverage for custom gait duty forwarding and swing touchdown velocity direction.

## Decisions

- Kept `--stab-margin` and `--duty` as independent gait options because Walk stability and duty override control different aspects of the trajectory.

## Tracking impact

- DEVLOG: fragment records the gait duty and swing touchdown fix.
- ROADMAP: fragment records completion of the gait override integration.
- Research log: fragment records the project-level code/test update.

## Next steps

- [LOW] Consider adding CLI/TUI smoke coverage for combined `--stab-margin` and `--duty` usage.
