# 2026-06-28 — Add TUI generate button

**Type:** feat

**Scope:** legwheel

**Branch / Context:** feat/toroidal-tire-geometry

## Changes

- Added a focusable `Generate` action row to the LegWheel CSV TUI in Gait, Lean, and Transform modes.
- Pressing Enter, Space, or `l` on the focused Generate row now starts CSV generation, while F5 / Ctrl+G remain supported.
- Added focused tests for the Generate action row and adjusted the existing TUI duty-forwarding regression test to load the example module by file path.
- Smoke-tested the TUI launch in a pseudo-terminal and confirmed the Generate row renders in the Gait mode field list.

## Decisions

- Kept the change keyboard-driven inside the existing prompt_toolkit layout rather than introducing mouse-only controls or a new UI dependency.

## Tracking impact

- DEVLOG: fragment records the LegWheel TUI usability feature.
- ROADMAP: fragment records completion of the requested Generate-button improvement.
- Research log: fragment records the project-level code/test update.

## Next steps

- [LOW] Optionally add mouse click support for the Generate action if terminal mouse interaction becomes desired.
