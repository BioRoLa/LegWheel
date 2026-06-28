# 2026-06-28 — CSV generation benchmark and stance C-port plan

**Type:** chore

**Scope:** performance

**Branch / Context:** feat/toroidal-tire-geometry

## Changes

- Added a repeatable hardware CSV generation benchmark script with optional cProfile capture.
- Added a stance solver C-port plan focused on the actual CSV-generation hotspots: contact search, rolling FK, numerical Jacobian, and stance loop batching.
- Recorded benchmark guidance showing that micro-kernel C ports alone are unlikely to accelerate end-to-end CSV generation.

## Decisions

- Do not directly merge `feature/c-based-benchmark`; selectively port safe C backend pieces only after protecting current toroidal tire behavior.
- Prioritize reducing repeated FK/contact calls before attempting a full C implementation.

## Tracking impact

- DEVLOG: fragment records the performance investigation tooling and plan.
- ROADMAP: fragment records the proposed staged acceleration path.
- Research log: fragment records the CSV generation profiling direction.

## Next steps

- [HIGH] Use `examples/gait/benchmark_csv_generation.py` as the baseline before optimizing `foot_rim_contact_fk()` or `stance_rt_solver()`.
- [MED] Prototype a Python-only contact-search reduction before C-porting the stance loop.
