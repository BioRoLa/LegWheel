# 2026-07-07 — Lateral trot height fix and pose planner feature set

**Type:** fix + feat

**Scope:** trot, pose

**Branch / Context:** fix/swing-touchdown-velocity

## Changes

### fix(trot): gamma-first initial pose for lateral gait (trajectory_planning_3d.py)

- Rewrote `_calculate_initial_pose()` ordering: lateral ABAD angles
  (`gamma0`, `gamma_td`) now solved **before** sagittal angles
  (`beta0`, `theta0`).
- Added `H_O_eff` correction — when `gamma0 ≠ 0`, the wheel contacts at
  its inner edge rather than the arc centre; effective sagittal drop
  height is recomputed as:
  `H_O_eff = (H_hip + sin(γ₀)·d_eff) / cos(γ₀) − R_arc`
  where `d_eff = gamma_sign · d_wheel − half_w`.
- Fixes four-leg touchdown height mismatch seen during lateral trot.
- Preserves the one-sided gamma sweep logic (`gamma_td` / `gamma_floor`)
  already present in this repo version.

### feat(pose): XYZ lean, workspace query, rock mode, ramp profile (pose_planner.py + CLI + TUI)

**New pose_planner.py capabilities:**
- `solve_pose()` and `plan_lean()` accept `x_offset` / `y_offset` for
  body XY translation (no leg lifting, feet fixed in world).
- `plan_lean(rock=True)`: each cycle oscillates `neutral → +target →
  neutral → −target → neutral`; RPY and XY offsets are negated for the
  minus phase; height unchanged.
- `_is_feasible()`: silent IK probe, returns bool.
- `compute_workspace(height)`: binary-searches each DOF (roll, pitch,
  yaw, x, y) independently; returns `{dof: (min, max)}` dict.
- `print_workspace()`: formatted table output.
- `_profile_map(ts, profile, ramp_ratio)`: maps linear `t ∈ [0,1]` to:
  - `'cosine'` (default) — sinusoidal velocity, zero at endpoints
  - `'trapezoid'` — constant-accel ramp + cruise + ramp-down, zero at endpoints
  - `'linear'` — original constant-velocity behaviour
- `plan_sequence()` and `plan_lean()` gain `profile` and `ramp_ratio`
  parameters; cosine is now the default, eliminating velocity spikes at
  waypoint direction changes (inertia-driven tip-over risk).

**CLI (generate_lean_csv.py):**
- `--x`, `--y` body translation
- `--rock` periodic ± oscillation
- `--check-workspace` — print workspace table and exit
- `--profile {cosine,trapezoid,linear}` and `--ramp-ratio`

**TUI (generate_csv_tui.py):**
- Lean form: X offset (m), Y offset (m), Rock ± fields
- F6 keybinding: workspace check in background thread, results in log
- Profile (choice cycle) and Ramp Ratio fields
- Summary panel shows profile and ramp ratio

## Decisions

- `profile='cosine'` chosen as default over trapezoid — smoother
  (no jerk discontinuity at accel/cruise boundary) and simpler.
  `'linear'` kept for backward compatibility.
- Workspace binary search is per-DOF independently (not joint); accurate
  enough for planning guard checks and takes < 1 s.
- `rock` negates RPY + XY but keeps height the same to prevent the
  body from varying height during symmetric oscillation.

## Tracking impact

- DEVLOG: gamma axis reversal diagnostic (roll ≈ 0.82 × γ correlation
  traced to reversed gamma motor axis in sim URDF — fixed on sim side).
- ROADMAP: pose planner XYZ lean and workspace query marked complete.

## Verification

- `plan_lean(profile='cosine')` and `plan_lean(profile='trapezoid')`
  produce correct (N, 12) trajectories; waypoint-junction velocity is
  ≈ 45 % of segment peak vs 100 % for linear (measured via finite
  difference on joint-angle output).
- `--check-workspace` exits cleanly after printing table.
- TUI lean fields (Profile, Ramp Ratio, X/Y offset, Rock, F6) render
  and cycle correctly.
