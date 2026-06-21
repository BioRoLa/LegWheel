# LegWheel Examples

This folder is the entry point for runnable examples.
Use this page as the clean index instead of scanning every script manually.

## Quick Start

From `LegWheel/`:

```bash
uv run python examples/kinematics/basic_usage.py
uv run python examples/gait/test_full_gait_cycle.py
uv run python examples/self_righting/plot_window_states.py
uv run python examples/self_righting/boundary_states/beta_0_90_comparison.py
uv run python examples/stability/demo_stability_definition.py
```

## Folder Map

- `kinematics/`: FK/IK checks, transform validation, geometry sanity checks.
- `gait/`: trajectory generation, gait cycle tests, CSV export helpers.
- `self_righting/`: recovery phases, contact geometry, gap analysis, window plots.
- `self_righting/boundary_states/`: categorized boundary-state rendering scripts.
- `stability/`: stability metric demos and planner-oriented tests.

## Recommended Scripts

If you only want the core demos, start with these:

1. `examples/kinematics/basic_usage.py`
2. `examples/gait/test_full_gait_cycle.py`
3. `examples/self_righting/plot_window_states.py`
4. `examples/stability/demo_stability_definition.py`
5. `examples/self_righting/boundary_states/beta_0_90_comparison.py`

## Self-Righting Script Groups

To keep the `self_righting/` page readable, use this grouping:

- Boundary-state visualization (new classified layer):
  - `boundary_states/render_boundary_sequence.py`
  - `boundary_states/beta_0_90_comparison.py`
  - `plot_window_states.py` (backward-compatible wrapper)
  - `plot_recovery_strategy.py`
  - `stability_analysis.py`

- Planning and scans:
  - `quasi_static_path_planner.py`
  - `quasi_static_gap_analysis.py`
  - `s4_support_gamma_scan.py`
  - `independent_gamma_scan.py`
  - `extended_gamma_analysis.py`

- Contact and geometry diagnostics:
  - `test_self_righting_collision.py`
  - `plot_collision_model.py`
  - `debug_contact_geometry.py`
  - `check_upper_reach.py`
  - `check_upper_reach_stability.py`

- Verification snapshots (reference scripts):
  - `verify_all_same.py`
  - `verify_orig_scan.py`
  - `verify_phase3.py`
  - `verify_phase3_fixed.py`
  - `verify_phase3_minimal.py`
  - `verify_phase3_quick.py`

## Optional Plotly Viewer

The first Plotly-based viewer lives under `render/` because it renders the full robot rather
than a single example trajectory:

```bash
uv sync --extra plotly
uv run python render/plotly_corgi_robot.py \
  --theta 75 --beta 0 --gamma 0 \
  --html outputs/plotly/corgi_robot.html
```

This is an optional interactive HTML backend. Existing Matplotlib examples remain available.

## Notes

- The current structure intentionally keeps script paths stable.
- This index cleans navigation without breaking imports or historical command usage.
