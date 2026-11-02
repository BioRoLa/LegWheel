# Self-Righting Examples

```
self_righting/
├── plot_window_states.py          ← legacy entry-point (delegates to boundary_states/)
├── boundary_states/               ← ordered S0-S4 rendering + beta comparison
├── analysis/                      ← stability analysis, gamma scans, gap analysis
├── viz/                           ← collision model and recovery strategy plots
├── planning/                      ← quasi-static path planner
└── verify/                        ← verification and debug scripts
```

---

## boundary_states/

| Script | Purpose |
|--------|---------|
| `render_boundary_sequence.py` | Unified one-shot renderer for S4→S3→S2→S1→S0. Indexed output filenames for sorting. `{B}` frame axes included. |
| `beta_0_90_comparison.py` | beta=0 (cone-on-ground criterion) vs beta=90 (grid search) over roll sweep. |

```bash
uv run python examples/self_righting/boundary_states/render_boundary_sequence.py --no-show
uv run python examples/self_righting/boundary_states/beta_0_90_comparison.py --no-show
uv run python examples/self_righting/plot_window_states.py --no-show   # legacy alias
```

---

## analysis/

Stability margin analysis, ABAD gamma scans, gap crossing analysis, and upper-reach checks.

| Script | Purpose |
|--------|---------|
| `stability_analysis.py` | General stability margin analysis |
| `s4_support_gamma_scan.py` | Gamma scan for S4 (upside-down) support polygon |
| `extended_gamma_analysis.py` | Extended ABAD gamma analysis |
| `independent_gamma_scan.py` | Per-leg independent gamma sweep |
| `quasi_static_gap_analysis.py` | Gap-crossing quasi-static analysis |
| `gap1_crossing_analysis.py` | Analysis of gap-1 transitions |
| `floating_leg_com_analysis.py` | CoM analysis with floating leg |
| `full_scan_all_same.py` | Full scan with symmetric leg configs |
| `check_upper_reach.py` | Can upper legs reach ground at high roll? |
| `check_upper_reach_stability.py` | Stability with upper-leg ground contact |

---

## viz/

| Script | Purpose |
|--------|---------|
| `plot_collision_model.py` | 3D visualisation of the collision model |
| `plot_recovery_strategy.py` | Recovery strategy animation/plot |

---

## planning/

| Script | Purpose |
|--------|---------|
| `quasi_static_path_planner.py` | Quasi-static path planner for self-righting |

---

## verify/

Quick sanity checks and regression tests.

| Script | Purpose |
|--------|---------|
| `verify_phase3.py` / `verify_phase3_fixed.py` / `verify_phase3_minimal.py` / `verify_phase3_quick.py` | Phase-3 region verification |
| `verify_all_same.py` | Verify symmetric leg configurations |
| `verify_orig_scan.py` | Reproduce original scan results |
| `debug_contact_geometry.py` | Debug contact point geometry |
| `test_self_righting_collision.py` | Collision model correctness test |

