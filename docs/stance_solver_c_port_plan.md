# Stance Solver C Port Plan for CSV Generation

## Goal

Accelerate hardware CSV generation by reducing the cost of stance trajectory generation. Profiling on the current Python path showed that CSV writing is not the bottleneck; most runtime is spent inside stance-phase rolling Jacobian evaluation:

```text
generate_hardware_csv
└── GaitGenerator3D.generate_full_gait
    └── TrajectoryPlanner3D.generate_trajectory
        └── stance_rt_solver
            ├── numerical_jacobian
            │   └── rolling_fk
            │       ├── foot_rim_contact_fk
            │       │   └── forward_kinematics repeated over w samples
            │       └── forward_kinematics
            └── DLS solve
```

A previous C-base branch showed useful micro-kernel speedups (DLS, Bezier, screw exponential), but end-to-end CSV generation did not improve materially because the Python loop, contact search, and repeated FK calls still dominate.

## Baseline to measure against

Use the production benchmark script:

```bash
cd LegWheel
uv run python examples/gait/benchmark_csv_generation.py \
  --gait Trot --vx 0.05 --vy 0.03 --height 0.25 \
  --period 1.0 --cycles 2 --dt 0.002 \
  --repeats 3 --profile
```

Track:

- `timing.mean_ms`
- `throughput.rows_per_second_mean`
- `throughput.seconds_per_1000_rows_mean`
- top cumulative profile entries

## Recommended acceleration phases

### Phase 1 — Reduce contact-search FK calls in Python

Before C porting, reduce the number of repeated FK calls in `foot_rim_contact_fk()`.

Current cost driver:

- Nonzero `gamma` samples 9 lateral `w` positions.
- Each `w` sample calls full `forward_kinematics()`.
- Numerical Jacobian calls `rolling_fk()` repeatedly, multiplying this cost.

Candidate approaches:

1. Cache the previous contact `w` during stance and only resample when `gamma` changes beyond a threshold.
2. Replace 9-point uniform sampling with a smaller coarse/fine search.
3. Derive a semi-analytic toroidal contact `w_contact(gamma)` and only validate by sampling near the predicted contact.

Expected payoff: high. This directly reduces the largest repeated FK multiplier.

### Phase 2 — Specialized rolling Jacobian

Replace generic `numerical_jacobian(rolling_fk, q)` with a stance-specific Jacobian routine.

Candidate API:

```python
def rolling_jacobian_fast(kin, q, ground_slope=0.0, diff=1e-5):
    """Return 3x3 rolling-contact Jacobian for stance solving."""
```

Possible implementation levels:

1. Python specialized finite difference:
   - Compute base contact once.
   - Reuse center-plane `w=0` for velocity mapping.
   - Avoid full lateral contact search for every finite-difference perturbation.
2. Hybrid analytic/numeric:
   - Analytic or semi-analytic sagittal rolling derivative for `theta`/`beta`.
   - Dedicated lateral derivative for `gamma`.
3. C implementation after Python behavior is validated.

Expected payoff: high. Profiling showed numerical Jacobian and FK dominate total runtime.

### Phase 3 — C port the stance loop, not only micro-kernels

If Python-side reductions are insufficient, move the whole stance update kernel into C.

Suggested C boundary:

```c
int stance_solve_batch(
    const double *q0,              // shape (3,)
    const double *velocity,        // shape (3,)
    double stand_height,
    double period,
    double dt,
    double stance_duty,
    int n_steps,
    const double *robot_params,
    double *q_out                 // shape (n_steps, 3)
);
```

Python wrapper:

```python
def stance_solve_batch_c(planner: TrajectoryPlanner3D, q0: np.ndarray, n_steps: int) -> np.ndarray:
    """Return stance command sequence with optional C backend."""
```

Keep `LEGWHEEL_USE_CBASE=1` or an explicit method flag so the C path remains opt-in until validated.

## Validation requirements

For every optimization phase:

1. Preserve current tests:
   - `pytest tests/test_toroidal_contact_and_checker.py`
   - `pytest tests/test_lateral_stance_symmetry.py`
   - `pytest tests/test_gait_duty_and_swing_velocity.py`
2. Add targeted equivalence tests:
   - Compare old/new stance command sequence over a fixed seedless scenario.
   - Assert max joint difference below a chosen tolerance.
   - Assert FK foot tracking error does not regress.
3. Re-run benchmark:
   - Same gait parameters before and after.
   - Report rows/s and seconds per 1000 rows.

## Why not directly merge `feature/c-based-benchmark`?

Do not merge the full branch into the current toroidal-tire branch. It is based on an older code state and contains diffs that would revert parts of the toroidal tire geometry path, including `rim_point(alpha, w)` and contact handling.

Safe parts to port selectively:

- `legwheel/cbase/*`
- benchmark scripts, after adapting to current branch
- optional dispatcher ideas, but only after behavior-preserving tests

Risky parts to avoid or rewrite:

- broad diffs in `legwheel/models/corgi_leg.py`
- broad diffs in visualization/model code that remove toroidal `w` handling
- production dispatch that silently falls back without test visibility

## Success criteria

A C or optimized Python path is worth keeping only if it shows a clear end-to-end CSV-generation improvement, not just micro-kernel improvement.

Suggested threshold:

- At least 20% faster `rows_per_second_mean` on representative Trot and Walk cases, or
- At least 2x faster for long hardware-generation cases (`dt=0.001`, multiple cycles), without increasing FK tracking error.
