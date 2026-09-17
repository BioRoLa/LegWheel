"""Day 13 probe: what theta-compensated rolling changes, end to end.

Day 12 Step 5 found the four-leg body height **infeasible**: the foot rim's
rolling profile makes the hip trace an arc (202.161 mm at the ends, 219.448 mm
in the middle), and three stance legs at different points of that arc demand
three different body heights -- all of them hard.

That arc is **not** a property of the robot.  It is a consequence of rolling at
a fixed ``theta``, which is a modelling choice Day 12 Step 1 made.  The leg has
the joint to cancel it, and (once the motor speed was known: 330 rpm shared
between the two joints) the budget to spare.

This runs the identical pipeline twice -- fixed theta, then held hip -- and
reports what actually moved.  It changes nothing in Day 12: the levelled run is
an opt-in ``NominalPosture2D.hold_hip_z_m``, and every frozen Day 12 number was
measured with the option off.

    python3 -u LegWheel/hybrid_note/scripts/experiments/day13_levelled_rolling_2d.py
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import numpy as np  # noqa: E402

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from hybrid_note.scripts.experiments.day10_11_composer_2d import (  # noqa: E402
    ComposedSequence2D,
)
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (  # noqa: E402
    StrategyId,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (  # noqa: E402
    HIP_TO_BODY_Z_M,
    body_trajectory_2d,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (  # noqa: E402
    LEG_ORDER,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (  # noqa: E402
    NominalPosture2D,
    run_foot_rim_roll_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (  # noqa: E402
    swing_stability_2d,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (  # noqa: E402
    walk_timing_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (  # noqa: E402
    build_leg_plan_2d,
    plan_four_legs_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (  # noqa: E402
    assemble_whole_body_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_validation_2d import (  # noqa: E402
    CheckId,
    validate_whole_body_2d,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day12"


def run_pipeline(posture: NominalPosture2D, nominal_body_z_m: float,
                 *, continuous: bool = False):
    """Day 12 Steps 4-9, unchanged, on whichever posture is handed in."""

    composed = ComposedSequence2D(
        strategy=StrategyId.SWING_SWING, height_m=0.0, top_length_m=0.0,
        sequence=None, refusal="flat run, no crossing")
    plans = {leg: build_leg_plan_2d(leg, composed, posture=posture,
                                    continuous_nominal=continuous)
             for leg in LEG_ORDER}
    four = plan_four_legs_2d(plans, walk_timing_2d())
    body = body_trajectory_2d(four, nominal_body_z_m=nominal_body_z_m,
                              samples=121)
    stability = swing_stability_2d(four, body)
    whole = assemble_whole_body_2d(four, body, stability, samples=121)
    report = validate_whole_body_2d(whole, body, stability)
    return four, body, stability, whole, report


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)

    fixed = NominalPosture2D()
    stroke = run_foot_rim_roll_2d(fixed)
    hip_z = np.array([f.hip_xz_m[1] for f in stroke.frames])
    held = float(hip_z.max())

    print("the arc that Step 5 tripped over")
    print(f"  hip_z {hip_z.min()*1e3:.3f} .. {hip_z.max()*1e3:.3f} mm"
          f"   bob {(hip_z.max()-hip_z.min())*1e3:.4f} mm")
    print(f"  holding at the top, {held*1e3:.4f} mm, keeps body_z at "
          f"{(held - HIP_TO_BODY_Z_M)*1e3:.4f} mm")
    print(f"  -- which is exactly the nominal Step 2 registered.")

    levelled = replace(fixed, hold_hip_z_m=held)
    lev_stroke = run_foot_rim_roll_2d(levelled)
    lev_hip = np.array([f.hip_xz_m[1] for f in lev_stroke.frames])
    lev_theta = np.rad2deg([f.theta_rad for f in lev_stroke.frames])

    print(f"\nthe stroke, both ways")
    print(f"{'':22} {'fixed theta':>14} {'held hip':>14}")
    print(f"{'frames':22} {len(stroke.frames):>14} {len(lev_stroke.frames):>14}")
    print(f"{'stop reason':22} {stroke.stop_reason:>14} "
          f"{lev_stroke.stop_reason:>14}")
    print(f"{'hip bob (mm)':22} {(hip_z.max()-hip_z.min())*1e3:>14.4f} "
          f"{(lev_hip.max()-lev_hip.min())*1e3:>14.6f}")
    print(f"{'theta sweep (deg)':22} {0.0:>14.3f} {np.ptp(lev_theta):>14.3f}")
    advance = (stroke.frames[-1].contact_xz_m[0]
               - stroke.frames[0].contact_xz_m[0])
    lev_advance = (lev_stroke.frames[-1].contact_xz_m[0]
                   - lev_stroke.frames[0].contact_xz_m[0])
    print(f"{'contact advance (mm)':22} {advance*1e3:>14.3f} "
          f"{lev_advance*1e3:>14.3f}")
    print(f"{'rims used':22} {','.join(sorted({f.rim for f in stroke.frames})):>14} "
          f"{','.join(sorted({f.rim for f in lev_stroke.frames})):>14}")

    rows = []
    results = {}
    variants = (
        ("fixed_theta", fixed, False),
        ("held_hip", levelled, False),
        ("held_hip+chained", levelled, True),
    )
    for name, posture, continuous in variants:
        four, body, stability, whole, report = run_pipeline(
            posture, held - HIP_TO_BODY_Z_M, continuous=continuous)
        results[name] = (four, body, stability, whole, report)
        finite = np.isfinite(body.body_z_m).sum()
        worst = max((c.disagreement_m for c in body.conflicts), default=0.0)
        rows.append({
            "variant": name,
            "body_conflicts": len(body.conflicts),
            "worst_body_disagreement_mm": worst * 1e3,
            "usable_body_samples": int(finite),
            "total_body_samples": len(body.samples),
            "body_is_feasible": body.is_feasible,
            "min_stability_margin_mm": (
                None if stability.minimum_margin_m is None
                else stability.minimum_margin_m * 1e3),
            "unstable_swings": len(stability.unstable_swings),
            "swings": len(stability.swings),
            "checks_failed": len(report.failed_checks()),
            "failures": len(report.failures),
            "failed_checks": ",".join(c.value for c in report.failed_checks()),
        })

    write_rows_csv(OUT / "day13_levelled_rolling.csv", rows)

    print(f"\nwhat that does to Day 12 Steps 5, 6 and 9")
    print(f"{'':30} {'fixed theta':>16} {'held hip':>16} {'+chained':>16}")
    for key, label, fmt in (
        ("body_conflicts", "Step 5 body conflicts", "{:>16}"),
        ("worst_body_disagreement_mm", "  worst disagreement (mm)", "{:>16.3f}"),
        ("usable_body_samples", "  usable body samples", "{:>16}"),
        ("body_is_feasible", "  body feasible", "{:>16}"),
        ("min_stability_margin_mm", "Step 6 min margin (mm)", "{:>16.4f}"),
        ("unstable_swings", "  unstable swings", "{:>16}"),
        ("checks_failed", "Step 9 checks failed", "{:>16}"),
        ("failures", "  failure records", "{:>16}"),
    ):
        values = [row[key] for row in rows]
        try:
            print(f"{label:30} " + " ".join(fmt.format(v) for v in values))
        except (TypeError, ValueError):
            print(f"{label:30} " + " ".join(f"{str(v):>16}" for v in values))

    print(f"\nStep 9, check by check")
    for check in CheckId:
        counts = [len(results[name][4].failures_of(check))
                  for name, _, _ in variants]
        mark = "" if len(set(counts)) == 1 else "   <-- changed"
        print(f"  {check.value:32s} "
              + " -> ".join(f"{c:>4}" for c in counts) + mark)

    print(f"\nwrote -> {OUT}")


if __name__ == "__main__":
    main()
