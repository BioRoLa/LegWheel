"""Day 14 Step 3 driver: four legs over one platform, swing up / roll / swing down.

Run from ``LegWheel/``::

    python3 -u hybrid_note/scripts/experiments/day14_step3_driver.py --height-mm 40

Writes into ``hybrid_note/notes/day14/``::

    day14_step3_<h>mm_events.csv        every swing on the clock + slowdowns
    day14_step3_<h>mm_transitions.csv   every transition asked for (accepted or refused)
    day14_step3_<h>mm_validation.csv    Step 9, each check tagged HARD / ADVISORY
    day14_step3_<h>mm_hardware.csv      the 1 kHz Walk-contract CSV (+ _phase)
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np  # noqa: E402

from hybrid_note.scripts.experiments.day10_11_decision_map_2d import StrategyId  # noqa: E402
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    SharedTerrainSpec2D,
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LEG_ORDER, LegId  # noqa: E402
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import RecoveryConfig2D  # noqa: E402
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import (  # noqa: E402
    hybrid_posture_2d,
    hybrid_timing_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_validation_2d import (  # noqa: E402
    CHECK_SEVERITY,
    MOTOR_MAX_RATE_RAD_S,
    validate_whole_body_2d,
)
from hybrid_note.scripts.experiments.day12_world_registration_2d import (  # noqa: E402
    swing_hip_advance_m,
)
from hybrid_note.scripts.experiments.day13_hardware_export_2d import (  # noqa: E402
    NotExportable,
    hardware_command_2d,
    write_hardware_csv_2d,
)
from hybrid_note.scripts.experiments.day14_leg_terrain_rule_2d import (  # noqa: E402
    plan_swing_swing_crossing_2d,
)
from hybrid_note.scripts.experiments.day14_whole_body_planner_2d import (  # noqa: E402
    PlannerRefusal2D,
    refit_body_plane_2d,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day14"

ADVISORY_CHECKS = {c.value for c, sev in CHECK_SEVERITY.items() if sev.value == "ADVISORY"}


def padded(rows):
    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    return [{key: row.get(key, "") for key in keys} for row in rows]


def peak_motor_rate_deg_s(rows: np.ndarray, dt_s: float) -> float:
    theta = rows[:, [0, 2, 4, 6]]
    beta = rows[:, [1, 3, 5, 7]]
    dtheta = np.diff(theta, axis=0) / dt_s
    dbeta = np.diff(beta, axis=0) / dt_s
    return float(np.rad2deg(max(np.abs(dtheta + dbeta).max(), np.abs(dbeta - dtheta).max())))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--height-mm", type=float, default=40.0)
    ap.add_argument("--top-length-mm", type=float, default=400.0)
    ap.add_argument("--x-start-m", type=float, default=1.0)
    ap.add_argument("--samples", type=int, default=601)
    ap.add_argument("--cycles-after", type=int, default=1)
    ap.add_argument("--no-hardware", action="store_true")
    ap.add_argument("--origin-mm", type=float, default=None,
                    help="walk origin in body x; default: the rule's own candidates")
    ap.add_argument("--passes", type=int, default=3)
    ap.add_argument("--period", type=float, default=2.4,
                    help="gait cycle period in s; larger = slower body, same body-x plan")
    ap.add_argument("--fall-ramp-mm", type=float, default=100.0,
                    help="length of the axle's fall ramp after the last descent")
    ap.add_argument("--min-stroke-after-recovery-mm", type=float, default=50.0,
                    help="0 lets a far leg recover, roll a little and climb in a short swing")
    ap.add_argument("--tag", type=str, default="", help="suffix for the output files")
    ap.add_argument("--early-top-landing", action="store_true",
                    help="a climb lands as early as it can on the top (shorter climb, longer descent)")
    ap.add_argument("--pre-drop-mm", type=float, default=0.0,
                    help="lower the raised axle by this much on the top before the first descent")
    ap.add_argument("--crossing-speed", type=float, default=1.0,
                    help="body speed over the crossing as a fraction of the flat speed (flat stays nominal)")
    ap.add_argument("--max-rise-slope", type=float, default=None,
                    help="axle rise per hip travel the top-of-climb swing may carry (rule default 0.5; run 6 had 2.5)")
    ap.add_argument("--late-rise-mm", type=float, default=0.0,
                    help="part of the axle rise deferred to the last --late-rise-ramp-mm before the pair's first descent (legs stand shorter on the top)")
    ap.add_argument("--late-rise-ramp-mm", type=float, default=100.0)
    ap.add_argument("--late-rise-rear-mm", type=float, default=None,
                    help="the rear pair's deferred rise (default: same as --late-rise-mm; 0 = rear rises fully at its climb)")
    ap.add_argument("--rear-prelift-mm", type=float, default=0.0,
                    help="rear axle takes off this much higher at the rear swinger's last recovery before its climb (hardware: the swinging rear hip sags 45-58 mm)")
    ap.add_argument("--rear-prelift-roller-mm", type=float, default=0.0,
                    help="same for the rear roller's recovery into the roll-up's start pose")
    ap.add_argument("--pre-rise-before-mm", type=float, default=220.0,
                    help="first-pass guess profile: the axle starts rising this far before the face")
    ap.add_argument("--pre-rise-until-mm", type=float, default=40.0,
                    help="first-pass guess profile: the axle is fully up this far before the face")
    ap.add_argument("--rear-descent-cap-mm", type=float, default=None,
                    help="the rear pair's first descent lands its contact no further than this past the back face")
    ap.add_argument("--rear-first-descent-travel-mm", type=float, default=0.0,
                    help="the rear pair's first descender takes off this much early and descends with that travel")
    ap.add_argument("--rear-first-descender", type=str, default="RH", choices=["LH", "RH"])
    ap.add_argument("--front-early-recovery", type=str, default=None, choices=["LF", "RF"],
                    help="this front leg recovers early after its descent so it trails while the rear pair's second descent flies")
    ap.add_argument("--front-early-recovery-margin-mm", type=float, default=30.0,
                    help="that leg's next stroke ends this far after the rear pair's second descent lands")
    ap.add_argument("--bound-rear", action="store_true",
                    help="both rear legs roll up together on the right rim (LH lands in place beside RH's roll-start pose)")
    ap.add_argument("--bound-front", action="store_true",
                    help="both front legs roll up together (RF lands in place beside LF's roll-start pose)")
    ap.add_argument("--hop-early", action="store_true",
                    help="on the top, end a stroke where a hop still lands when its natural end is in the dead zone short of the edge")
    ap.add_argument("--fast-fall", action="store_true",
                    help="start the axle's fall as soon as the landed pair's actual stance stands, not at the arc-end limit")
    ap.add_argument("--axle-rise-mm", type=float, default=None,
                    help="raise the axle by this much (not the block's height) while its pair is on the top")
    ap.add_argument("--fold-time", type=float, default=0.0,
                    help="terrain transitions fold and extend no faster than this (s), body stopped")
    ap.add_argument("--theta-compact-deg", type=float, default=None,
                    help="terrain transitions fold to this theta instead of wheel mode's 17 deg")
    ap.add_argument("--fast-long-swings", type=float, default=1.0,
                    help="body may cross a longer-than-nominal swing up to this many times faster")
    ap.add_argument("--rolling-legs", type=str, default=None,
                    help="comma list of legs that roll up (overrides --roll/--bound-*), e.g. LF,RH,LH")
    ap.add_argument("--bound-legs", type=str, default=None,
                    help="comma list of bound partners (the second leg of an axle where both roll), e.g. LH")
    ap.add_argument("--roll", action="store_true",
                    help="LF and RH climb by rolling (Day 6-7 right-rim roll-up); RF and LH swing")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    tag = f"day14_step3_{args.height_mm:.0f}mm" + ("_roll" if (args.roll or args.rolling_legs) else "") + args.tag

    t0 = time.perf_counter()
    spec = SharedTerrainSpec2D(height_m=args.height_mm / 1e3,
                               top_length_m=args.top_length_mm / 1e3,
                               x_start_m=args.x_start_m, arc_samples=121)
    posture = hybrid_posture_2d()
    timing = hybrid_timing_2d(args.period)
    config = RecoveryConfig2D(hip_advance_m=swing_hip_advance_m(timing, posture))
    print(f"terrain {spec.height_m * 1e3:.0f} x {spec.top_length_m * 1e3:.0f} mm at "
          f"x = {spec.x_start_m:.3f} m", flush=True)
    try:
        run, plan, rule, passes = plan_swing_swing_crossing_2d(
            spec, timing=timing, posture=posture, config=config,
            samples=args.samples, cycles_after=args.cycles_after, passes=args.passes,
            origins=None if args.origin_mm is None else [args.origin_mm * 1e-3],
            rolling_legs=(frozenset(LegId[v] for v in args.rolling_legs.split(",") if v)
                          if args.rolling_legs is not None else
                          (frozenset({LegId.LF, LegId.RH}
                                     | ({LegId.LH} if args.bound_rear else set())
                                     | ({LegId.RF} if args.bound_front else set()))
                           if args.roll else frozenset())),
            fall_ramp_m=args.fall_ramp_mm * 1e-3,
            rule_overrides={"min_stroke_after_recovery_m": args.min_stroke_after_recovery_mm * 1e-3,
                            **({} if args.max_rise_slope is None else {"max_rise_slope": float(args.max_rise_slope)}),
                            **({"hop_early_on_top": True} if args.hop_early else {}),
                            "bound_partner_legs": (frozenset(LegId[v] for v in args.bound_legs.split(",") if v)
                                                   if args.bound_legs is not None else
                                                   frozenset(({LegId.LH} if args.bound_rear else set())
                                                             | ({LegId.RF} if args.bound_front else set()))),
                            "prefer_nominal_descent": not args.early_top_landing,
                            **({} if args.rear_descent_cap_mm is None
                               else {"rear_first_descent_contact_cap_m": args.rear_descent_cap_mm * 1e-3}),
                            **({} if args.rear_first_descent_travel_mm <= 0
                               else {"rear_first_descender": LegId[args.rear_first_descender],
                                     "rear_first_descent_travel_m": args.rear_first_descent_travel_mm * 1e-3}),
                            **({} if args.front_early_recovery is None
                               else {"early_recovery_front_leg": LegId[args.front_early_recovery],
                                     "early_recovery_margin_m": args.front_early_recovery_margin_mm * 1e-3}),
                            "transition_theta_compact_rad": (None if args.theta_compact_deg is None
                                                             else float(np.deg2rad(args.theta_compact_deg)))},
            phase_dwell_min_s=args.fold_time,
            axle_rise_override_m=None if args.axle_rise_mm is None else args.axle_rise_mm * 1e-3,
            late_rise_m=args.late_rise_mm * 1e-3, late_rise_ramp_m=args.late_rise_ramp_mm * 1e-3,
            late_rise_rear_m=None if args.late_rise_rear_mm is None else args.late_rise_rear_mm * 1e-3,
            fast_fall=args.fast_fall,
            rear_prelift_m=args.rear_prelift_mm * 1e-3,
            pre_rise_before_m=args.pre_rise_before_mm * 1e-3, pre_rise_until_m=args.pre_rise_until_mm * 1e-3,
            rear_prelift_roller_m=args.rear_prelift_roller_mm * 1e-3,
            long_swing_speed_scale=args.fast_long_swings,
            crossing_speed_scale=args.crossing_speed,
            pre_drop_m=args.pre_drop_mm * 1e-3,
            log=lambda m: print(m, flush=True))
    except PlannerRefusal2D as refusal:
        print(f"[{time.perf_counter() - t0:6.1f}s] REFUSED: {refusal}")
        raise SystemExit(1)
    run, plane = refit_body_plane_2d(run)
    worst = plane.get("coplanarity_worst_at") or {}
    for leg_name, state in (worst.get("legs") or {}).items():
        if state and state.get("hip_x_ends_mm"):
            mount = 0.255 if leg_name in ("LF", "RF") else -0.255
            a, b = state["hip_x_ends_mm"]
            print(f"           worst-instant {leg_name}: segment {state['kind']} hip x {a}..{b} mm "
                  f"clock leave({a})={plan.clock.time_at_body_x(a * 1e-3 - mount, side='leave'):.4f} "
                  f"arrive({b})={plan.clock.time_at_body_x(b * 1e-3 - mount):.4f} "
                  f"window {state['window_s']} frames x {state['hip_x_uniform_mm']}")
    print(f"[{time.perf_counter() - t0:6.1f}s] planned in {passes} pass(es); "
          f"strokes generated {rule.generated_strokes} (cached {rule.cached_strokes}), "
          f"translated {rule.translated_strokes}; body plane {plane}")
    print(f"           {plan.as_dict()}")
    for event in plan.swings:
        print(f"           swing {event.leg.value:2s} {event.kind:20s} "
              f"{event.start_s:8.4f} .. {event.end_s:8.4f} s  body x "
              f"{event.body_x_start_m * 1e3:8.1f} .. {event.body_x_end_m * 1e3:8.1f} mm  "
              f"min {event.minimum_duration_s:.3f} s")
    for slow in plan.slowdowns:
        print(f"           slowdown {slow}")
    for cut in plan.cuts:
        print(f"           cut {cut}")
    lo, hi = plan.schedule.covered_interval_s
    print(f"           covered {lo:.3f} .. {hi:.3f} s   max airborne "
          f"{plan.max_airborne_count}   spread {run.body.world_x_spread_m * 1e3:.4f} mm")
    print(f"           swings: nominal {run.nominal_recovery_swings}, "
          f"terrain {run.terrain_transition_swings}; min margin "
          f"{run.stability.minimum_margin_m}")

    # Validate the trajectory as it will be exported: body z and pitch are the
    # plane through the four hips, not the merged demand.
    report = validate_whole_body_2d(run.trajectory, run.body, run.stability)
    run = replace(run, report=report)
    hard, advisory = [], []
    for check in report.failed_checks():
        (advisory if check.value in ADVISORY_CHECKS else hard).append(check.value)
    print(f"           Step 9 (after the plane refit): HARD failures {hard}   ADVISORY {advisory}")
    for check in report.failed_checks():
        failures = report.failures_of(check)
        first = failures[0]
        print(f"             {check.value:28s} x{len(failures)}: {first.detail[:90]}"
              f"  (first at t={first.time_s}, value={getattr(first, 'value', None)},"
              f" leg={None if first.leg is None else first.leg.value})")

    write_rows_csv(OUT / f"{tag}_events.csv", padded(
        [{"row_kind": "swing", **e.as_dict()} for e in plan.swings]
        + [{"row_kind": "slowdown", **s} for s in plan.slowdowns]
        + [{"row_kind": "cut", **c} for c in plan.cuts]
        + [{"row_kind": "plan", **plan.as_dict(), **plane}]
        + rule.axles.as_rows()))
    write_rows_csv(OUT / f"{tag}_transitions.csv", padded(rule.rows()))
    write_rows_csv(OUT / f"{tag}_validation.csv", padded(
        [{"row_kind": "check", "check": c.value,
          "severity": "ADVISORY" if c.value in ADVISORY_CHECKS else "HARD",
          "failures": len(run.report.failures_of(c)),
          "first_detail": run.report.failures_of(c)[0].detail}
         for c in run.report.failed_checks()]
        + [{"row_kind": "summary", "hard_failures": len(hard),
            "advisory_failures": len(advisory),
            "min_margin_mm": (None if run.stability.minimum_margin_m is None
                              else run.stability.minimum_margin_m * 1e3),
            "world_x_spread_mm": run.body.world_x_spread_m * 1e3,
            "max_airborne": plan.max_airborne_count,
            "nominal_recovery_swings": run.nominal_recovery_swings,
            "terrain_transition_swings": run.terrain_transition_swings}]))

    if not args.no_hardware:
        try:
            command = hardware_command_2d(run.trajectory, reverse=True)
        except NotExportable as error:
            print(f"           NOT EXPORTED: {error}")
        else:
            csv_path, phase_path = write_hardware_csv_2d(command, OUT / f"{tag}_hardware.csv")
            peak = peak_motor_rate_deg_s(command.rows, command.controller_dt_s)
            limit = float(np.rad2deg(MOTOR_MAX_RATE_RAD_S))
            theta = np.rad2deg(command.rows[:, [0, 2, 4, 6]])
            xs = [s.body_position_world_m[0] for s in run.trajectory.samples]
            print(f"           CSV rows {len(command.rows)}  peak per-motor {peak:.1f} deg/s "
                  f"({peak / limit * 100:.1f}%)  theta {theta.min():.2f}..{theta.max():.2f}  "
                  f"rows below 17 deg {(theta < 16.9).sum()}  body x "
                  f"{xs[0] * 1e3:.1f} -> {xs[-1] * 1e3:.1f} mm")
            print(f"wrote -> {csv_path}\nwrote -> {phase_path}")
    print(f"[{time.perf_counter() - t0:6.1f}s] done")


if __name__ == "__main__":
    main()
