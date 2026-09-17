"""Day 13: splice the verified flat gait onto an obstacle crossing.

The project owner's design, and it is better than slowing one plan down:

    walk the approach with the EXISTING flat CSV -> stop -> cross the obstacle

Why it matters beyond tidiness.  Planning the whole run as one obstacle plan
put the flat part on the obstacle plan's grid: 121 samples over 38.9 s is one
planner point every 324 ms, against ``hybrid_flat_v1``'s 3960 points at 5 ms.
The 1 kHz resample then interpolates the flat walking rather than commanding
it, which is why the approach "looked very different" from the flat gait the
owner had already driven.  Splicing keeps the flat half exactly as verified.

Three things the splice has to get right, all measured (log 28):

``beta is a turn counter``
    ``hybrid_flat_v1`` ends near -3200 deg (about -8.9 turns) after 3.16 m; the
    obstacle plan starts its own count near -40 deg.  The obstacle half is
    therefore offset by the flat half's final beta, per leg.  Butting them
    together without it would command the wheels to unwind nine turns in one
    controller step.

``the obstacle plan carries its own approach``
    Its body runs 148 -> 1589 mm for an obstacle at 1000 mm, so 59% of it is
    walking up to the obstacle.  That prefix is dropped here: the flat half has
    already done it.

``the halves must meet at a pose``
    theta agrees to within 3.4 deg (FR is exact), because both use the same
    nominal posture family.  A hold is inserted so the machine settles before
    the crossing starts, which is what "stop and change gait" means anyway.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from legwheel.models.corgi_leg import CorgiLegKinematics  # noqa: E402
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (  # noqa: E402
    StrategyId,
    load_tables_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    SharedTerrainSpec2D,
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import (  # noqa: E402
    hybrid_timing_2d,
)
from hybrid_note.scripts.experiments.day12_terrain_generalization_2d import (  # noqa: E402
    plan_terrain_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_validation_2d import (  # noqa: E402
    MOTOR_MAX_RATE_RAD_S,
)
from hybrid_note.scripts.experiments.day13_hardware_export_2d import (  # noqa: E402
    planner_rows_2d,
)
from hybrid_note.scripts.experiments.day13_step4_obstacle_driver import (  # noqa: E402
    peak_motor_rate_deg_s,
)

NOTES = _ROOT / "hybrid_note" / "notes"
OUT = NOTES / "day13"
CONTROLLER_DT_S = 1e-3
PREP_ROWS = 5000


def _flat_walks_forward(rows: np.ndarray, phase: np.ndarray,
                        skip: int = PREP_ROWS) -> bool:
    """Does this **flat-walking** block carry the body forwards?

    Geometric, not a sign convention: a stance foot is planted, so its drift in
    the hip frame is the hip's drift reversed.  A foot moving backwards (-x)
    while on the ground means the hip advances.  Verified against
    ``outputs/csv/Walk_Vx0.10_...`` (known forward) and ``hybrid_flat_v1``.

    **Only valid for flat walking**, and the name says so because assuming
    otherwise cost a full round here.  The test rests on "the foot is planted
    and the leg rolls beneath it", which is what ``FOOT_RIM_ROLL`` does.  A
    crossing's stance is mostly ``WHEEL_TRANSITION`` -- retracted to 17 deg,
    rolling along the top of the obstacle on a different rim at a changing
    posture -- and there the foot's hip-frame motion no longer stands in for
    the body's.  Measured: the kept crossing portion drifts +85.7 um/step,
    which this function would read as backwards, while the planner has the body
    advancing 550.3 -> 1600.0 mm monotonically over the very same rows.

    For a crossing, ask the planner (``body_position_world_m``) instead.

    Stance comes from the phase sidecar rather than a "the foot barely moved"
    threshold: such a threshold is a rate in disguise, and the crossing half is
    stretched 16x, so swing frames slip through it.
    """

    leg = CorgiLegKinematics(0, gamma=0.0)
    gait, ph = rows[skip:], phase[skip:]
    drifts = []
    for j in range(min(len(gait), len(ph)) - 1):
        if ph[j, 0] != 0 or ph[j + 1, 0] != 0:      # FL airborne: not stance
            continue
        a = leg.forward_kinematics(float(gait[j, 0]), float(gait[j, 1]), 0.0)
        b = leg.forward_kinematics(float(gait[j + 1, 0]), float(gait[j + 1, 1]), 0.0)
        drifts.append(b[0] - a[0])
    if not drifts:
        raise SystemExit("no stance frames found; cannot determine direction.")
    return float(np.median(drifts)) < 0.0


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--flat-csv", default="hybrid_flat_v1",
                    help="name (no extension) of the flat CSV in notes/day13")
    ap.add_argument("--height-mm", type=float, default=40.0)
    ap.add_argument("--top-length-mm", type=float, default=400.0)
    ap.add_argument("--x-start-m", type=float, default=1.0)
    ap.add_argument("--whole-body", action="store_true",
                    help="schedule the four legs as one machine: hold the "
                         "BODY's clock until the previous swing has landed, "
                         "instead of deciding each leg's timing from its own "
                         "position.  Position scheduling makes a leg's time "
                         "its claim about where the body is, so a pair sharing "
                         "a mount_x always swings together; holding the body "
                         "delays every leg equally and cannot make them "
                         "disagree (log 33).")
    ap.add_argument("--strategy", choices=("roll", "swing"), default=None,
                    help="force ROLL_ROLL or SWING_SWING instead of taking the "
                         "decision rule's pick.  The rule ranks by body "
                         "deviation and so prefers SWING wherever both work "
                         "(ROLL loses by 14.3 mm at 40 mm, 14.9 at 100); ROLL "
                         "keeps contact all the way up, which is the reason to "
                         "ask for it.")
    ap.add_argument("--samples", type=int, default=1201,
                    help="planner samples for the CROSSING half only.  It is "
                         "a short span, so this can be dense without the cost "
                         "of planning the whole run finely.")
    ap.add_argument("--margin-floor-mm", type=float, default=-40.0)
    ap.add_argument("--crossing-slowdown", type=float, default=1.0,
                    help="stretch the crossing half in time by this factor.  "
                         "The flat half is untouched, so the robot walks at "
                         "its normal pace and only crosses slowly.  The joint "
                         "rates scale as 1/factor, which is the direct knob on "
                         "motor_rate_limit -- the recovery swings inside the "
                         "crossing are what exceed it.")
    ap.add_argument("--hold-s", type=float, default=1.0,
                    help="how long the machine stands still between the two "
                         "halves.  This is the 'stop and change gait' moment.")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    flat = np.loadtxt(OUT / f"{args.flat_csv}.csv", delimiter=",")
    flat_phase = np.loadtxt(OUT / f"{args.flat_csv}_phase.csv",
                            delimiter=",", skiprows=1)
    print(f"flat half : {args.flat_csv}  {flat.shape[0]} rows "
          f"({flat.shape[0] * CONTROLLER_DT_S:.3f} s)")

    terrain = SharedTerrainSpec2D(height_m=args.height_mm / 1e3,
                                  top_length_m=args.top_length_mm / 1e3,
                                  x_start_m=args.x_start_m)
    tables = load_tables_2d(NOTES / "day10-11", NOTES / "day6-7")
    print(f"planning the crossing at {args.samples} samples ...")
    forced = {"roll": StrategyId.ROLL_ROLL,
              "swing": StrategyId.SWING_SWING}.get(args.strategy)
    run = plan_terrain_2d(terrain, tables, timing=hybrid_timing_2d(),
                          margin_floor_m=args.margin_floor_mm / 1e3,
                          samples=args.samples, strategy=forced,
                          whole_body=args.whole_body)
    if not run.planned:
        print("  NOT PLANNED -- nothing to splice:")
        for f in run.failures:
            print(f"     {f.stage.value}: {f.detail[:100]}")
        return

    # Drop the crossing plan's own approach: keep from where the body is close
    # enough to the obstacle that the flat half has taken it there.
    samples = run.trajectory.samples
    xs = np.array([s.body_position_world_m[0] for s in samples])
    keep_from = int(np.searchsorted(xs, args.x_start_m - 0.45))
    print(f"  crossing samples {len(samples)}, body x "
          f"{xs[0]*1e3:.1f}..{xs[-1]*1e3:.1f} mm")
    print(f"  dropping the first {keep_from} (its own approach), "
          f"keeping from x = {xs[keep_from]*1e3:.1f} mm")

    # Direction, decided from the planner rather than inferred.
    #
    # Got wrong four times, each by reasoning about ``beta``'s sign or about
    # the other half's file instead of asking what the plan does.  The one
    # source of truth is ``body_position_world_m``: the planner says where the
    # body goes.  Measured on this crossing it runs 148.1 -> 1589.3 mm, i.e.
    # forward, and only ``reverse=True`` exports rows whose stance feet drift
    # backwards in the hip frame -- which is what carries the hip forwards.
    # The flat generator needs ``True`` for the same reason (0 -> 475.3 mm).
    #
    # The previous version computed ``reverse = not _walks_forward(flat)``,
    # which is backwards twice over: the flat CSV is a finished artefact whose
    # direction is already settled, so a *correct* flat half made this pass
    # ``False`` to the crossing and produced exactly the mismatch the check
    # then caught.  What the crossing needs has nothing to do with what the
    # flat half happens to be; it is a property of this plan.
    body_xs = [s.body_position_world_m[0] for s in run.trajectory.samples]
    planner_forward = body_xs[-1] > body_xs[0]
    reverse = planner_forward
    print(f"  planner body x {body_xs[0]*1e3:.1f} -> {body_xs[-1]*1e3:.1f} mm"
          f" ({'forward' if planner_forward else 'backward'})"
          f" -> reverse={reverse}")
    rows, phase, dt = planner_rows_2d(run.trajectory, reverse=reverse)
    rows, phase = rows[keep_from:], phase[keep_from:]
    body_kept = np.asarray(body_xs[keep_from:], dtype=float)

    # 1 kHz by linear interpolation between planner rows.
    #
    # Holding each row instead (np.repeat) was tried and is worse: a staircase
    # is not smoother, it is a sequence of instantaneous steps, and every step
    # edge shows up as an unbounded joint rate -- 344 spikes of 10811 deg/s,
    # one every 8 ms, exactly the repeat period.  "No interpolation" trades
    # invented motion for invented *discontinuity*, which the controller has to
    # execute either way.  Interpolating is honest about the same uncertainty
    # and at least commands a reachable rate; the planner grid is dense enough
    # here (7.58 ms, against the flat gait's 5 ms) that little is being guessed.
    dt = dt * args.crossing_slowdown
    n_out = int(round((len(rows) - 1) * dt / CONTROLLER_DT_S)) + 1
    src_t = np.arange(len(rows)) * dt
    dst_t = np.arange(n_out) * CONTROLLER_DT_S
    cross = np.empty((n_out, rows.shape[1]), dtype=float)
    for col in range(rows.shape[1]):
        cross[:, col] = np.interp(dst_t, src_t, rows[:, col])
    # Phase is a label, not a signal: take the nearest planner row's value
    # rather than blending two flags into a fraction.
    idx = np.clip(np.round(dst_t / dt).astype(int), 0, len(phase) - 1)
    cross_phase = phase[idx]
    print(f"  crossing dt {dt*1e3:.2f} ms -> interpolated to "
          f"{len(cross)} rows ({len(cross)*CONTROLLER_DT_S:.2f} s)")

    # beta is a turn counter: carry the flat half's count into the crossing.
    beta_cols = [1, 3, 5, 7]
    offset = flat[-1, beta_cols] - cross[0, beta_cols]
    cross[:, beta_cols] += offset
    print(f"  beta carry-over per leg (deg): "
          f"{np.rad2deg(offset).round(1)}")

    # The stop, as a cosine ease from the flat half's last pose to the
    # crossing's first -- not a hold at one pose followed by a jump to the
    # other, which is what the first version did and which put a single
    # 51908 deg/s step in the middle of the pause.  theta differs by up to
    # 3.4 deg between the halves, so the machine really does have to move; the
    # honest way to spend a 1 s pause is to move that little, slowly.
    hold_n = int(round(args.hold_s / CONTROLLER_DT_S))
    ease = (1.0 - np.cos(np.linspace(0.0, np.pi, hold_n))) / 2.0
    hold = flat[-1] + ease[:, None] * (cross[0] - flat[-1])
    hold_phase = np.zeros((hold_n, 4), dtype=np.int8)
    move = np.rad2deg(np.abs(cross[0] - flat[-1])[[0, 2, 4, 6]])
    print(f"  pause moves theta by {move.round(2)} deg over {args.hold_s} s")

    out_rows = np.vstack([flat, hold, cross])
    out_phase = np.vstack([flat_phase.astype(np.int8), hold_phase, cross_phase])

    name = args.out or f"hybrid_spliced_{args.height_mm:.0f}mm"
    np.savetxt(OUT / f"{name}.csv", out_rows, delimiter=",", fmt="%.6f")
    np.savetxt(OUT / f"{name}_phase.csv", out_phase, delimiter=",", fmt="%.0f",
               header="FL(LF),FR(RF),RR(RH),RL(LH)", comments="")

    # The finished file must walk FORWARDS, and both halves must agree.
    #
    # Checked geometrically, because every other check in this pipeline passes
    # on a trajectory that runs backwards -- theta range, motor rate, folded
    # legs, chaining all look fine while the machine reverses.  An earlier
    # version of this only compared the two halves with each other, which
    # cannot catch both being wrong together, and they were.
    if not _flat_walks_forward(out_rows[:len(flat)], out_phase[:len(flat)]):
        raise SystemExit(
            "the flat half walks backwards: a stance foot drifts +x in the hip "
            "frame, which carries the hip the wrong way."
        )
    # The crossing half is checked against the planner, not the feet: see
    # ``_flat_walks_forward``.  ``body_kept`` was taken from the same samples
    # that were exported, so this is the direction of what was actually written.
    if body_kept[-1] <= body_kept[0]:
        raise SystemExit(
            f"the crossing half moves the body backwards "
            f"({body_kept[0]*1e3:.1f} -> {body_kept[-1]*1e3:.1f} mm)."
        )
    print(f"  direction check: flat half walks forwards; crossing body "
          f"{body_kept[0]*1e3:.1f} -> {body_kept[-1]*1e3:.1f} mm")

    theta_deg = np.rad2deg(out_rows[:, [0, 2, 4, 6]])
    peak = peak_motor_rate_deg_s(out_rows, CONTROLLER_DT_S)
    limit = float(np.rad2deg(MOTOR_MAX_RATE_RAD_S))
    print(f"\nspliced: {len(out_rows)} rows "
          f"({len(out_rows)*CONTROLLER_DT_S:.2f} s)")
    print(f"  theta range      {theta_deg.min():.2f} .. {theta_deg.max():.2f} deg")
    print(f"  rows below 17deg {(theta_deg < 16.9).sum()}")
    print(f"  peak per-motor   {peak:.2f} deg/s ({peak/limit*100:.1f}% of limit)")

    write_rows_csv(OUT / f"{name}_summary.csv", [{
        "row_kind": "spliced",
        "SIMULATION_ONLY": "the crossing half does NOT pass Step 9",
        "flat_source": args.flat_csv,
        "flat_rows": int(flat.shape[0]),
        "hold_rows": int(hold_n),
        "crossing_rows": int(len(cross)),
        "total_rows": int(len(out_rows)),
        "duration_s": len(out_rows) * CONTROLLER_DT_S,
        "height_mm": args.height_mm,
        "top_length_mm": args.top_length_mm,
        "x_start_m": args.x_start_m,
        "crossing_samples": args.samples,
        "strategy": getattr(run.composed.strategy, "value", None)
                    if run.composed else None,
        "strategy_forced": args.strategy or "(decision rule)",
        "whole_body_scheduling": bool(args.whole_body),
        "whole_body_holds": len(run.swing_waits),
        "crossing_planner_dt_s": dt,
        "crossing_slowdown": args.crossing_slowdown,
        "reverse": bool(reverse),
        "reverse_source": "inherited from the flat half",
        "dropped_approach_samples": keep_from,
        "theta_min_deg": float(theta_deg.min()),
        "peak_per_motor_deg_s": peak,
        "peak_percent_of_limit": peak / limit * 100.0,
        "step9_failures_crossing": len(run.report.failures) if run.report else "",
    }])
    print(f"\nwrote -> {OUT / f'{name}.csv'}")
    print(f"wrote -> {OUT / f'{name}_phase.csv'}")
    print(f"wrote -> {OUT / f'{name}_summary.csv'}")


if __name__ == "__main__":
    main()
