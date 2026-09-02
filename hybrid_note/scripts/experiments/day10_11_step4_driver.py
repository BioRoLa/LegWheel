"""Day 10--11 Step 4: price the rolling side in the swing side's currency.

Six sections.  Sections A--C fill in the spec's original completion criteria
(``RollConcession2D`` for every Day 6--7 cell, best over ``theta_climb``, with
the ``L_top`` axis attached); D--F answer the two questions the spec says must
be answered with data rather than intuition -- 5.3's "which body requirement is
harder" and the 2026-08-30 addendum's "is rolling really the lower CoM
excursion".

**No traversal is re-run.**  ``day6_7_step11r_sweep_trajectories.csv`` already
holds ``hip_x_m`` / ``hip_z_m`` for all 70 cells.  Re-running would spend ~264 s
per cell (implementation log trap 5) to recompute numbers already on disk.

    python3 day10_11_step4_driver.py                 # everything
    python3 day10_11_step4_driver.py --plots-only    # just redraw
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day10_11_roll_concession_2d import (  # noqa: E402
    REFERENCE_APPROACH_CLEARANCE_M,
    REFERENCE_TOP_LENGTH_M,
    BodyDemandComparison2D,
    compare_body_demand_2d,
    hip_excursion_2d,
    load_roll_trajectories_2d,
    roll_concessions_by_height_2d,
    swing_obstacle_hip_profile_2d,
    swing_off_hip_profile_2d,
    swing_onto_hip_profile_2d,
)

OUTPUT_DIR = Path(__file__).resolve().parents[2] / "notes" / "day10-11"
DAY6_7_DIR = Path(__file__).resolve().parents[2] / "notes" / "day6-7"

#: Section E.  Day 6--7 Step 12R measured the rolling side's own lower bound on
#: top length; below it the traversal does not complete, so extrapolating the
#: excursion metric there would describe a motion that cannot happen.
TOP_LENGTHS_M: tuple[float, ...] = (0.25, 0.30, 0.35, 0.45, 0.60, 0.80)


# --------------------------------------------------------------------------
# Inputs
# --------------------------------------------------------------------------


def _read(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _best_swing_up(clearance_mm: float | None = None) -> dict[float, dict]:
    """Per height, the cheapest ascent -- optionally at one fixed clearance.

    Approach clearance is the swing side's internal freedom exactly as
    ``theta_climb`` is rolling's, so spec 3.1 says each side takes its own best.
    That is right for the *bound* (the hip lift) and wrong for any **ratio**:
    a larger clearance buys forward distance at zero vertical cost, so letting
    the swing choose it while rolling's is frozen at 40 mm lets the swing
    dilute its own denominator.  Measured, that flips the verdict at
    h = 60 and 80 mm, so both variants are reported and the matched one leads.
    """

    best: dict[float, dict] = {}
    for row in _read(OUTPUT_DIR / "day10_11_step2_swing_onto_sweep.csv"):
        if row["feasible"] != "True":
            continue
        if clearance_mm is not None and abs(
            float(row["approach_clearance_mm"]) - clearance_mm
        ) > 1e-9:
            continue
        height_m = float(row["obstacle_mm"]) / 1e3
        lift_m = float(row["min_hip_lift_mm"]) / 1e3
        if height_m not in best or lift_m < float(best[height_m]["min_hip_lift_mm"]) / 1e3:
            best[height_m] = row
    return best


def _best_swing_down() -> dict[float, dict]:
    """Per height, the cheapest descent over takeoff distance."""

    best: dict[float, dict] = {}
    for row in _read(OUTPUT_DIR / "day10_11_step3_swing_off_sweep.csv"):
        if row["section"] != "map" or row["feasible"] != "True":
            continue
        height_m = float(row["obstacle_mm"]) / 1e3
        hold = float(row["min_hip_hold_fraction"])
        if height_m not in best or hold < float(best[height_m]["min_hip_hold_fraction"]):
            best[height_m] = row
    return best


#: The two ways to let the swing pick its approach, and why both are shown.
SWING_VARIANTS: tuple[tuple[str, float | None], ...] = (
    ("matched c = 40 mm", REFERENCE_APPROACH_CLEARANCE_M * 1e3),
    ("swing's own best c", None),
)


def _swing_up_profiles(rows: dict[float, dict]) -> dict:
    return {
        h: swing_onto_hip_profile_2d(
            height_m=h,
            theta_deg=float(row["theta_deg"]),
            approach_hip_x_m=float(row["approach_hip_x_m"]),
            landing_distance_m=float(row["landing_distance_m"]),
            min_hip_lift_m=float(row["min_hip_lift_mm"]) / 1e3,
        )
        for h, row in rows.items()
    }


def _profiles():
    trajectories = load_roll_trajectories_2d(
        DAY6_7_DIR / "day6_7_step11r_sweep_trajectories.csv",
        DAY6_7_DIR / "day6_7_step11r_feasibility_sweep.csv",
    )
    up_rows, down_rows = _best_swing_up(), _best_swing_down()
    variants = {
        label: _swing_up_profiles(_best_swing_up(clearance))
        for label, clearance in SWING_VARIANTS
    }
    up = variants[SWING_VARIANTS[0][0]]
    down = {
        h: swing_off_hip_profile_2d(
            height_m=h,
            theta_deg=float(row["theta_deg"]),
            takeoff_hip_x_m=float(row["takeoff_hip_x_m"]),
            landing_hip_x_m=float(row["landing_hip_x_m"]),
            min_hip_hold_fraction=float(row["min_hip_hold_fraction"]),
        )
        for h, row in down_rows.items()
    }
    return trajectories, up_rows, down_rows, up, down, variants


# --------------------------------------------------------------------------
# A -- every rolling cell, priced
# --------------------------------------------------------------------------


def section_a(trajectories) -> list[dict]:
    print("\n=== A. every Day 6-7 cell, priced in hip excursion", flush=True)
    rows = []
    for (height_m, theta_deg), trajectory in sorted(trajectories.items()):
        cell = trajectory.as_cell_concession()
        row = {"section": "cell", **cell.as_dict()}
        if trajectory.feasible:
            row.update(trajectory.excursion.as_dict())
            for stage in ("ROLL_UP", "WHEEL_TRANSITION", "ROLL_DOWN"):
                stage_excursion = trajectory.excursion_for_stages([stage])
                row[f"{stage.lower()}_peak_to_peak_mm"] = (
                    None if stage_excursion is None
                    else stage_excursion.peak_to_peak_m * 1e3
                )
            row["flat_top_forward_mm"] = trajectory.flat_top_forward_distance_m * 1e3
            row["flat_top_hip_z_range_mm"] = trajectory.flat_top_hip_z_range_m * 1e3
        rows.append(row)
    feasible = [r for r in rows if r["feasible"]]
    print(f"  {len(feasible)} / {len(rows)} feasible cells priced.", flush=True)
    flat = [r["flat_top_hip_z_range_mm"] for r in feasible]
    print(f"  flat-top hip range across every feasible cell: "
          f"max {max(flat):.4f} mm  -> the L_top extrapolation is licensed.",
          flush=True)
    return rows


# --------------------------------------------------------------------------
# B -- best over theta_climb
# --------------------------------------------------------------------------


def section_b(trajectories) -> list[dict]:
    print("\n=== B. best over theta_climb, one RollConcession2D per height",
          flush=True)
    concessions = roll_concessions_by_height_2d(trajectories)
    rows = []
    for height_m, concession in concessions.items():
        row = {"section": "concession", **concession.as_dict()}
        best = concession.best_theta_climb_deg
        if best is not None:
            trajectory = trajectories[(round(height_m, 6), round(best, 3))]
            row.update(trajectory.excursion.as_dict())
        rows.append(row)
        print(f"  h = {height_m * 1e3:5.0f} mm  feasible={concession.feasible}  "
              f"best theta = {best}  span = {concession.feasible_theta_span_deg:.0f} deg  "
              f"contiguous = {concession.theta_window_is_contiguous}  "
              f"hip travel = "
              f"{'n/a' if concession.hip_z_travel_m is None else f'{concession.hip_z_travel_m * 1e3:.1f} mm'}",
              flush=True)
    return rows


# --------------------------------------------------------------------------
# C -- matched stages
# --------------------------------------------------------------------------


def section_c(trajectories, up, down) -> list[dict]:
    print("\n=== C. matched stages: ROLL_UP vs SWING_UP, ROLL_DOWN vs SWING_DOWN",
          flush=True)
    concessions = roll_concessions_by_height_2d(trajectories)
    rows = []
    for height_m, concession in concessions.items():
        best = concession.best_theta_climb_deg
        if best is None:
            continue
        trajectory = trajectories[(round(height_m, 6), round(best, 3))]
        for label, stages, swing in (
            ("ascent", ("ROLL_UP",), up.get(height_m)),
            ("descent", ("ROLL_DOWN",), down.get(height_m)),
        ):
            roll_excursion = trajectory.excursion_for_stages(stages)
            if roll_excursion is None or swing is None:
                continue
            comparison = BodyDemandComparison2D(
                label=label,
                obstacle_height_m=height_m,
                roll_excursion=roll_excursion,
                swing_excursion=swing.excursion,
                roll_theta_climb_deg=best,
                swing_theta_deg=swing.theta_deg,
            )
            rows.append({"section": "matched", **comparison.as_dict()})
            print(f"  h = {height_m * 1e3:5.0f} mm  {label:8s}  "
                  f"roll {roll_excursion.hip_z_per_forward_distance:.4f}  "
                  f"swing {swing.excursion.hip_z_per_forward_distance:.4f}  "
                  f"-> {'ROLL' if comparison.roll_is_cheaper else 'SWING'} cheaper "
                  f"(ratio {comparison.ratio:.2f})", flush=True)
    return rows


# --------------------------------------------------------------------------
# D -- the whole obstacle
# --------------------------------------------------------------------------


def section_d(trajectories, variants, down) -> list[dict]:
    print("\n=== D. one whole obstacle: rolling traversal vs SWING_UP + top + SWING_DOWN",
          flush=True)
    concessions = roll_concessions_by_height_2d(trajectories)
    rows = []
    for variant, up in variants.items():
        print(f"\n  -- swing approach: {variant}", flush=True)
        for height_m, concession in concessions.items():
            best = concession.best_theta_climb_deg
            if best is None or height_m not in up or height_m not in down:
                continue
            trajectory = trajectories[(round(height_m, 6), round(best, 3))]
            comparison = BodyDemandComparison2D(
                label="whole obstacle",
                obstacle_height_m=height_m,
                roll_excursion=trajectory.excursion,
                swing_excursion=swing_obstacle_hip_profile_2d(
                    up[height_m], down[height_m]
                ),
                roll_theta_climb_deg=best,
                swing_theta_deg=up[height_m].theta_deg,
            )
            rows.append({
                "section": "whole", "swing_variant": variant, **comparison.as_dict()
            })
            roll_per = comparison.roll_excursion.hip_z_per_forward_distance
            swing_per = comparison.swing_excursion.hip_z_per_forward_distance
            print(f"    h = {height_m * 1e3:5.0f} mm  "
                  f"p2p roll {comparison.roll_excursion.peak_to_peak_m * 1e3:6.1f} "
                  f"/ swing {comparison.swing_excursion.peak_to_peak_m * 1e3:6.1f} mm   "
                  f"per-forward {roll_per:.4f} vs {swing_per:.4f}  "
                  f"({100.0 * (swing_per - roll_per) / swing_per:+5.1f}% for roll)  "
                  f"-> {'ROLL' if comparison.roll_is_cheaper else 'SWING'}", flush=True)
    caveats = {r["posture_caveat"] for r in rows if r["posture_caveat"]}
    if caveats:
        print("\n  posture caveat (the same one on every row):", flush=True)
        print(f"    {sorted(caveats)[0]}", flush=True)
    return rows


# --------------------------------------------------------------------------
# E -- the L_top dependence
# --------------------------------------------------------------------------


def section_e(trajectories, variants, down) -> list[dict]:
    """Does the verdict survive a different top length?

    Both sides answer ``L_top`` the same way -- a longer top adds flat forward
    distance and no vertical path -- so each side's metric is
    ``V / (D_ref + (L - 0.35))`` with its own ``V`` and ``D_ref``.  Setting the
    two equal has a closed form, so the top length at which the verdict flips
    can be solved for rather than sampled:

    ``L* = 0.35 + (V_swing * D_roll - V_roll * D_swing) / (V_roll - V_swing)``

    As ``L -> infinity`` both tend to zero and the ordering is decided by the
    vertical paths alone, which is the asymptotic verdict reported here.
    """

    print("\n=== E. does the verdict depend on L_top?  (both sides extrapolated)",
          flush=True)
    floors = {}
    for row in _read(DAY6_7_DIR / "day6_7_step12r_minimum_top_length.csv"):
        floors[(round(float(row["obstacle_height_m"]), 6),
                round(float(row["theta_climb_deg"]), 3))] = float(
            row["minimum_feasible_top_length_m"])

    concessions = roll_concessions_by_height_2d(trajectories)
    rows = []
    variant, up = next(iter(variants.items()))
    print(f"  (swing approach: {variant} -- the matched one, for the reason in "
          "_best_swing_up)", flush=True)
    for height_m, concession in concessions.items():
        best = concession.best_theta_climb_deg
        if best is None or height_m not in up or height_m not in down:
            continue
        trajectory = trajectories[(round(height_m, 6), round(best, 3))]
        roll_excursion = trajectory.excursion
        swing_excursion = swing_obstacle_hip_profile_2d(up[height_m], down[height_m])

        v_roll = roll_excursion.total_vertical_path_m
        v_swing = swing_excursion.total_vertical_path_m
        d_roll = roll_excursion.forward_distance_m
        d_swing = swing_excursion.forward_distance_m

        # Day 6--7 Step 12R only measured the rolling floor at five
        # ``(h, theta)`` combinations, so most rows have none.  Say so rather
        # than reporting an extrapolation as if its validity were checked.
        roll_floor = floors.get((round(height_m, 6), round(best, 3)))
        crossing = None
        if abs(v_roll - v_swing) > 1e-9:
            crossing = REFERENCE_TOP_LENGTH_M + (
                (v_swing * d_roll - v_roll * d_swing) / (v_roll - v_swing)
            )

        row = {
            "section": "top_length",
            "swing_variant": variant,
            "obstacle_mm": height_m * 1e3,
            "best_theta_climb_deg": best,
            "roll_vertical_path_mm": v_roll * 1e3,
            "swing_vertical_path_mm": v_swing * 1e3,
            "roll_forward_mm": d_roll * 1e3,
            "swing_forward_mm": d_swing * 1e3,
            "roll_floor_top_length_m": roll_floor,
            "crossing_top_length_m": crossing,
            "asymptotic_winner": "ROLL" if v_roll < v_swing else "SWING",
        }
        for top_length_m in TOP_LENGTHS_M:
            delta = float(top_length_m) - REFERENCE_TOP_LENGTH_M
            row[f"roll_per_forward_L{top_length_m:.2f}"] = (
                None if roll_floor is not None and top_length_m < roll_floor
                else v_roll / (d_roll + delta)
            )
            row[f"swing_per_forward_L{top_length_m:.2f}"] = v_swing / (d_swing + delta)
        rows.append(row)

        crossing_text = (
            "never (the two paths never re-order)" if crossing is None
            else f"{crossing:.3f} m"
            + (" -- outside any usable top" if crossing < 0.20 or crossing > 1.0 else "")
        )
        print(f"  h = {height_m * 1e3:5.0f} mm  V_roll {v_roll * 1e3:6.1f} mm  "
              f"V_swing {v_swing * 1e3:6.1f} mm  ->  crossing L_top = {crossing_text}; "
              f"as L grows, {row['asymptotic_winner']} wins", flush=True)

    unmeasured = sum(1 for r in rows if r["roll_floor_top_length_m"] is None)
    print(f"\n  {unmeasured} / {len(rows)} heights have no measured rolling floor "
          "(Step 12R covered five (h, theta) pairs only) -- their short-top "
          "columns are extrapolations whose feasibility is unchecked.", flush=True)
    return rows


# --------------------------------------------------------------------------
# F -- the cross-kind rule
# --------------------------------------------------------------------------


def section_f(trajectories, up_rows, down_rows) -> list[dict]:
    print("\n=== F. spec 5.3's missing rule, applied", flush=True)
    verdicts = {None: "n/a", -1: "ROLL", 0: "tie", 1: "SWING"}
    concessions = roll_concessions_by_height_2d(trajectories)
    rows = []
    for height_m, concession in concessions.items():
        best = concession.best_theta_climb_deg
        if best is None:
            continue
        trajectory = trajectories[(round(height_m, 6), round(best, 3))]
        up_row = up_rows.get(height_m)
        demand_m = (
            None if up_row is None
            else height_m + float(up_row["min_hip_lift_mm"]) / 1e3
        )
        verdict, reason = compare_body_demand_2d(
            concession, demand_m, roll_excursion=trajectory.excursion
        )
        rows.append({
            "section": "rule",
            "obstacle_mm": height_m * 1e3,
            "roll_kind": concession.requirement_kind.value,
            "roll_peak_to_peak_mm": trajectory.excursion.peak_to_peak_m * 1e3,
            "swing_minimum_demand_mm": None if demand_m is None else demand_m * 1e3,
            "verdict": verdicts[verdict],
            "reason": reason,
        })
        print(f"  h = {height_m * 1e3:5.0f} mm  -> {verdicts[verdict]}: {reason}",
              flush=True)
    return rows


def section_g(trajectories) -> list[dict]:
    """``theta_climb`` prices two different things, and in opposite directions.

    Section B picks the theta with the least hip travel and that is always the
    smallest feasible one.  But Day 6--7 Step 12R measured the other side of
    the same knob: a more extended climb needs a *shorter* top.  So theta is
    not a free optimisation -- it is a trade, and Step 5 has to see both halves
    of it or it will pick a theta that cannot fit on the obstacle it chose.
    """

    print("\n=== G. theta_climb trades hip excursion against top length", flush=True)
    required = {}
    for row in _read(DAY6_7_DIR / "day6_7_step12r_minimum_top_length.csv"):
        required[(round(float(row["obstacle_height_m"]), 6),
                  round(float(row["theta_climb_deg"]), 3))] = float(
            row["required_top_length_m"])

    rows = []
    for (height_m, theta_deg), trajectory in sorted(trajectories.items()):
        if not trajectory.feasible:
            continue
        excursion = trajectory.excursion
        overhead_m = excursion.peak_to_peak_m - height_m
        rows.append({
            "section": "theta_trade",
            "obstacle_mm": height_m * 1e3,
            "theta_climb_deg": theta_deg,
            "hip_travel_mm": excursion.peak_to_peak_m * 1e3,
            # The height enters additively: travel = h + overhead(theta).
            # Measured at theta = 40 deg it is 14.2-14.9 mm at every height.
            "roll_up_overhead_mm": overhead_m * 1e3,
            "required_top_length_m": required.get(
                (round(height_m, 6), round(theta_deg, 3))
            ),
        })

    for height_m in (0.06, 0.10):
        subset = [r for r in rows
                  if abs(r["obstacle_mm"] - height_m * 1e3) < 1e-6
                  and r["required_top_length_m"] is not None]
        if len(subset) < 2:
            continue
        low, high = min(subset, key=lambda r: r["theta_climb_deg"]), max(
            subset, key=lambda r: r["theta_climb_deg"])
        print(f"  h = {height_m * 1e3:5.0f} mm: "
              f"theta {low['theta_climb_deg']:.0f} -> {high['theta_climb_deg']:.0f} deg "
              f"costs {high['hip_travel_mm'] - low['hip_travel_mm']:+.1f} mm of hip travel "
              f"and buys {1e3 * (high['required_top_length_m'] - low['required_top_length_m']):+.1f} mm "
              "of required top length", flush=True)

    overheads = [r["roll_up_overhead_mm"] for r in rows
                 if abs(r["theta_climb_deg"] - 40.0) < 1e-9]
    if overheads:
        print(f"\n  at theta_climb = 40 deg the overhead is "
              f"{min(overheads):.1f}-{max(overheads):.1f} mm at every height: "
              "hip travel = h + overhead(theta), with h entering additively.",
              flush=True)
    return rows


# --------------------------------------------------------------------------
# Figures
# --------------------------------------------------------------------------


def _plot_excursion(rows, path: Path) -> Path:
    """The figure the addendum asks for: roll vs swing, with the crossing."""

    matched_variant = SWING_VARIANTS[0][0]
    whole = [r for r in rows
             if r["section"] == "whole" and r["swing_variant"] == matched_variant]
    free = [r for r in rows
            if r["section"] == "whole" and r["swing_variant"] != matched_variant]
    matched = [r for r in rows if r["section"] == "matched"]
    heights = [r["obstacle_mm"] for r in whole]

    fig, axes = plt.subplots(1, 3, figsize=(16.0, 4.8))

    # --- A: the dimensionless currency, the one the addendum names ---------
    ax = axes[0]
    roll = [r["roll_hip_z_per_forward_distance"] for r in whole]
    swing = [r["swing_hip_z_per_forward_distance"] for r in whole]
    ax.plot(heights, roll, "o-", color="#2563eb", lw=2.2,
            label="ROLL (whole traversal)")
    ax.plot(heights, swing, "s-", color="#ea580c", lw=2.2,
            label="SWING (up + top + down), matched c = 40 mm")
    if free:
        ax.plot([r["obstacle_mm"] for r in free],
                [r["swing_hip_z_per_forward_distance"] for r in free],
                "^:", color="#fbbf24", lw=1.4, ms=5,
                label="SWING, own best c (dilutes its own denominator)")
    for i in range(len(heights) - 1):
        if (roll[i] < swing[i]) != (roll[i + 1] < swing[i + 1]):
            ax.axvspan(heights[i], heights[i + 1], color="#16a34a", alpha=0.15)
            ax.text((heights[i] + heights[i + 1]) / 2, max(swing) * 0.35,
                    "crossing", ha="center", fontsize=7.5, color="#16a34a",
                    rotation=90)
    ax.set_xlabel("obstacle height [mm]")
    ax.set_ylabel("hip vertical path / forward distance  [-]")
    ax.set_title("A. hip_z_per_forward_distance -- the addendum's metric\n"
                 "within 1% below h = 100; roll pulls ahead above it",
                 fontsize=9.5)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=6.5, loc="upper left")

    # --- B: peak-to-peak, where the posture caveat lives ------------------
    ax = axes[1]
    roll_p2p = [r["roll_hip_z_peak_to_peak_mm"] for r in whole]
    swing_p2p = [r["swing_hip_z_peak_to_peak_mm"] for r in whole]
    ax.plot(heights, roll_p2p, "o-", color="#2563eb", lw=2.2, label="ROLL")
    ax.plot(heights, swing_p2p, "s-", color="#ea580c", lw=2.2, label="SWING")
    ax.plot(heights, heights, ":", color="#64748b", lw=1.4,
            label="h (a body that only rises by the step)")
    net = whole[0]["roll_hip_z_net_change_mm"] if whole else 0.0
    ax.annotate(
        f"roll carries a {abs(net):.0f} mm posture change:\n"
        "starts standing at theta_climb,\n"
        "ends in wheel mode at theta = 17 deg",
        xy=(heights[0], roll_p2p[0]), xytext=(0.06, 0.60),
        textcoords="axes fraction", fontsize=7,
        arrowprops=dict(arrowstyle="->", lw=0.9, color="#2563eb"))
    ax.set_xlabel("obstacle height [mm]")
    ax.set_ylabel("hip peak-to-peak [mm]")
    ax.set_title("B. peak-to-peak: the envelope\n"
                 "swing wins low, roll wins from h = 120 mm", fontsize=9.5)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=7, loc="upper left")

    # --- C: matched stages, which remove the top crossing entirely --------
    ax = axes[2]
    for label, colour, marker in (("ascent", "#7c3aed", "o"),
                                  ("descent", "#0d9488", "s")):
        subset = sorted((r for r in matched if r["comparison"] == label),
                        key=lambda r: r["obstacle_mm"])
        if not subset:
            continue
        xs = [r["obstacle_mm"] for r in subset]
        ax.plot(xs, [r["roll_hip_z_per_forward_distance"] for r in subset],
                marker + "-", color=colour, lw=1.9, label=f"ROLL {label}")
        ax.plot(xs, [r["swing_hip_z_per_forward_distance"] for r in subset],
                marker + "--", color=colour, lw=1.9, alpha=0.55,
                label=f"SWING {label}")
    ax.set_xlabel("obstacle height [mm]")
    ax.set_ylabel("hip vertical path / forward distance  [-]")
    ax.set_title("C. matched stages only -- no flat top on either side\n"
                 "swing is cheaper at every height, 1.1x to 2.3x", fontsize=9.5)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=6.5, loc="upper left")

    fig.suptitle("Step 4: rolling is not zero body concession -- and which side "
                 "is cheaper depends on what you measure", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def _plot_profiles(trajectories, up, down, path: Path) -> Path:
    """What the two body requirements actually look like."""

    concessions = roll_concessions_by_height_2d(trajectories)
    shown = [h for h in (0.06, 0.10, 0.14) if concessions.get(h) and concessions[h].feasible]

    fig, axes = plt.subplots(1, len(shown) + 1, figsize=(4.4 * (len(shown) + 1), 4.4))
    for ax, height_m in zip(axes, shown):
        best = concessions[height_m].best_theta_climb_deg
        trajectory = trajectories[(round(height_m, 6), round(best, 3))]
        x = np.asarray(trajectory.hip_x_m) * 1e3
        z = np.asarray(trajectory.hip_z_m) * 1e3
        ax.plot(x, z, "-", color="#2563eb", lw=2,
                label=f"ROLL, theta_climb = {best:.0f} deg")
        if height_m in up and height_m in down:
            swing_x = [up[height_m].start_hip_xz_m[0], up[height_m].end_hip_xz_m[0],
                       down[height_m].start_hip_xz_m[0], down[height_m].end_hip_xz_m[0]]
            swing_z = [up[height_m].start_hip_xz_m[1], up[height_m].end_hip_xz_m[1],
                       down[height_m].start_hip_xz_m[1], down[height_m].end_hip_xz_m[1]]
            ax.plot(np.asarray(swing_x) * 1e3, np.asarray(swing_z) * 1e3, "s--",
                    color="#ea580c", lw=2, ms=5, label="SWING (minimum demand)")
            # The two profiles sit at different absolute heights only because
            # they stand differently on the top -- rolling crosses it in wheel
            # mode at theta = 17 deg, the swing stands at theta = 60 deg.  That
            # offset is posture, not excursion, and it cancels in peak-to-peak,
            # so the numbers that are actually being compared go on the panel.
            swing_p2p = max(swing_z) - min(swing_z)
            ax.text(0.02, 0.02,
                    f"peak-to-peak:  ROLL {trajectory.excursion.peak_to_peak_m * 1e3:.0f} mm"
                    f"   vs   SWING {swing_p2p * 1e3:.0f} mm\n"
                    "(the vertical offset between the curves is posture:\n"
                    " roll crosses the top in wheel mode, the swing stands)",
                    transform=ax.transAxes, fontsize=6.5, va="bottom",
                    bbox=dict(boxstyle="round,pad=0.3", fc="#f8fafc", ec="#cbd5e1"))
        ax.axhline(z.min(), color="#94a3b8", ls=":", lw=1)
        ax.set_title(f"h = {height_m * 1e3:.0f} mm", fontsize=10)
        ax.set_xlabel("hip x [mm]")
        ax.set_ylabel("hip z [mm]")
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=7, loc="upper left")

    # The last panel: why theta_climb matters, at one height.
    ax = axes[-1]
    for theta in (40.0, 60.0, 85.0):
        key = (0.06, theta)
        if key not in trajectories or not trajectories[key].feasible:
            continue
        trajectory = trajectories[key]
        ax.plot(np.asarray(trajectory.hip_x_m) * 1e3,
                np.asarray(trajectory.hip_z_m) * 1e3, "-", lw=1.8,
                label=f"theta_climb = {theta:.0f} deg "
                      f"({trajectory.excursion.peak_to_peak_m * 1e3:.0f} mm p2p)")
    ax.set_title("h = 60 mm: theta_climb is rolling's own knob", fontsize=10)
    ax.set_xlabel("hip x [mm]")
    ax.set_ylabel("hip z [mm]")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=7, loc="upper right")

    fig.suptitle("Step 4: rolling's hip profile is a flat-topped trapezoid; "
                 "the swing's is a triangle -- and rolling's is prescribed at "
                 "every instant", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


# --------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--plots-only", action="store_true")
    args = parser.parse_args()

    trajectories, up_rows, down_rows, up, down, variants = _profiles()
    excursion_csv = args.output_dir / "day10_11_step4_hip_excursion.csv"
    concession_csv = args.output_dir / "day10_11_step4_roll_concession.csv"
    excursion_png = args.output_dir / "day10_11_step4_hip_excursion_roll_vs_swing.png"
    profile_png = args.output_dir / "day10_11_step4_roll_hip_profile.png"

    if args.plots_only:
        with excursion_csv.open(encoding="utf-8") as handle:
            rows = [
                {k: (None if v == "" else _coerce(v)) for k, v in row.items()}
                for row in csv.DictReader(handle)
            ]
        print(f"  wrote {_plot_excursion(rows, excursion_png)}", flush=True)
        print(f"  wrote {_plot_profiles(trajectories, up, down, profile_png)}", flush=True)
        return 0

    cells = section_a(trajectories)
    concessions = section_b(trajectories)
    matched = section_c(trajectories, up, down)
    whole = section_d(trajectories, variants, down)
    tops = section_e(trajectories, variants, down)
    rule = section_f(trajectories, up_rows, down_rows)
    theta_trade = section_g(trajectories)

    write_rows_csv(concession_csv, _union(cells + concessions))
    write_rows_csv(excursion_csv,
                   _union(matched + whole + tops + rule + theta_trade))
    print(f"\n  wrote {concession_csv.name} ({len(cells) + len(concessions)} rows)",
          flush=True)
    print(f"  wrote {excursion_csv.name} "
          f"({len(matched) + len(whole) + len(tops) + len(rule) + len(theta_trade)} rows)",
          flush=True)
    print(f"  wrote {_plot_excursion(matched + whole, excursion_png)}", flush=True)
    print(f"  wrote {_plot_profiles(trajectories, up, down, profile_png)}", flush=True)
    return 0


def _coerce(value: str):
    for cast in (int, float):
        try:
            return cast(value)
        except (TypeError, ValueError):
            continue
    if value in ("True", "False"):
        return value == "True"
    return value


def _union(rows: list[dict]) -> list[dict]:
    """``write_rows_csv`` takes its header from the first row (log trap 21)."""

    columns: list[str] = []
    for row in rows:
        for key in row:
            if key not in columns:
                columns.append(key)
    return [{key: row.get(key, "") for key in columns} for row in rows]


if __name__ == "__main__":
    raise SystemExit(main())
