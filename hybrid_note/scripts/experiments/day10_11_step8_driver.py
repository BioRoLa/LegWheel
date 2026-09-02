"""Day 10--11 Step 8: five terrain cases, positive **and** negative.

The spec's five cases were written against the old five-strategy framing, and
two of those strategies no longer exist.  So the cases are re-cut onto Step 5's
actual regions, keeping what each original case was *for*:

    A  short top, low obstacle   -> #5 wins; both top-landing pairs must fail
    B  top long, h where roll wins  -> #1 wins; #4 feasible but dearer
    C  top long, h where swing wins -> #4 wins; #1 feasible but dearer
    D  the hole                  -> nothing works, and for three different reasons
    E  h = 160 mm                -> the case the spec built for ``#2``

**Case E is the one that matters**, and it is where this step's third
completion criterion dies.  The spec asks for at least one case whose best
strategy is *mixed*, "否則 2x2 沒有被驗證".  Both mixed pairs are blocked, so
no such case exists.  E measures what that costs rather than glossing it: at
``h = 160 mm`` Day 6--7's sweep has ``roll_up`` succeeding at **all ten**
thetas and ``roll_down`` at **none**, which is exactly the terrain
``ROLL_UP + SWING_DOWN`` was invented for.

Every case composes **every** strategy, winner and losers, so a negative claim
rests on a run rather than on a table lookup.

    python3 day10_11_step8_driver.py
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from hybrid_note.scripts.experiments.day10_11_composer_2d import (  # noqa: E402
    compose_2d,
    compose_roll_roll_2d,
)
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (  # noqa: E402
    BLOCKED_PAIRS,
    StrategyId,
    decide_2d,
    load_tables_2d,
)
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (  # noqa: E402
    SegmentKind,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    write_rows_csv,
)

OUTPUT_DIR = Path(__file__).resolve().parents[2] / "notes" / "day10-11"
DAY6_7_DIR = Path(__file__).resolve().parents[2] / "notes" / "day6-7"

LIVE = (StrategyId.ROLL_ROLL, StrategyId.SWING_SWING, StrategyId.SWING_OVER)

#: ``(label, h, L_top, what this case is for)``.
CASES: tuple[tuple[str, float, float, str], ...] = (
    ("A short top, low obstacle", 0.040, 0.050,
     "#5's region: every top-landing pair must fail on top length"),
    ("B roll wins", 0.140, 0.350,
     "#1's region: #4 is feasible here too, and must cost more"),
    ("C swing wins", 0.080, 0.350,
     "#4's region: #1 is feasible here too, and must cost more"),
    ("D the hole", 0.120, 0.150,
     "nothing works -- and the three refusals must not share one reason"),
    ("E the case built for #2", 0.160, 0.350,
     "roll_up works at every theta and roll_down at none: the mixed pair's terrain"),
)

#: Case E.  The theta Day 6--7's sweep takes furthest before the descent fails.
CASE_E_THETA_DEG = 85.0


def _sweep_rows() -> list[dict]:
    path = DAY6_7_DIR / "day6_7_step11r_feasibility_sweep.csv"
    with path.open(encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


# --------------------------------------------------------------------------
# A -- the five cases, every strategy
# --------------------------------------------------------------------------


def section_a(tables):
    print("\n=== A. five cases; every strategy composed, winner and losers",
          flush=True)
    rows, composed = [], {}
    for label, height_m, top_m, purpose in CASES:
        decision = decide_2d(height_m, top_m, tables)
        print(f"\n  {label}: h = {height_m * 1e3:.0f} mm, "
              f"L_top = {top_m * 1e3:.0f} mm", flush=True)
        print(f"    {purpose}", flush=True)
        print(f"    Step 5 picks: "
              f"{decision.winner.value if decision.winner else 'NONE'}", flush=True)
        for strategy in LIVE:
            started = time.perf_counter()
            result = compose_2d(height_m, top_m, tables, strategy=strategy)
            composed[(label, strategy)] = result
            verdict = "COMPOSED" if result.composed else "refused"
            body = (
                "--" if result.sequence is None
                else f"{decision_body(decision, strategy):.1f} mm"
            )
            print(f"      {strategy.value:24s} {verdict:9s} body {body:>10s}  "
                  f"({time.perf_counter() - started:.0f} s)"
                  + ("" if result.composed else f"  <- {result.refusal[:80]}"),
                  flush=True)
            rows.append({
                "case": label, "obstacle_mm": height_m * 1e3,
                "top_length_m": top_m, "purpose": purpose,
                "is_winner": decision.winner is strategy,
                # Step 5's number, carried here so the case CSV can be read on
                # its own: what the composer reports is whether the plan exists,
                # not what the rule priced it at.
                "body_deviation_mm": decision_body(decision, strategy),
                **result.as_dict(),
            })
        for strategy in BLOCKED_PAIRS:
            result = compose_2d(height_m, top_m, tables, strategy=strategy)
            rows.append({
                "case": label, "obstacle_mm": height_m * 1e3,
                "top_length_m": top_m, "purpose": purpose,
                "is_winner": False, "body_deviation_mm": float("nan"),
                **result.as_dict(),
            })
    return rows, composed


def decision_body(decision, strategy) -> float:
    cell = next(c for c in decision.cells if c.strategy is strategy)
    return float("nan") if cell.body_deviation_m is None else cell.body_deviation_m * 1e3


# --------------------------------------------------------------------------
# B -- negatives have to name a reason
# --------------------------------------------------------------------------


def section_b(rows) -> None:
    print("\n=== B. every negative names a reason", flush=True)
    negatives = [r for r in rows if not r["composed"]]
    vague = [r for r in negatives if not (r["refusal"] or "").strip()]
    print(f"  {len(negatives)} refusals, {len(vague)} of them without a reason",
          flush=True)
    by_limiter: dict[str, int] = {}
    for row in negatives:
        head = (row["refusal"] or "").split(":")[0][:60]
        by_limiter[head] = by_limiter.get(head, 0) + 1
    for reason, count in sorted(by_limiter.items(), key=lambda kv: -kv[1]):
        print(f"    {count:2d}  {reason}", flush=True)

    # The hole's three refusals: compare the **whole** reason, not its first
    # clause.  #1 and #4 both open "infeasible (top length)" but one is short of
    # ``L_transition`` and the other cannot fit a landing plus a take-off --
    # different walls that happen to share a limiter name.
    hole = [r for r in rows if r["case"].startswith("D") and not r["composed"]
            and r["strategy"] not in {s.value for s in BLOCKED_PAIRS}]
    distinct = {(r["refusal"] or "").strip() for r in hole}
    limiters = {(r["refusal"] or "").split(":")[0] for r in hole}
    print(f"\n  the hole: {len(hole)} refusals, {len(distinct)} distinct reasons "
          f"across {len(limiters)} limiter name(s)", flush=True)
    for row in hole:
        print(f"    {row['strategy']:24s} {row['refusal']}", flush=True)
    print(f"  -> {'not one wall' if len(distinct) > 1 else 'ONE wall'}: "
          "two of them share a limiter name and still fail for different "
          "reasons.", flush=True)


# --------------------------------------------------------------------------
# C -- the 2x2 verdict
# --------------------------------------------------------------------------


def section_c(rows) -> list[dict]:
    print("\n=== C. the third completion criterion: is any case's best mixed?",
          flush=True)
    winners = {
        r["case"]: r["strategy"] for r in rows
        if r["is_winner"] and r["composed"]
    }
    mixed = {
        StrategyId.ROLL_SWING.value, StrategyId.SWING_ROLL.value,
    }
    found = [case for case, strategy in winners.items() if strategy in mixed]
    for case, strategy in sorted(winners.items()):
        print(f"    {case:28s} -> {strategy}", flush=True)
    print(f"\n  cases whose best is a MIXED pair: {len(found)}", flush=True)
    print("  -> the criterion cannot be met, because both mixed pairs are "
          "DIRECT_HANDOFF_INFEASIBLE.\n     The 2x2's off-diagonal is empty "
          "**for the current primitive set** -- not for the robot\n     "
          "(spec 5.5); TOP_REPOSITION under multi-leg support is the route back "
          "(spec 5.6).", flush=True)
    return [{
        "claim": "at least one case's best strategy is mixed",
        "outcome": "cannot be met",
        "detail": (
            "#2 and #3 are DIRECT_HANDOFF_INFEASIBLE (Step 3 D/E and Step 2b); "
            "both surviving pairs are homogeneous and #5 is not a pair at all. "
            "Neither is PHYSICALLY_INFEASIBLE -- see spec 5.5 / 5.6."
        ),
    }]


# --------------------------------------------------------------------------
# D -- case E: what the refutation of #2 costs
# --------------------------------------------------------------------------


def section_d(tables) -> list[dict]:
    print("\n=== D. case E: what refuting #2 actually costs", flush=True)
    sweep = [r for r in _sweep_rows() if abs(float(r["obstacle_height_m"]) - 0.16) < 1e-9]
    up_ok = sum(1 for r in sweep if r["roll_up_success"] == "True")
    down_ok = sum(1 for r in sweep if r["roll_down_success"] == "True")
    print(f"  Day 6-7 at h = 160 mm: roll_up succeeds at {up_ok}/{len(sweep)} thetas, "
          f"roll_down at {down_ok}/{len(sweep)}", flush=True)
    print("  -> this is precisely the terrain ROLL_UP + SWING_DOWN was invented for.",
          flush=True)

    print(f"\n  pricing the ascent that works but cannot be used "
          f"(theta = {CASE_E_THETA_DEG:.0f} deg):", flush=True)
    partial = compose_roll_roll_2d(
        0.160, 0.350, theta_climb_deg=CASE_E_THETA_DEG,
        approach_clearance_m=0.04, keep_partial=True,
    )
    rows = []
    if partial.sequence is None:
        print(f"    could not even build a partial sequence: {partial.refusal}",
              flush=True)
        return rows
    roll_up = [
        s for s in partial.sequence.segments if s.kind is SegmentKind.ROLL_UP
    ]
    profile = np.concatenate([
        s.body_requirement.hip_z_profile_m for s in roll_up
    ]) if roll_up else np.array([])
    roll_up_p2p_m = float(profile.max() - profile.min()) if profile.size else float("nan")

    with (OUTPUT_DIR / "day10_11_step2_swing_onto_sweep.csv").open(
        encoding="utf-8"
    ) as handle:
        step2 = [
            r for r in csv.DictReader(handle)
            if r["feasible"] == "True" and abs(float(r["obstacle_mm"]) - 160.0) < 1e-9
        ]
    lift_mm = min(float(r["min_hip_lift_mm"]) for r in step2)
    swing_up_p2p_mm = 160.0 + lift_mm

    print(f"    ROLL_UP  hip peak-to-peak  {roll_up_p2p_m * 1e3:7.1f} mm  "
          f"({len(roll_up)} segments, {sum(s.frames.frame_count for s in roll_up)} frames)",
          flush=True)
    print(f"    SWING_UP hip peak-to-peak  {swing_up_p2p_mm:7.1f} mm  "
          f"(= h + min_hip_lift {lift_mm:.0f} mm)", flush=True)
    print(f"    -> rolling up would ask the body for "
          f"{swing_up_p2p_mm / (roll_up_p2p_m * 1e3):.1f}x less, and it works at "
          f"every theta.\n       It cannot be used because the pair it belongs to "
          "will not chain, and\n       the reason has nothing to do with the "
          "ascent.",
          flush=True)
    print(f"    the traversal stopped at: {partial.refusal}", flush=True)

    rows.append({
        "case": "E the case built for #2",
        "obstacle_mm": 160.0,
        "roll_up_thetas_ok": f"{up_ok}/{len(sweep)}",
        "roll_down_thetas_ok": f"{down_ok}/{len(sweep)}",
        "roll_up_p2p_mm": roll_up_p2p_m * 1e3,
        "swing_up_p2p_mm": swing_up_p2p_mm,
        "ratio_swing_over_roll": swing_up_p2p_mm / (roll_up_p2p_m * 1e3),
        "partial_stopped_at": partial.refusal,
        "note": (
            "the ascent is real work the leg does; the pair is refused for a "
            "descent-side reason (Step 3 D/E)."
        ),
    })
    return rows


# --------------------------------------------------------------------------


def _plot(rows, path: Path) -> Path:
    fig, axes = plt.subplots(1, 2, figsize=(14.5, 5.0))

    ax = axes[0]
    cases = [c[0] for c in CASES]
    colours = {
        StrategyId.ROLL_ROLL.value: "#2563eb",
        StrategyId.SWING_SWING.value: "#ea580c",
        StrategyId.SWING_OVER.value: "#16a34a",
    }
    width = 0.26
    for offset, strategy in zip((-width, 0.0, width), LIVE):
        heights, colour = [], colours[strategy.value]
        for case in cases:
            row = next(
                (r for r in rows
                 if r["case"] == case and r["strategy"] == strategy.value),
                None,
            )
            heights.append(1.0 if row and row["composed"] else 0.0)
        positions = np.arange(len(cases)) + offset
        ax.bar(positions, heights, width * 0.9, color=colour,
               label=strategy.value)
        for x, value in zip(positions, heights):
            if value == 0.0:
                ax.plot(x, 0.04, "x", ms=7, color="#dc2626")
    ax.set_xticks(np.arange(len(cases)), [c.split()[0] for c in cases])
    ax.set_yticks([0, 1], ["refused", "composed"])
    ax.set_ylim(-0.05, 1.35)
    ax.set_xlabel("case")
    ax.set_title("A-E: every strategy composed at every case\n"
                 "x = refused, and each refusal names a reason", fontsize=10)
    ax.legend(fontsize=7, loc="upper center", ncol=3)
    ax.grid(True, alpha=0.2, axis="y")

    # --- the 2x2, with its off-diagonal struck out --------------------------
    ax = axes[1]
    ax.set_xlim(0, 2)
    ax.set_ylim(0, 2)
    labels = {
        (0, 1): ("#1 ROLL + ROLL", "#2563eb", True),
        (1, 1): ("#2 ROLL + SWING", "#94a3b8", False),
        (0, 0): ("#3 SWING + ROLL", "#94a3b8", False),
        (1, 0): ("#4 SWING + SWING", "#ea580c", True),
    }
    for (col, row_i), (label, colour, alive) in labels.items():
        ax.add_patch(plt.Rectangle((col, row_i), 1, 1, facecolor=colour,
                                   alpha=0.85 if alive else 0.35,
                                   edgecolor="white", lw=2))
        ax.text(col + 0.5, row_i + 0.58, label, ha="center", va="center",
                fontsize=9.5, color="white", fontweight="bold")
        ax.text(col + 0.5, row_i + 0.36,
                "composed" if alive else "HAND-OVER\nBLOCKED",
                ha="center", va="center", fontsize=8,
                color="white" if alive else "#450a0a")
    # Inset, so each cross belongs to its own cell rather than reading as two
    # diagonals drawn across the whole grid.
    for col, row_i in ((1, 1), (0, 0)):
        lo, hi = 0.08, 0.92
        ax.plot([col + lo, col + hi], [row_i + lo, row_i + hi], "-",
                color="#dc2626", lw=3)
        ax.plot([col + lo, col + hi], [row_i + hi, row_i + lo], "-",
                color="#dc2626", lw=3)
    ax.set_xticks([0.5, 1.5], ["ROLL_UP", "SWING_UP"])
    ax.set_yticks([0.5, 1.5], ["SWING_DOWN", "ROLL_DOWN"])
    ax.set_title("the 2x2 after Steps 2b / 3 / 8\n"
                 "the off-diagonal is empty for the CURRENT primitive set\n"
                 "(DIRECT_HANDOFF_INFEASIBLE, not physically impossible)",
                 fontsize=9)

    fig.suptitle("Step 8: five terrain cases, and the criterion the 2x2 cannot meet",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def _union(rows: list[dict]) -> list[dict]:
    columns: list[str] = []
    for row in rows:
        for key in row:
            if key not in columns:
                columns.append(key)
    return [{key: row.get(key, "") for key in columns} for row in rows]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    parser.add_argument(
        "--plots-only", action="store_true",
        help="redo the figure and the pure-function sections from the CSV; "
             "sections A and D need planner runs and are skipped",
    )
    args = parser.parse_args()

    figure = args.output_dir / "day10_11_step8_cases.png"
    if args.plots_only:
        with (args.output_dir / "day10_11_step8_cases.csv").open(
            encoding="utf-8"
        ) as handle:
            rows = [
                {k: (v == "True" if v in ("True", "False") else v)
                 for k, v in row.items()}
                for row in csv.DictReader(handle)
            ]
        section_b(rows)
        section_c(rows)
        print(f"  wrote {_plot(rows, figure)}", flush=True)
        return 0

    tables = load_tables_2d(
        args.output_dir, DAY6_7_DIR,
        swing_over_csv=args.output_dir / "day10_11_step5_swing_over.csv",
    )
    rows, _ = section_a(tables)
    section_b(rows)
    verdict = section_c(rows)
    cost = section_d(tables)

    write_rows_csv(args.output_dir / "day10_11_step8_cases.csv", _union(rows))
    write_rows_csv(
        args.output_dir / "day10_11_step8_two_by_two_verdict.csv", _union(verdict)
    )
    if cost:
        write_rows_csv(
            args.output_dir / "day10_11_step8_mixed_pair_cost.csv", _union(cost)
        )
    print(f"\n  wrote day10_11_step8_cases.csv ({len(rows)} rows)", flush=True)
    print(f"  wrote {_plot(rows, figure)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
