"""Day 10--11 Step 6: write the two real cases into the schema and check them.

Three sections, all cheap -- the schema is a contract, not a sweep.

    A  Day 6--7 Step 10R's full traversal, wheel-mode segment included
    B  one Day 8--9 swing, shaping knobs included
    C  the sampling parameters, shown to be load-bearing rather than asserted

    python3 day10_11_step6_driver.py
    python3 day10_11_step6_driver.py --plots-only
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from legwheel.planners.hybrid import HipPose2D  # noqa: E402
from hybrid_note.scripts.experiments.cartesian_swing_contract_2d import (  # noqa: E402
    HipTrajectory2D,
    build_swing_request_2d,
    flat_to_flat_swing_request_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_planner_2d import (  # noqa: E402
    generate_swing_2d,
)
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (  # noqa: E402
    RollingMode,
    SegmentKind,
)
from hybrid_note.scripts.experiments.day10_11_sequence_builders_2d import (  # noqa: E402
    coverage_report_2d,
    handoff_report_2d,
    segment_from_swing_plan_2d,
    sequence_from_traversal_frames_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    SharedTerrainSpec2D,
    approach_hip_x_for_clearance_2d,
    standing_scene_2d,
    write_rows_csv,
)
from hybrid_note.scripts.experiments.right_up_left_down_sweep_2d import (  # noqa: E402
    seam_bridge_for_sampling_m,
)

OUTPUT_DIR = Path(__file__).resolve().parents[2] / "notes" / "day10-11"
DAY6_7_DIR = Path(__file__).resolve().parents[2] / "notes" / "day6-7"

#: Day 6--7's ``ObstacleSpec2D`` default, and what Step 10R ran at.  Recorded
#: here because the frame CSV does not carry it -- which is itself an instance
#: of the problem this step exists to fix.
STEP10R_ARC_SAMPLES = 241

#: Section B.  A cell Step 2 recorded as feasible, so the swing being written
#: into the schema is one the sweep actually accepted rather than a fresh guess.
SWING_HEIGHT_M = 0.16
SWING_CLEARANCE_M = 0.04
SWING_LIFT_M = 0.060
SWING_LIFTOFF_RISE_M = 0.030


def _traversal_rows() -> list[dict]:
    path = DAY6_7_DIR / "day6_7_step10r_full_traversal_frames.csv"
    with path.open(encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


# --------------------------------------------------------------------------
# A -- the rolling traversal
# --------------------------------------------------------------------------


def section_a():
    print("\n=== A. Day 6-7 Step 10R -> MotionSequence2D", flush=True)
    rows = _traversal_rows()
    sequence = sequence_from_traversal_frames_2d(
        rows,
        terrain_id="day6_7_obstacle",
        source_id="day6_7_step10r_full_traversal_frames.csv",
        arc_samples=STEP10R_ARC_SAMPLES,
    )
    print(f"  {len(sequence.segments)} segments", flush=True)
    for index, segment in enumerate(sequence.segments):
        rolling = segment.rolling
        theta_step = segment.sampling.theta_step_rad
        print(f"    {index} {segment.kind.value:17s} {segment.phase_label:30s} "
              f"n={segment.frames.frame_count:3d}  {rolling.mode.value:13s}  "
              f"beta {np.rad2deg(rolling.beta_sweep_rad):6.1f} deg  "
              f"theta step "
              f"{'--' if theta_step is None else f'{np.rad2deg(theta_step):+.1f} deg'}",
              flush=True)

    report = coverage_report_2d(sequence, rows)
    print(f"\n  coverage: {report.as_dict()}", flush=True)
    if not report.lossless:
        print("  *** NOT LOSSLESS ***", flush=True)

    print("\n  hand-overs (contact may jump; joints may not):", flush=True)
    handoffs = handoff_report_2d(sequence)
    for handoff in handoffs:
        row = handoff.as_dict()
        print(f"    -> {row['to']:44s} rim_change={str(row['rim_changed']):5s} "
              f"contact {row['contact_jump_mm']:7.1f} mm  "
              f"alpha {row['alpha_jump_deg']:7.2f}  theta {row['theta_jump_deg']:6.2f}  "
              f"beta {row['beta_jump_deg']:6.2f}", flush=True)
    print(f"\n  largest joint jump: theta "
          f"{max(abs(np.rad2deg(h.theta_jump_rad)) for h in handoffs):.2f} deg, "
          f"beta {max(abs(np.rad2deg(h.beta_jump_rad)) for h in handoffs):.2f} deg",
          flush=True)
    print(f"  largest contact jump: "
          f"{max(h.contact_jump_m for h in handoffs) * 1e3:.1f} mm "
          "(a rim transfer moves the contact to a different part of the wheel)",
          flush=True)
    print(f"\n  total duration: {sequence.total_duration_s} "
          "(quasi-static -- Day 6-7 never assigned time)", flush=True)
    return sequence, handoffs, rows


# --------------------------------------------------------------------------
# B -- one swing
# --------------------------------------------------------------------------


def section_b():
    print("\n=== B. one Day 8-9 swing -> MotionSegment2D", flush=True)
    spec = SharedTerrainSpec2D(height_m=SWING_HEIGHT_M, top_length_m=0.35,
                               x_start_m=0.10, arc_samples=121)
    theta = float(np.deg2rad(60.0))
    hip_x = approach_hip_x_for_clearance_2d(spec, theta, SWING_CLEARANCE_M)
    start = standing_scene_2d(spec, theta, hip_x_m=hip_x, support_height_m=0.0)
    target = standing_scene_2d(spec, theta, hip_x_m=spec.x_start_m + 0.16,
                               support_height_m=spec.top_z_m)
    base = build_swing_request_2d(start, target, clearance_m=0.03,
                                  swing_duration_s=0.6, sample_count=31)
    request = replace(base, hip_trajectory=HipTrajectory2D(
        start.hip_pose,
        HipPose2D(target.hip_pose.position_world_xz_m + np.array([0.0, SWING_LIFT_M])),
    ))
    plan = generate_swing_2d(request, arc_samples=61,
                             liftoff_rise_m=SWING_LIFTOFF_RISE_M)
    print(f"  plan valid: {plan.valid}", flush=True)
    segment = segment_from_swing_plan_2d(
        plan, request, kind=SegmentKind.SWING_UP,
        source_id="day10_11_step6_swing_up", arc_samples=121, leg_arc_samples=61,
        apex_clearance_m=0.03, liftoff_rise_m=SWING_LIFTOFF_RISE_M,
        touchdown_drop_m=0.0,
    )
    row = segment.as_dict()
    print("\n  the four fields spec 2.5 says no contact state mentions:", flush=True)
    for name in ("liftoff_rise_mm", "touchdown_drop_mm", "apex_clearance_mm",
                 "duration_s"):
        print(f"    {name:20s} {row[name]}", flush=True)
    print("\n  the sampling that makes it reproducible:", flush=True)
    for name in ("arc_samples", "sample_count", "leg_arc_samples",
                 "max_joint_step_deg"):
        print(f"    {name:20s} {row[name]}", flush=True)
    print("\n  endpoints:", flush=True)
    print(f"    start {segment.start_contact.as_dict()}", flush=True)
    print(f"    end   {segment.end_contact.as_dict()}", flush=True)
    print("\n  the landing theta is the IK's output, not the request's input "
          f"(60 deg in, {np.rad2deg(segment.end_contact.theta_rad):.1f} deg out) "
          "-- implementation log trap 16.", flush=True)
    return segment


# --------------------------------------------------------------------------
# C -- the sampling parameters, demonstrated
# --------------------------------------------------------------------------


def section_c() -> list[dict]:
    print("\n=== C. every sampling parameter is load-bearing", flush=True)
    rows: list[dict] = []

    coarse, fine = seam_bridge_for_sampling_m(61), seam_bridge_for_sampling_m(481)
    rows.append({
        "parameter": "arc_samples",
        "what_changes": "what counts as a rim seam",
        "at_low": f"{coarse * 1e3:.2f} mm at 61",
        "at_high": f"{fine * 1e3:.2f} mm at 481",
        "ratio": coarse / fine,
    })
    print(f"  arc_samples        seam bridge {coarse * 1e3:.2f} mm (61) -> "
          f"{fine * 1e3:.2f} mm (481), a factor of {coarse / fine:.1f}", flush=True)

    steps = []
    for sample_count in (16, 31):
        request = flat_to_flat_swing_request_2d(
            step_length_m=0.20, sample_count=sample_count, arc_samples=121
        )
        plan = generate_swing_2d(request, arc_samples=61)
        steps.append(max(
            s.joint_step_rad for s in plan.result.samples
            if s.joint_step_rad is not None
        ))
    rows.append({
        "parameter": "sample_count",
        "what_changes": "the per-sample joint step the limit is checked against",
        "at_low": f"{np.rad2deg(steps[0]):.2f} deg at 16",
        "at_high": f"{np.rad2deg(steps[1]):.2f} deg at 31",
        "ratio": steps[0] / steps[1],
    })
    print(f"  sample_count       max joint step {np.rad2deg(steps[0]):.2f} deg (16) -> "
          f"{np.rad2deg(steps[1]):.2f} deg (31)", flush=True)

    clearances = []
    request = flat_to_flat_swing_request_2d(
        step_length_m=0.20, sample_count=31, arc_samples=121
    )
    for leg_arc_samples in (31, 241):
        plan = generate_swing_2d(request, arc_samples=leg_arc_samples)
        clearances.append(plan.collision.minimum_clearance_m)
    rows.append({
        "parameter": "leg_arc_samples",
        "what_changes": "the minimum clearance the collision sweep reports",
        "at_low": f"{clearances[0] * 1e3:.5f} mm at 31",
        "at_high": f"{clearances[1] * 1e3:.5f} mm at 241",
        "ratio": clearances[0] / clearances[1],
    })
    print(f"  leg_arc_samples    min clearance {clearances[0] * 1e3:.5f} mm (31) -> "
          f"{clearances[1] * 1e3:.5f} mm (241)", flush=True)

    rows.append({
        "parameter": "max_joint_step_rad",
        "what_changes": "the continuity limit -- per SAMPLE, so it is coupled "
                        "to sample_count",
        "at_low": "0.20 rad x 16 samples",
        "at_high": "0.10 rad x 31 samples",
        "ratio": 1.0,
    })
    print("  max_joint_step_rad a per-sample limit, so (0.20, 16) and (0.10, 31) "
          "are the same constraint -- neither field pins it down alone",
          flush=True)
    return rows


# --------------------------------------------------------------------------
# Figure
# --------------------------------------------------------------------------


def _plot(sequence, path: Path) -> Path:
    """The traversal, decomposed the way the schema sees it."""

    rows = _traversal_rows()
    by_index = {int(r["index"]): r for r in rows}
    colours = {
        SegmentKind.APPROACH: "#94a3b8",
        SegmentKind.ROLL_UP: "#2563eb",
        SegmentKind.WHEEL_TRANSITION: "#16a34a",
        SegmentKind.ROLL_DOWN: "#ea580c",
    }

    fig, axes = plt.subplots(1, 2, figsize=(15.0, 5.0),
                             gridspec_kw={"width_ratios": [1.4, 1]})

    ax = axes[0]
    for segment in sequence.segments:
        indices = segment.frames.indices
        hip_x = [float(by_index[i]["hip_x_m"]) * 1e3 for i in indices]
        hip_z = [float(by_index[i]["hip_z_m"]) * 1e3 for i in indices]
        style = "--" if segment.rolling.mode is RollingMode.CORNER_PIVOT else "-"
        ax.plot(hip_x, hip_z, style, lw=3, color=colours[segment.kind])
        ax.plot(hip_x[0], hip_z[0], "o", ms=5, color="white",
                markeredgecolor=colours[segment.kind], markeredgewidth=1.5, zorder=5)
    ax.set_xlabel("hip x [mm]")
    ax.set_ylabel("hip z [mm]")
    ax.set_title("Step 10R written into the schema: 10 segments, 299 frames\n"
                 "dashed = CORNER_PIVOT (alpha pinned, beta sweeping); "
                 "circles = segment starts", fontsize=9.5)
    ax.grid(True, alpha=0.25)
    ax.legend(handles=[Patch(facecolor=c, label=k.value) for k, c in colours.items()],
              fontsize=7.5, loc="upper right")

    ax = axes[1]
    labels, widths, lefts, bar_colours, hatches = [], [], [], [], []
    left = 0
    for segment in sequence.segments:
        labels.append(f"{segment.phase_label}")
        widths.append(segment.frames.frame_count)
        lefts.append(left)
        bar_colours.append(colours[segment.kind])
        hatches.append(
            "//" if segment.rolling.mode is RollingMode.CORNER_PIVOT else ""
        )
        left += segment.frames.frame_count
    positions = np.arange(len(labels))
    for pos, width, colour, hatch in zip(positions, widths, bar_colours, hatches):
        ax.barh(pos, width, color=colour, hatch=hatch, edgecolor="white")
    for pos, width in zip(positions, widths):
        ax.text(width + 1.5, pos, str(width), va="center", fontsize=7.5)
    ax.set_yticks(positions, labels, fontsize=7.5)
    ax.invert_yaxis()
    ax.set_xlabel("frames in the segment")
    ax.set_title("frames per segment\n"
                 "hatched = corner pivot: 72 of 299 frames, which a schema "
                 "with only alpha_range\nwould record as a single static pose",
                 fontsize=9.5)
    ax.grid(True, alpha=0.25, axis="x")

    fig.suptitle("Step 6: the segment schema, applied to the traversal it has "
                 "to represent losslessly", fontsize=11)
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

    rows = _traversal_rows()
    sequence = sequence_from_traversal_frames_2d(
        rows, terrain_id="day6_7_obstacle",
        source_id="day6_7_step10r_full_traversal_frames.csv",
        arc_samples=STEP10R_ARC_SAMPLES,
    )
    figure = args.output_dir / "day10_11_step6_sequence_segments.png"
    if args.plots_only:
        print(f"  wrote {_plot(sequence, figure)}", flush=True)
        return 0

    sequence, handoffs, rows = section_a()
    swing = section_b()
    sampling = section_c()

    segment_rows = sequence.rows() + [
        {"terrain_id": "day10_11_obstacle", "segment_index": len(sequence.segments),
         **swing.as_dict()}
    ]
    columns: list[str] = []
    for row in segment_rows:
        for key in row:
            if key not in columns:
                columns.append(key)
    segments_csv = args.output_dir / "day10_11_step6_sequence_segments.csv"
    write_rows_csv(segments_csv, [{k: r.get(k, "") for k in columns}
                                  for r in segment_rows])

    handoff_csv = args.output_dir / "day10_11_step6_handoff.csv"
    write_rows_csv(handoff_csv, [h.as_dict() for h in handoffs])

    sampling_csv = args.output_dir / "day10_11_step6_sampling_evidence.csv"
    write_rows_csv(sampling_csv, sampling)

    print(f"\n  wrote {segments_csv.name} ({len(segment_rows)} segments)", flush=True)
    print(f"  wrote {handoff_csv.name} ({len(handoffs)} hand-overs)", flush=True)
    print(f"  wrote {sampling_csv.name} ({len(sampling)} parameters)", flush=True)
    print(f"  wrote {_plot(sequence, figure)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
