"""Day 12 Step 0 driver: emit the freeze as data.

Step 0's acceptance is that ``FOOT_RIM_ROLL != WHEEL_ROLL`` can be answered
without reading prose, and that a segment's exit state can be handed to the
next segment.  Both are checked in ``tests/test_day12_segment_contract_2d.py``;
this driver writes the same facts out as tables and one figure, so the notebook
and the paper log can quote measurements rather than assertions.

Outputs (into ``hybrid_note/notes/day12/``):

``day12_step0_segment_semantics.csv``
    every ``SegmentKind`` with the two Day 12 discriminators.

``day12_step0_boundary_evidence.csv``
    every boundary of Day 6--7 Step 10R's traversal, plus the Day 12-shaped
    hand-over onto it, labelled cut or hand-over and measured.

``day12_step0_boundary_evidence.png``
    the same table as a figure: why one tolerance cannot serve both.

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step0_driver.py
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (  # noqa: E402
    WHEEL_MODE_THETA_RAD,
    BodyRequirement2D,
    FrameRef2D,
    MotionSegment2D,
    RollSampling2D,
    RollingContact2D,
    SegmentKind,
)
from hybrid_note.scripts.experiments.day10_11_concession_2d import BodyRequirementKind
from hybrid_note.scripts.experiments.day10_11_sequence_builders_2d import (
    sequence_from_traversal_frames_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import write_rows_csv
from hybrid_note.scripts.experiments.day12_segment_contract_2d import (
    SegmentChain2D,
    boundary_rows_2d,
    entry_state_2d,
    segment_semantics_rows,
)

NOTES = Path(__file__).resolve().parents[2] / "notes"
DAY6_7_FRAMES = (
    NOTES / "day6-7" / "day6_7_step10r_full_traversal_frames.csv"
)
OUT = NOTES / "day12"
SOURCE_ID = "day6_7_step10r_full_traversal_frames.csv"

#: The posture Day 6--7's approach actually stands at.  Read off the traversal
#: below rather than written down, so this driver cannot drift from it.
FLAT_RUN_SOURCE = "day12_step0_flat_run"


def _flat_run_ending_at(entry, *, back_m: float = 0.05) -> MotionSegment2D:
    """A stand-in nominal flat run that ends exactly where the traversal starts.

    Step 1 replaces this with a generated ``FOOT_RIM_ROLL``.  Its only job here
    is to be a **second frame source**, so the hand-over boundary this step
    exists to characterise actually appears.  Its interior is not a claim: the
    body requirement holds the hip flat, which is what a flat foot-rim roll at
    a fixed theta does, and nothing downstream reads it in Step 0.
    """

    x_end = float(entry.point_world_xz_m[0])
    x_start = x_end - back_m
    theta = float(entry.theta_rad)
    return MotionSegment2D(
        kind=SegmentKind.FOOT_RIM_ROLL,
        phase_label="NOMINAL_FLAT_RUN",
        start_contact=entry.__class__(
            rim=entry.rim,
            alpha_rad=entry.alpha_rad,
            point_world_xz_m=(x_start, float(entry.point_world_xz_m[1])),
            surface_id=entry.surface_id,
            theta_rad=theta,
            beta_rad=entry.beta_rad,
            hip_xz_m=(float(entry.hip_xz_m[0]) - back_m, float(entry.hip_xz_m[1])),
        ),
        end_contact=entry,
        sampling=RollSampling2D(
            arc_samples=241, beta_step_rad=float(np.deg2rad(-2.0)), theta_step_rad=None
        ),
        body_requirement=BodyRequirement2D(
            kind=BodyRequirementKind.TRACK,
            x_range_m=(x_start, x_end),
            hip_z_profile_m=np.full(3, float(entry.hip_xz_m[1])),
        ),
        frames=FrameRef2D(source_id=FLAT_RUN_SOURCE, indices=(0, 1, 2)),
        rolling=RollingContact2D(
            rim=entry.rim,
            surface_ids=(entry.surface_id,),
            alpha_range_rad=(entry.alpha_rad, entry.alpha_rad),
            beta_range_rad=(entry.beta_rad, entry.beta_rad),
            theta_range_rad=(theta, theta),
            contact_start_xz_m=(x_start, float(entry.point_world_xz_m[1])),
            contact_end_xz_m=entry.point_world_xz_m,
        ),
    )


def build_chain() -> SegmentChain2D:
    with DAY6_7_FRAMES.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    sequence = sequence_from_traversal_frames_2d(
        rows, terrain_id="day6_7_obstacle", source_id=SOURCE_ID, arc_samples=241
    )
    entry = entry_state_2d(sequence.segments[0])
    return SegmentChain2D(
        leg_id="single_leg_2d",
        segments=(_flat_run_ending_at(entry), *sequence.segments),
        notes=(
            "Day 12 Step 0: a stand-in nominal flat run handed over to Day 6-7 "
            "Step 10R.  Two frame sources, which MotionSequence2D forbids."
        ),
    )


def plot_boundary_evidence(rows: list[dict], path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    idx = [r["boundary_index"] for r in rows]
    is_cut = [r["boundary_kind"] == "cut" for r in rows]
    joint = [max(abs(r["theta_jump_deg"]), abs(r["beta_jump_deg"])) for r in rows]
    hip = [r["hip_jump_mm"] for r in rows]
    contact = [r["contact_jump_mm"] for r in rows]

    fig, axes = plt.subplots(3, 1, figsize=(9.5, 8.0), sharex=True)
    panels = (
        (axes[0], joint, "max |joint jump|  [deg]", None),
        (axes[1], hip, "hip jump  [mm]", None),
        (axes[2], contact, "contact jump  [mm]", None),
    )
    for ax, values, label, _ in panels:
        colours = ["#c44e52" if c else "#4c72b0" for c in is_cut]
        ax.bar(idx, values, color=colours, width=0.65)
        ax.set_ylabel(label)
        ax.grid(axis="y", alpha=0.3)

    # Boundary 0 is the hand-over and it is exactly zero in all three panels --
    # a zero-height bar is invisible, and an invisible bar reads as missing
    # data rather than as the result it is.
    for ax, values in ((axes[0], joint), (axes[1], hip), (axes[2], contact)):
        for i, (value, cut) in enumerate(zip(values, is_cut)):
            if not cut and abs(value) < 1e-9:
                ax.annotate(
                    "0.000\n(exact\nhand-over)", xy=(i, 0.0),
                    xytext=(0, 6), textcoords="offset points",
                    ha="center", va="bottom", fontsize=7, color="#4c72b0",
                )

    axes[0].axhline(2.0, color="k", ls="--", lw=1.0,
                    label="hand-over joint bound (2 deg)")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[1].axhline(10.0, color="k", ls="--", lw=1.0,
                    label="hand-over hip bound (10 mm)")
    axes[1].axhline(2.0, color="grey", ls=":", lw=1.0,
                    label="a fixed 2 mm bound: refuses 5 real cuts")
    axes[1].legend(loc="upper right", fontsize=8)
    axes[2].set_yscale("symlog", linthresh=1.0)
    axes[2].set_xlabel("boundary index")
    axes[2].set_xticks(idx)

    handles = [
        plt.Rectangle((0, 0), 1, 1, color="#4c72b0"),
        plt.Rectangle((0, 0), 1, 1, color="#c44e52"),
    ]
    axes[2].legend(handles, ["hand-over (two sources)", "cut (one run)"],
                   loc="upper left", fontsize=8)
    fig.suptitle(
        "Day 12 Step 0 -- a cut and a hand-over are not the same boundary\n"
        "Step 10R's own cuts move the hip up to 6.6 mm; that is one rolling "
        "step, not a discontinuity.",
        fontsize=10,
    )
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)

    semantics = segment_semantics_rows()
    write_rows_csv(OUT / "day12_step0_segment_semantics.csv", semantics)
    nominal = [r["kind"] for r in semantics if r["is_nominal_locomotion"]]
    pinned = [r["kind"] for r in semantics if r["pins_theta_to_wheel_mode"]]
    print(f"segment kinds          {len(semantics)}")
    print(f"nominal locomotion     {nominal}")
    print(f"pins theta to 17 deg   {pinned}  "
          f"(theta = {np.rad2deg(WHEEL_MODE_THETA_RAD):.0f} deg)")

    chain = build_chain()
    rows = boundary_rows_2d(chain.segments, chain.tolerance)
    write_rows_csv(OUT / "day12_step0_boundary_evidence.csv", rows)
    plot_boundary_evidence(rows, OUT / "day12_step0_boundary_evidence.png")

    cuts = [r for r in rows if r["boundary_kind"] == "cut"]
    handovers = [r for r in rows if r["boundary_kind"] == "handover"]
    print(f"\nchain sources          {chain.sources}")
    print(f"boundaries             {len(rows)} "
          f"({len(handovers)} hand-over, {len(cuts)} cut)")
    print(f"max cut hip jump       {max(r['hip_jump_mm'] for r in cuts):.3f} mm")
    print(f"max cut joint jump     "
          f"{max(max(abs(r['theta_jump_deg']), abs(r['beta_jump_deg'])) for r in cuts):.2f} deg")
    print(f"max contact jump       {max(r['contact_jump_mm'] for r in rows):.1f} mm")
    print(f"all boundaries continuous  {all(r['continuous'] for r in rows)}")
    print(f"chain is_chained       {chain.is_chained}")
    print(f"chain is_complete      {chain.is_complete}")
    print(f"untimed segments       {len(chain.untimed_segments)} / "
          f"{len(chain.segments)}  (total_duration_s = {chain.total_duration_s})")
    print(f"\nwrote -> {OUT}")


if __name__ == "__main__":
    main()
