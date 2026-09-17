"""Day 12 Step 1 driver: generate the nominal cycles and their evidence.

Outputs (into ``hybrid_note/notes/day12/``):

``day12_step1_cycle_frames.csv``     every pose of two consecutive cycles
``day12_step1_cycle_summary.csv``    one row per cycle
``day12_step1_arc_start_evidence.csv``  the rolling/pivot boundary measurement
``day12_step1_phase_plot.png``       ROLL / RECOVERY / ROLL, plan §8.5(8)
``day12_step1_cycle_animation.gif``  the same two cycles, drawn

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step1_driver.py
    python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step1_driver.py --no-animation
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from legwheel.planners.hybrid import RimId  # noqa: E402
from hybrid_note.scripts.experiments.cartesian_swing_contract_2d import (  # noqa: E402
    rim_alpha_limits_rad,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (  # noqa: E402
    NominalPosture2D,
    cycle_frame_rows,
    cycle_segments_2d,
    cycle_summary_rows,
    run_nominal_cycles_2d,
)
from hybrid_note.scripts.experiments.day12_segment_contract_2d import (  # noqa: E402
    SegmentChain2D,
)
from hybrid_note.scripts.experiments.single_leg_rolling_scene_2d import (  # noqa: E402
    plot_single_leg_rolling_scene_2d,
)
from hybrid_note.scripts.experiments.trailing_edge_roll_down_2d import (  # noqa: E402
    _candidate_for_sample,
    _lowest_contact_sample,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day12"

PHASE_COLORS = {
    "FOOT_RIM_ROLL": "#2a6f4e",
    "RECOVERY_RETRACT": "#b06000",
    "RECOVERY_ROTATE": "#1a4d8f",
    "RECOVERY_EXTEND": "#8a5cb5",
    "RECOVERY_TOUCHDOWN": "#c5221f",
}


# --------------------------------------------------------------------------
# The rolling / pivot boundary, measured rather than asserted
# --------------------------------------------------------------------------


def arc_start_evidence_rows(posture: NominalPosture2D) -> list[dict]:
    """Where rolling stops and the seam-corner pivot begins.

    The stroke must start at the first beta that reaches the foot rim's alpha
    limit, not at the last beta still on the foot rim.  Between the two the
    support sample is pinned and the leg pivots about the foot/left seam
    corner: no rolling distance, and a large hip drop.  This table is the
    measurement that says so.
    """

    rows = []
    for beta_deg in np.arange(30.0, 64.0, 2.0):
        beta = float(np.deg2rad(beta_deg))
        hip_z = posture.hip_z_for_flat_stance(beta)
        scene = posture.scene(beta, 0.0, hip_z)
        sample = _lowest_contact_sample(scene.geometry)
        region = str(scene.geometry.contact_regions[sample])
        candidate = _candidate_for_sample(
            posture.query(scene), sample,
            surface_ids=(scene.terrain.ground_surface_id,),
        )
        rows.append({
            "beta_deg": float(beta_deg),
            "support_sample": int(sample),
            "contact_region": region,
            "alpha_deg": (
                None if candidate is None
                else float(np.rad2deg(candidate.alpha_rad))
            ),
            "hip_z_mm": hip_z * 1e3,
            "on_foot_rim": region == RimId.FOOT.value,
        })
    return rows


# --------------------------------------------------------------------------
# Plots
# --------------------------------------------------------------------------


def _phase_spans(frames):
    spans, start = [], 0
    for i in range(1, len(frames) + 1):
        if i == len(frames) or frames[i].phase != frames[start].phase:
            spans.append((start, i - 1, frames[start].phase))
            start = i
    return spans


def plot_phases(cycles, path: Path) -> None:
    frames = []
    for cycle in cycles:
        frames.extend(cycle.frames)
    n = np.arange(len(frames))
    theta = np.rad2deg([f.theta_rad for f in frames])
    beta = np.rad2deg([f.beta_rad for f in frames])
    hip_z = np.array([f.hip_xz_m[1] for f in frames]) * 1e3
    hip_x = np.array([f.hip_xz_m[0] for f in frames]) * 1e3
    clearance = np.array([f.clearance_m for f in frames]) * 1e3
    alpha = np.array(
        [np.nan if f.alpha_rad is None else np.rad2deg(f.alpha_rad) for f in frames]
    )

    fig, axes = plt.subplots(5, 1, figsize=(11.5, 11.0), sharex=True)
    spans = _phase_spans(frames)
    for ax in axes:
        for lo, hi, phase in spans:
            ax.axvspan(lo - 0.5, hi + 0.5, color=PHASE_COLORS[phase], alpha=0.10, lw=0)
        ax.grid(alpha=0.25)

    axes[0].plot(n, theta, color="#111", lw=1.6)
    axes[0].axhline(17.0, color="#b06000", ls="--", lw=1.0,
                    label="theta_compact = 17 deg (a recovery parameter)")
    axes[0].axhline(60.0, color="#2a6f4e", ls="--", lw=1.0,
                    label="nominal rolling posture = 60 deg")
    axes[0].set_ylabel("theta  [deg]")
    axes[0].legend(fontsize=8, loc="center right")

    axes[1].plot(n, beta, color="#111", lw=1.6)
    axes[1].set_ylabel("beta  [deg]")
    axes[1].annotate(
        "beta only ever decreases: one forward rotation sense,\n"
        "in contact and in the air alike",
        xy=(0.02, 0.12), xycoords="axes fraction", fontsize=8, color="#333",
    )

    axes[2].plot(n, alpha, color="#2a6f4e", lw=1.8)
    low, high = np.rad2deg(rim_alpha_limits_rad(RimId.FOOT))
    for value in (low, high):
        axes[2].axhline(value, color="#888", ls=":", lw=1.0)
    axes[2].set_ylabel("contact alpha  [deg]")
    axes[2].annotate("foot-rim limits; gaps are airborne", xy=(0.02, 0.08),
                     xycoords="axes fraction", fontsize=8, color="#333")

    axes[3].plot(n, hip_z, color="#111", lw=1.6, label="hip z")
    axes[3].set_ylabel("hip z  [mm]")
    twin = axes[3].twinx()
    twin.plot(n, hip_x, color="#1a4d8f", lw=1.2, ls="--")
    twin.set_ylabel("hip x  [mm]", color="#1a4d8f")
    twin.tick_params(axis="y", labelcolor="#1a4d8f")

    axes[4].plot(n, clearance, color="#111", lw=1.6)
    axes[4].axhline(10.0, color="#c5221f", ls="--", lw=1.0,
                    label="rotation clearance floor (10 mm)")
    axes[4].set_ylabel("terrain clearance  [mm]")
    axes[4].set_xlabel("frame")
    axes[4].legend(fontsize=8, loc="upper right")

    handles = [Patch(color=c, alpha=0.35, label=p) for p, c in PHASE_COLORS.items()]
    axes[0].legend(
        handles=handles
        + [
            plt.Line2D([], [], color="#b06000", ls="--",
                       label="theta_compact = 17 deg"),
            plt.Line2D([], [], color="#2a6f4e", ls="--",
                       label="nominal posture = 60 deg"),
        ],
        fontsize=7.5, loc="center right", ncol=2,
    )
    fig.suptitle(
        "Day 12 Step 1 -- two nominal flat-ground Hybrid cycles\n"
        "FOOT_RIM_ROLL (rolling stance) + RECOVERY_SWING "
        "(retract / rotate / extend / land)",
        fontsize=11,
    )
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def animate_cycles(cycles, posture: NominalPosture2D, path: Path,
                   *, stride: int = 3) -> None:
    """Draw the cycles by rebuilding each pose's scene.

    The frames carry joint values rather than scenes, so the drawing is
    reconstructed here.  That keeps the trajectory dataclasses light and, more
    usefully, means the animation is a *re-derivation* of the poses rather than
    a replay of whatever objects happened to be in memory.
    """

    from matplotlib import animation

    frames = []
    for i, cycle in enumerate(cycles):
        picked = list(cycle.frames[::stride])
        if picked[-1] is not cycle.frames[-1]:
            picked.append(cycle.frames[-1])
        frames.extend((i, f) for f in picked)

    scenes = [
        posture.scene(f.beta_rad, f.hip_xz_m[0], f.hip_xz_m[1],
                      theta_rad=f.theta_rad)
        for _, f in frames
    ]
    points = np.vstack([
        np.vstack((s.geometry.points_world_xz_m,
                   s.hip_pose.position_world_xz_m[None, :]))
        for s in scenes
    ])
    x_pad, z_pad = 0.06, 0.05
    x_limits = (float(points[:, 0].min() - x_pad), float(points[:, 0].max() + x_pad))
    z_limits = (posture.ground_height_m - 0.05,
                float(points[:, 1].max() + z_pad))

    figure, ax = plt.subplots(figsize=(12, 5.0))

    def draw(index: int):
        ax.clear()
        cycle_index, frame = frames[index]
        plot_single_leg_rolling_scene_2d(scenes[index], ax=ax)
        ax.set_xlim(*x_limits)
        ax.set_ylim(*z_limits)
        ax.set_aspect("equal", adjustable="box")
        ax.set_title(
            f"cycle {cycle_index}   {frame.phase}   "
            f"theta {np.rad2deg(frame.theta_rad):5.1f} deg   "
            f"beta {np.rad2deg(frame.beta_rad):7.1f} deg   "
            f"clearance {frame.clearance_m * 1e3:5.1f} mm",
            fontsize=10, color=PHASE_COLORS[frame.phase],
        )
        legend = ax.get_legend()
        if legend is not None:
            legend.remove()
        return ax.get_children()

    movie = animation.FuncAnimation(
        figure, draw, frames=len(frames), interval=90, blit=False, repeat=False
    )
    movie.save(path, writer=animation.PillowWriter(fps=11))
    plt.close(figure)


# --------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--no-animation", action="store_true")
    parser.add_argument("--stride", type=int, default=3)
    args = parser.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)
    posture = NominalPosture2D()

    started = time.time()
    cycles = run_nominal_cycles_2d(args.cycles)
    print(f"generated {len(cycles)} cycles in {time.time() - started:.1f} s")

    summary = cycle_summary_rows(cycles)
    write_rows_csv(OUT / "day12_step1_cycle_summary.csv", summary)
    write_rows_csv(OUT / "day12_step1_cycle_frames.csv", cycle_frame_rows(cycles))
    write_rows_csv(
        OUT / "day12_step1_arc_start_evidence.csv", arc_start_evidence_rows(posture)
    )

    for row in summary:
        print(
            f"\ncycle {row['cycle']}  success={row['success']}  "
            f"stop={row['stroke_stop_reason']}"
        )
        print(f"  alpha            {row['alpha_from_deg']:+.1f} -> "
              f"{row['alpha_to_deg']:+.1f} deg   (the whole usable foot arc)")
        print(f"  contact advance  {row['stroke_contact_advance_mm']:.1f} mm")
        print(f"  hip advance      {row['cycle_hip_advance_mm']:.1f} mm")
        print(f"  hip z travel     {row['stroke_hip_z_travel_mm']:.1f} mm "
              f"(the stance body requirement)")
        print(f"  rotation         {row['stroke_rotation_deg']:.2f} deg rolling + "
              f"{row['recovery_rotation_deg']:.2f} deg airborne = "
              f"{row['cycle_rotation_deg']:.2f} deg")
        print(f"  recovery theta   down to {row['recovery_theta_min_deg']:.0f} deg")
        print(f"  clearance        {row['recovery_rotation_min_clearance_mm']:.1f} mm "
              f"while rotating, {row['recovery_ramp_min_clearance_mm'] * 1e3:.3f} um "
              f"at the liftoff/touchdown ramps")

    segments, offset = [], 0
    for cycle in cycles:
        pair = cycle_segments_2d(cycle, source_id="day12_step1", frame_offset=offset)
        segments.extend(pair)
        offset += sum(s.frames.frame_count for s in pair)
    chain = SegmentChain2D(leg_id="single_leg_2d", segments=tuple(segments))
    print(f"\nchain of {len(chain.segments)} segments: "
          f"is_chained={chain.is_chained}  is_complete={chain.is_complete}")
    print(f"  kinds  {[s.kind.value for s in chain.segments]}")
    print(f"  untimed segments {len(chain.untimed_segments)} / "
          f"{len(chain.segments)}  (Step 3 supplies the timeline)")

    plot_phases(cycles, OUT / "day12_step1_phase_plot.png")
    print(f"\nwrote plots and tables -> {OUT}")
    if not args.no_animation:
        started = time.time()
        animate_cycles(cycles, posture, OUT / "day12_step1_cycle_animation.gif",
                       stride=args.stride)
        print(f"animation written in {time.time() - started:.1f} s")


if __name__ == "__main__":
    main()
