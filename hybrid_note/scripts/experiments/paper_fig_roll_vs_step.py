#!/usr/bin/env python3
"""Paper figure: the same edge climbed two ways, rolling above, stepping below.

Section III presents rolling and stepping as the two phases one gait alternates
between, and the whole selector rests on the reader seeing how differently they
get a leg onto a top.  Prose does not carry that: "the contact point evolves
along the rim" and "the leg lifts off and repositions" are both abstract until
the two are drawn over the same obstacle at the same scale.

So this draws one compact, single-column panel for each alternative.  Four
keyframes are overlaid on one obstacle in the top panel for rolling and one
obstacle in the bottom panel for stepping.  The rolling ascent preserves
ground/front/corner/top contact; the stepping ascent passes through stance,
liftoff, mid-flight, and touchdown.  The shared scale and fading make motion
visible without spending eight separate axes on repeated terrain.

Geometry is not recomputed for the figure.  The rolling row replays the Day 6-7
traversal and the stepping row replays a Day 14 nominal transition, each drawn
at its own accepted frames.

Output: hybrid_note/paper/figures/roll_vs_step.pdf (vector, for LaTeX)
        hybrid_note/paper/figures/roll_vs_step.png (preview)
"""

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams["ps.fonttype"] = 42

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import ConnectionPatch  # noqa: E402

from hybrid_note.scripts.experiments.right_up_left_down_full_traversal_2d import (  # noqa: E402
    ObstacleSpec2D,
    PHASE_APPROACH,
    PHASE_FRONT_CONTACT,
    PHASE_ROLL_UP,
    PHASE_RIGHT_RIM_TOP,
    check_right_up_left_down_traversal,
)
from hybrid_note.scripts.experiments.single_leg_rolling_scene_2d import (  # noqa: E402
    build_single_leg_rolling_scene_2d,
    plot_single_leg_rolling_scene_2d,
)
from legwheel.planners.hybrid.terrain_2d import plot_terrain_profile_2d  # noqa: E402
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    SharedTerrainSpec2D,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (  # noqa: E402
    NominalPosture2D,
    RecoveryConfig2D,
)
from hybrid_note.scripts.experiments.day14_nominal_transitions_2d import (  # noqa: E402
    TransitionKind2D,
    run_nominal_transition_2d,
)

HERE = Path(__file__).resolve().parents[2]
OUT_DIR = HERE / "paper" / "figures"

ROLL_STAGES = [
    (PHASE_APPROACH, "approach"),
    (PHASE_FRONT_CONTACT, "front-face contact"),
    (PHASE_ROLL_UP, "corner pivot"),
    (PHASE_RIGHT_RIM_TOP, "top contact"),
]
STEP_CAPTIONS = ["stance", "liftoff", "mid-flight", "touchdown"]


def roll_frames(height_m, top_len_m, theta_climb_rad):
    """Last accepted frame of each ascent stage of the Day 6-7 traversal."""
    result = check_right_up_left_down_traversal(
        obstacle=ObstacleSpec2D(height_m=height_m, width_m=top_len_m),
        theta_climb=theta_climb_rad)
    out = []
    for phase, caption in ROLL_STAGES:
        accepted = [f for f in result.trajectory if f.phase == phase and f.accepted]
        if not accepted:
            raise SystemExit(f"no accepted frame for rolling stage {phase!r}")
        out.append((caption, accepted[-1].scene,
                    accepted[-1].theta_rad, accepted[-1].beta_rad,
                    np.asarray(accepted[-1].hip_position_world_xz_m, float)))
    return out


def step_frames(spec, posture, config, theta_climb_rad):
    """Four poses spanning one stepping ascent onto the same top."""
    # Take off from a stance short of the face and land on the top, which is
    # what the per-leg rule does when it assigns a stepping ascent.
    takeoff_beta = float(np.deg2rad(-20.0))
    stand_z = float(posture.hip_z_for_flat_stance(takeoff_beta))
    hip_x0 = float(spec.x_start_m) - 0.16
    out = run_nominal_transition_2d(
        spec, posture, config, kind=TransitionKind2D.UP,
        takeoff_theta_rad=theta_climb_rad,
        takeoff_beta_rad=takeoff_beta,
        takeoff_hip_xz_m=(hip_x0, stand_z),
        landing_hip_x_m=float(spec.x_start_m) + 0.06,
        landing_hip_z_above_surface_m=stand_z)
    if not out.success:
        raise SystemExit(f"stepping ascent refused: {out.refusal}")
    frames = [f for f in out.swing.frames]
    picks = [0, len(frames) // 3, 2 * len(frames) // 3, len(frames) - 1]
    res = []
    for caption, i in zip(STEP_CAPTIONS, picks):
        f = frames[i]
        hip = np.asarray(f.hip_xz_m, dtype=float)
        scene = build_single_leg_rolling_scene_2d(
            float(f.theta_rad), float(f.beta_rad), float(hip[0]), float(hip[1]),
            ground_height_m=float(spec.ground_height_m),
            obstacle_x_start_m=float(spec.x_start_m),
            obstacle_width_m=float(spec.top_length_m),
            obstacle_height_m=float(spec.height_m))
        res.append((caption, scene, float(f.theta_rad), float(f.beta_rad), hip))
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--height-mm", type=float, default=100.0)
    ap.add_argument("--top-length-mm", type=float, default=350.0)
    ap.add_argument("--theta-climb-deg", type=float, default=60.0)
    ap.add_argument("--width-in", type=float, default=3.45,
                    help="IEEE single-column width")
    ap.add_argument("--out", type=Path, default=OUT_DIR)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    h = args.height_mm * 1e-3
    L = args.top_length_mm * 1e-3
    theta_climb = float(np.deg2rad(args.theta_climb_deg))

    spec = SharedTerrainSpec2D(height_m=h, top_length_m=L)
    posture = NominalPosture2D()
    config = RecoveryConfig2D()

    rows = [("rolling ascent", roll_frames(h, L, theta_climb)),
            ("stepping ascent", step_frames(spec, posture, config, theta_climb))]

    plt.rcParams.update({
        "font.size": 7, "axes.labelsize": 7, "axes.titlesize": 7,
        "xtick.labelsize": 6, "ytick.labelsize": 6,
        "axes.linewidth": 0.6,
        "xtick.major.width": 0.6, "ytick.major.width": 0.6,
    })

    fig, axes = plt.subplots(2, 4, figsize=(args.width_in, 2.42), squeeze=False)
    z_lo, z_hi = -0.025, 0.365

    for r, (row_label, frames) in enumerate(rows):
        hip_points = []
        for c, (caption, scene, theta, beta, hip) in enumerate(frames):
            ax = axes[r][c]
            plot_single_leg_rolling_scene_2d(scene, ax=ax)
            for item in list(ax.texts):
                item.remove()
            # Use the same metric scale but follow the hip with a tight local
            # window.  This keeps all four poses legible at single-column size.
            x_lo, x_hi = float(hip[0]) - 0.19, float(hip[0]) + 0.19
            plot_terrain_profile_2d(scene.terrain, ax=ax,
                                    x_limits_m=(x_lo, x_hi))
            for item in list(ax.texts):
                item.remove()
            ax.set_xlim(x_lo, x_hi)
            ax.set_ylim(z_lo, z_hi)
            ax.set_aspect("equal", adjustable="box")
            ax.set_xticks([])
            ax.set_xlabel("")
            ax.grid(False)
            ax.set_title("")
            for spine in ax.spines.values():
                spine.set_visible(False)
            if c == 0:
                ax.spines["left"].set_visible(True)
                ax.spines["left"].set_linewidth(0.65)
                ax.set_yticks((0.0, 0.1, 0.2, 0.3))
                ax.tick_params(axis="y", labelsize=4.7, length=2.0,
                               width=0.55, pad=1.0)
                ax.set_ylabel(r"$z$ [m]", fontsize=5.2, labelpad=1.2)
            else:
                ax.set_yticks([])
                ax.set_ylabel("")
            leg = ax.get_legend()
            if leg is not None:
                leg.remove()
            hip_points.append((ax, hip))

        # Connect the hip origins across the four snapshots.  ConnectionPatch
        # spans the subplot gaps, so the dashed line reads as one body path.
        for (ax0, hip0), (ax1, hip1) in zip(hip_points[:-1], hip_points[1:]):
            fig.add_artist(ConnectionPatch(
                xyA=hip0, coordsA=ax0.transData,
                xyB=hip1, coordsB=ax1.transData,
                color="#5F6368", linewidth=1.15, linestyle=(0, (2.4, 2.0)),
                zorder=100, clip_on=False))

    fig.subplots_adjust(left=0.075, right=0.995, bottom=0.015, top=0.84,
                        wspace=0.015, hspace=0.58)

    # Row semantics and phase labels.  Centers are taken from the actual axes,
    # so each phrase remains aligned with its snapshot after layout changes.
    fig.text(0.075, 0.975, "(a) Continuous right-rim contact",
             ha="left", va="top", fontsize=5.5, weight="bold")
    fig.text(0.075, 0.49, "(b) Airborne recovery",
             ha="left", va="top", fontsize=5.5, weight="bold")

    centers = [0.5 * (ax.get_position().x0 + ax.get_position().x1)
               for ax in axes[0]]
    boundaries = [0.5 * (axes[0][i].get_position().x1
                         + axes[0][i + 1].get_position().x0)
                  for i in range(3)]
    for x, label in zip(centers,
                        ("ground contact", "front-face contact",
                         "corner pivot", "top contact")):
        fig.text(x, 0.885, label, ha="center", va="center",
                 fontsize=4.85, weight="bold")
    for x in boundaries:
        fig.text(x, 0.885, r"$\rightarrow$", ha="center", va="center",
                 fontsize=5.2)

    fig.text(centers[0], 0.40, "stance", ha="center", va="center",
             fontsize=4.85, weight="bold")
    fig.text(0.5 * (centers[1] + centers[2]), 0.40, "airborne recovery",
             ha="center", va="center", fontsize=4.85, weight="bold")
    fig.text(centers[3], 0.40, "touchdown", ha="center", va="center",
             fontsize=4.85, weight="bold")
    for x in (boundaries[0], boundaries[2]):
        fig.text(x, 0.40, r"$\rightarrow$", ha="center", va="center",
                 fontsize=5.2)
    for ext in ("pdf", "png"):
        path = args.out / f"roll_vs_step.{ext}"
        fig.savefig(path, bbox_inches="tight")
        print("wrote", path)


if __name__ == "__main__":
    main()
