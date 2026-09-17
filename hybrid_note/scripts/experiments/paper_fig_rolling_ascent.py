#!/usr/bin/env python3
"""Paper figure: the rolling-ascent primitive, four stages in one row.

The Day 6--7 showcase renders all ten stages of the full right-up/left-down
traversal in a 5x2 grid.  That figure is a development view: at one column
width the panels are illegible, the axis ranges differ, and the panel titles
carry internal surface names.

The paper needs less.  Section III-C presents rolling ascent as a primitive in
its own right -- ground contact, front-face contact, corner pivot, top contact
-- so the figure shows those four stages only, in a single row, with a shared
window and tick labels at a size that survives printing.  The descent is a
separate primitive and is carried by the text.

Geometry is not recomputed: this calls the same traversal as the showcase and
draws the last accepted frame of each ascent phase.

Output: hybrid_note/paper/figures/rolling_ascent.pdf (vector, for LaTeX)
        hybrid_note/paper/figures/rolling_ascent.png (preview)
"""

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from hybrid_note.scripts.experiments.right_up_left_down_full_traversal_2d import (  # noqa: E402
    ObstacleSpec2D,
    PHASE_APPROACH,
    PHASE_FRONT_CONTACT,
    PHASE_ROLL_UP,
    PHASE_RIGHT_RIM_TOP,
    check_right_up_left_down_traversal,
)
from hybrid_note.scripts.experiments.single_leg_rolling_scene_2d import (  # noqa: E402
    plot_single_leg_rolling_scene_2d,
)

HERE = Path(__file__).resolve().parents[2]
OUT_DIR = HERE / "paper" / "figures"

# The four stages that make up the ascent, with the captions the paper uses.
ASCENT_STAGES = [
    (PHASE_APPROACH, "approach"),
    (PHASE_FRONT_CONTACT, "front-face contact"),
    (PHASE_ROLL_UP, "corner pivot"),
    (PHASE_RIGHT_RIM_TOP, "top contact"),
]

HALF_SPAN_M = 0.26


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--height-mm", type=float, default=100.0)
    ap.add_argument("--top-length-mm", type=float, default=350.0)
    ap.add_argument("--theta-climb-deg", type=float, default=60.0)
    ap.add_argument("--width-in", type=float, default=3.45,
                    help="figure width in inches (3.45 = IEEE single column)")
    ap.add_argument("--out", type=Path, default=OUT_DIR)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    obstacle = ObstacleSpec2D(
        height_m=args.height_mm * 1e-3,
        width_m=args.top_length_mm * 1e-3,
    )
    result = check_right_up_left_down_traversal(
        obstacle=obstacle,
        theta_climb=float(np.deg2rad(args.theta_climb_deg)),
    )

    frames = []
    for phase, caption in ASCENT_STAGES:
        accepted = [f for f in result.trajectory
                    if f.phase == phase and f.accepted]
        if not accepted:
            raise SystemExit(f"no accepted frame for stage {phase!r}")
        frames.append((caption, accepted[-1]))

    plt.rcParams.update({
        "font.size": 7, "axes.labelsize": 7, "axes.titlesize": 7,
        "xtick.labelsize": 6, "ytick.labelsize": 6,
        "axes.linewidth": 0.6,
        "xtick.major.width": 0.6, "ytick.major.width": 0.6,
    })

    n = len(frames)
    # Two rows of two at column width: four panels in one column strip cost a
    # full text-width float, which a six-page paper cannot spare.
    rows, cols = 2, 2
    fig, axes2 = plt.subplots(
        rows, cols,
        figsize=(args.width_in, args.width_in / cols * rows + 0.5),
        squeeze=False)
    flat = [a for row in axes2 for a in row]
    axes = [flat]
    for ax, (caption, frame) in zip(flat, frames):
        plot_single_leg_rolling_scene_2d(frame.scene, ax=ax)
        centre = np.asarray(frame.hip_position_world_xz_m, dtype=float)
        ax.set_xlim(centre[0] - HALF_SPAN_M, centre[0] + HALF_SPAN_M)
        ax.set_ylim(centre[1] - HALF_SPAN_M, centre[1] + HALF_SPAN_M)
        ax.set_aspect("equal", adjustable="box")
        ax.set_title(f"{caption}\n"
                     rf"$\theta$ = {np.rad2deg(frame.theta_rad):.1f}$^\circ$, "
                     rf"$\beta$ = {np.rad2deg(frame.beta_rad):.0f}$^\circ$",
                     fontsize=7)
        # The shared scene plotter annotates every terrain surface with its
        # internal id ("day6_7_obstacle_front"), which is a development aid and
        # has no place in the paper.  The geometry stays; only the text goes.
        for text in list(ax.texts):
            if text.get_text() not in ("H",):
                text.remove()
        # x label only on the bottom row, or the titles below collide with it
        if flat.index(ax) >= (rows - 1) * cols:
            ax.set_xlabel("world $x$ [m]")
        else:
            ax.set_xlabel("")
            ax.tick_params(labelbottom=False)
        if flat.index(ax) % cols == 0:
            ax.set_ylabel("world $z$ [m]")
        else:
            ax.set_ylabel("")
            ax.tick_params(labelleft=False)
        leg = ax.get_legend()
        if leg is not None:
            leg.remove()

    fig.tight_layout(pad=0.3, w_pad=0.4, h_pad=1.1)
    for ext in ("pdf", "png"):
        path = args.out / f"rolling_ascent.{ext}"
        fig.savefig(path, bbox_inches="tight", dpi=400)
        print(f"-> {path}")


if __name__ == "__main__":
    main()
