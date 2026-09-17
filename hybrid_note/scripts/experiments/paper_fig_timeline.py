#!/usr/bin/env python3
"""Four-leg liftoff timeline for the 40 mm and 100 mm crossings.

Reads the Day 14 event CSVs and draws, for each crossing, one horizontal band
per leg carrying its airborne intervals against body travel.  The point of the
figure is the contrast between the two panels: at 40 mm every leg ascends by
stepping, while in the final 100 mm hardware plan only RF steps and LF, RH,
and LH roll up, so those three bands carry no ascent event.  No interval
overlaps another in the same panel, which is the single-airborne-leg condition.

Usage:  python3 paper_fig_timeline.py [output.pdf]
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams["ps.fonttype"] = 42
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

DAY14 = Path(__file__).resolve().parents[2] / "notes" / "day14"
PANELS = [
    ("day14_step3_40mm_v3-2_events.csv", 40, 400.0),
    ("versions/v5/day14_step3_100mm_roll_v5lf875/"
     "day14_step3_100mm_roll_v5lf875_events.csv", 100, 875.0),
]
LEGS = ["LF", "RF", "LH", "RH"]

# Colour-blind-safe (Okabe--Ito).  Recovery is deliberately the quietest.
COLOR = {
    "RECOVERY_SWING": "#BBBBBB",
    "SWING_UP": "#0072B2",
    "SWING_DOWN": "#D55E00",
    "TOP_REPOSITION_SWING": "#009E73",
}
LABEL = {
    "RECOVERY_SWING": "nominal recovery",
    "SWING_UP": "stepping ascent",
    "SWING_DOWN": "stepping descent",
    "TOP_REPOSITION_SWING": "top reposition",
}


def read_events(path):
    rows = list(csv.DictReader(open(path)))
    plan = next(r for r in rows if r["row_kind"] == "plan")
    swings = [r for r in rows if r["row_kind"] == "swing"]
    return plan, swings


def rolls_up(swings, leg):
    """A leg that never steps up on an obstacle ascended by rolling."""
    return not any(s["leg"] == leg and s["kind"] == "SWING_UP" for s in swings)


def main(out):
    fig, axes = plt.subplots(2, 1, figsize=(3.4, 3.0), sharex=False)
    for ax, (fname, height, top_len) in zip(axes, PANELS):
        plan, swings = read_events(DAY14 / fname)
        x0 = min(float(s["body_x_start_mm"]) for s in swings)
        x1 = max(float(s["body_x_end_mm"]) for s in swings)

        # Obstacle span in body-x.  The body reaches the leading edge when its
        # own x does; the events CSV is already in that frame.
        obs_x = float(next(r for r in swings if r["kind"] != "RECOVERY_SWING")
                      ["body_x_start_mm"])
        ax.axvspan(1000.0, 1000.0 + top_len, color="#F0F0F0", zorder=0)

        for row, leg in enumerate(LEGS):
            y = len(LEGS) - 1 - row
            ax.axhline(y, color="#DDDDDD", lw=0.5, zorder=1)
            for s in swings:
                if s["leg"] != leg:
                    continue
                a = float(s["body_x_start_mm"])
                b = float(s["body_x_end_mm"])
                if b - a < 1.0:          # zero-length events still read as ticks
                    a, b = a - 4.0, b + 4.0
                ax.barh(y, b - a, left=a, height=0.55, zorder=3,
                        color=COLOR.get(s["kind"], "#999999"),
                        edgecolor="none")
            if rolls_up(swings, leg):
                # Mark the ascent that leaves no airborne event on this band.
                ax.annotate("rolls up", xy=(1000.0, y), xytext=(-6, 0),
                            textcoords="offset points", ha="right",
                            va="center", fontsize=6, color="#333333",
                            zorder=4,
                            bbox=dict(boxstyle="square,pad=0.15", fc="white",
                                      ec="none"))

        ax.set_yticks(range(len(LEGS)))
        ax.set_yticklabels(LEGS[::-1], fontsize=7)
        ax.set_ylim(-0.6, len(LEGS) - 0.4)
        ax.set_xlim(x0 - 40, x1 + 40)
        ax.tick_params(axis="x", labelsize=7)
        ax.set_title(f"{height} mm obstacle", fontsize=7.5, pad=3)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)

    axes[-1].set_xlabel("body travel (mm)", fontsize=7)
    handles = [Patch(facecolor=COLOR[k], label=LABEL[k])
               for k in ["RECOVERY_SWING", "SWING_UP", "TOP_REPOSITION_SWING",
                         "SWING_DOWN"]]
    axes[0].legend(handles=handles, fontsize=6, ncol=2, frameon=False,
                   loc="lower left", bbox_to_anchor=(0.0, 1.18))
    fig.tight_layout(pad=0.3)
    fig.subplots_adjust(hspace=0.55, top=0.80)
    fig.savefig(out, bbox_inches="tight")
    print("wrote", out)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "timeline.pdf")
