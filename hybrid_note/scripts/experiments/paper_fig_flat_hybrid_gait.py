#!/usr/bin/env python3
"""Paper schematic: one flat-ground Hybrid gait period.

The upper row makes the support semantics explicit: exactly one highlighted
leg performs an airborne recovery while the other three maintain rolling
stance contacts.  The lower timeline shows the same LF--RH--RF--LH sequence
and its non-overlapping recovery windows.  This is intentionally a schematic,
not a kinematic reconstruction.
"""

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle


HERE = Path(__file__).resolve().parents[2]
OUT_DIR = HERE / "paper" / "figures"
ORDER = ("LF", "RH", "RF", "LH")
COLOR_STANCE = "#4C78A8"
COLOR_SWING = "#E67E22"
COLOR_CONTACT = "#159447"
COLOR_BODY = "#555555"


def wheel(ax, x, y, color, *, airborne=False, zorder=4):
    ax.add_patch(Circle((x, y), 0.050, fc="white", ec=color, lw=1.5,
                        zorder=zorder))
    for dx, dy in ((0.036, 0.036), (-0.036, 0.036),
                   (0.036, -0.036), (-0.036, -0.036)):
        ax.plot([x, x + dx], [y, y + dy], color=color, lw=0.65,
                zorder=zorder)
    if not airborne:
        ax.plot(x, 0.0, "o", ms=3.4, color=COLOR_CONTACT, zorder=6)


def robot_panel(ax, active):
    ax.set_xlim(0.05, 0.95)
    ax.set_ylim(-0.04, 0.90)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.plot([0.04, 0.96], [0, 0], color="#222222", lw=1.1, zorder=1)
    ax.add_patch(Rectangle((0.20, 0.58), 0.60, 0.14,
                           fc="#D9D9D9", ec=COLOR_BODY, lw=1.0, zorder=3))
    ax.annotate("travel", xy=(0.82, 0.81), xytext=(0.47, 0.81),
                arrowprops=dict(arrowstyle="->", lw=0.9, color="#555555"),
                ha="center", va="center", fontsize=6, color="#555555")

    hip = {"LF": (0.29, 0.58), "LH": (0.35, 0.58),
           "RF": (0.71, 0.58), "RH": (0.65, 0.58)}
    foot = {"LF": 0.24, "LH": 0.38, "RF": 0.76, "RH": 0.62}
    for leg in ("LH", "RH", "LF", "RF"):
        hx, hy = hip[leg]
        if leg == active:
            direction = 1.0 if leg in ("LF", "RF") else -1.0
            fx, fy = foot[leg] + 0.09 * direction, 0.29
            ax.plot([hx, fx], [hy, fy], color=COLOR_SWING, lw=1.6, zorder=5)
            wheel(ax, fx, fy, COLOR_SWING, airborne=True, zorder=6)
            start = (foot[leg] - 0.10 * direction, 0.05)
            end = (foot[leg] + 0.14 * direction, 0.05)
            ax.add_patch(FancyArrowPatch(start, end,
                         connectionstyle="arc3,rad=-0.55" if direction > 0
                         else "arc3,rad=0.55",
                         arrowstyle="-|>", mutation_scale=7,
                         ls=(0, (2, 2)), lw=1.0, color=COLOR_SWING,
                         zorder=2))
            ax.text(fx, fy + 0.085, "airborne", color=COLOR_SWING,
                    fontsize=6, ha="center", weight="bold")
        else:
            fx, fy = foot[leg], 0.05
            style = "--" if leg in ("LH", "RH") else "-"
            ax.plot([hx, fx], [hy, fy], style, color=COLOR_STANCE,
                    lw=1.15, zorder=2)
            wheel(ax, fx, fy, COLOR_STANCE, zorder=3)
    ax.text(0.50, -0.025, "3 stance contacts", color=COLOR_CONTACT,
            fontsize=6, ha="center", va="top")


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 7, "font.family": "DejaVu Sans"})
    fig = plt.figure(figsize=(7.16, 3.05))
    grid = fig.add_gridspec(2, 4, height_ratios=(2.1, 1.0),
                            hspace=0.20, wspace=0.08)
    for i, leg in enumerate(ORDER):
        ax = fig.add_subplot(grid[0, i])
        robot_panel(ax, leg)
        ax.set_title(f"({chr(97+i)}) {leg} recovery", fontsize=7.2, pad=1.5)
        if i < 3:
            fig.add_artist(FancyArrowPatch(
                (0.245 + i * 0.244, 0.665), (0.270 + i * 0.244, 0.665),
                transform=fig.transFigure, arrowstyle="->", mutation_scale=8,
                lw=0.8, color="#666666"))

    ax = fig.add_subplot(grid[1, :])
    ax.set_xlim(0, 1)
    ax.set_ylim(-0.55, 3.65)
    recovery = 0.15
    for row, (leg, start) in enumerate(zip(ORDER[::-1], (0.75, 0.50, 0.25, 0.0))):
        ax.barh(row, 1.0, left=0, height=0.48, color=COLOR_STANCE,
                edgecolor="none")
        ax.barh(row, recovery, left=start, height=0.48, color=COLOR_SWING,
                edgecolor="none")
    ax.set_yticks(range(4), ORDER[::-1])
    ax.set_xlabel("normalized gait period  $t/T$")
    ax.set_xticks([0, 0.25, 0.50, 0.75, 1.0])
    ax.grid(axis="x", color="#DDDDDD", lw=0.55)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.tick_params(axis="y", length=0)
    ax.text(1.0, 3.45, "blue: rolling stance    orange: airborne recovery",
            ha="right", va="bottom", fontsize=6.3)

    fig.text(0.5, 0.985,
             "Flat-ground Hybrid gait: one leg recovers while three legs remain in stance",
             ha="center", va="top", fontsize=8, weight="bold")
    fig.subplots_adjust(left=0.055, right=0.995, bottom=0.16, top=0.91)
    for ext in ("pdf", "png"):
        out = OUT_DIR / f"flat_hybrid_gait.{ext}"
        fig.savefig(out, bbox_inches="tight")
        print("wrote", out)


if __name__ == "__main__":
    main()
