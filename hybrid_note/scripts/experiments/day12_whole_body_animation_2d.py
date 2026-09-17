"""Day 12 appendix: draw the assembled whole-body trajectory.

This module **shows** Step 8's result.  It computes no motion: every pose it
draws comes out of :class:`WholeBodyTrajectory2D`, and the one number it does
compute -- a body height to draw at -- is a *viewing* quantity that is never
written back into any trajectory.

Why a viewing height is needed at all
-------------------------------------

Step 5 refused to pick ``body_z``: the four legs' hard demands disagree by up
to 15.3 mm, so every sample of the frozen trajectory carries ``nan`` there
(Step 8 summary: 0 of 241 finite).  A drawing needs *some* height, and the
honest way to pick one is to state the rule and then show what it costs:

    the lowest body height at which **no stance foot is pushed into the
    ground** -- that is, the maximum of the hard demands at that instant.

Every other stance leg then ends up with its foot *above* the ground by
``chosen - its own demand``.  Those residuals are the Step 5 conflict made
visible, and they are drawn on the figure rather than smoothed away.  Picking
the *minimum* instead would bury the same conflict underground, which is worse:
it looks like contact.

The demands themselves are read with :func:`leg_demand_at` -- Step 5's own
function, on Step 8's own sample times -- rather than re-derived here.  Day 12
has twice paid for writing a second copy of a measuring function (Step 8's
notes), and a viewer is the last place that should own a definition.

``use_generator_frames`` and the frozen numbers
-----------------------------------------------

An animation assembled with the default ``use_generator_frames=False`` shows a
``RECOVERY_SWING`` with ``theta`` pinned at 60 deg for the whole swing -- the
segment's endpoints are both 60 deg and the fallback interpolates between them,
so the retract that the swing is *made of* never appears.  Step 9 reports the
same limitation as a peak-``theta``-rate of 0.  So the caller should assemble
with ``use_generator_frames=True`` for anything meant to be looked at, and say
so: those poses are the generator's own frames, while every frozen Day 12
metric was measured on the interpolation.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

from legwheel.visualization.plot_leg import PlotLeg

from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import SegmentKind
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (
    LegDemand2D,
    leg_demand_at,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (
    LEG_ORDER,
    LegId,
    leg_mounts_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (
    GAMMA_RAD,
    segment_at,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import LegMode
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
    TRANSITION_PHASE_OF_KIND,
    FourLegPlan2D,
)
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (
    WholeBodySample2D,
    WholeBodyTrajectory2D,
)

#: What the drawn height is, in one sentence that travels with the data.
VIEWING_BASIS: str = (
    "viewing-only body height: the lowest body_z at which no stance foot is "
    "pushed into the ground (the maximum of that instant's hard demands). "
    "Step 5 refused to choose a height at all -- this is a drawing choice, it "
    "resolves nothing, and it is never written into a trajectory."
)

#: Source labels for :attr:`ViewingHeight2D.source`.
FROM_HARD_DEMANDS: str = "max_hard_demand"
FROM_NOMINAL: str = "nominal_no_hard_demand"

#: A residual smaller than this is the contact tolerance, not a visible gap.
GAP_TOLERANCE_M: float = 1.0e-3

#: Nominal kinds get their Day 12 colours; every terrain-transition kind gets
#: the one the raised-surface line is drawn in, so a crossing is the same
#: colour wherever it appears on the figure.
CROSSING_COLOR = "#7a3f9d"
SEGMENT_COLORS: dict[str, str] = {
    SegmentKind.FOOT_RIM_ROLL.value: "#2a6f4e",
    SegmentKind.RECOVERY_SWING.value: "#b06000",
    **{kind.value: CROSSING_COLOR for kind in TRANSITION_PHASE_OF_KIND},
}
DEFAULT_SEGMENT_COLOR = "#4a5568"

SUPPORT_COLOR = "#1f2937"
SWING_COLOR = "#b06000"
GAP_COLOR = "#c5221f"
GROUND_COLOR = "#8a8577"


# --------------------------------------------------------------------------
# The viewing height
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class ViewingHeight2D:
    """A body height to draw one sample at, and what it costs each leg."""

    time_s: float
    body_z_m: float
    source: str
    demands: tuple[LegDemand2D, ...]
    #: Per stance leg, how far its foot ends up **above** the ground.  Never
    #: negative by construction: that is the whole point of taking the maximum.
    float_gap_m: dict[LegId, float]
    #: Chosen height minus the highest airborne floor.  Negative would mean the
    #: drawn body is below a clearance requirement; reported, not clipped.
    lower_bound_slack_m: float | None
    #: The disagreement this sample had to be resolved out of.
    spread_m: float

    @property
    def worst_gap_m(self) -> float:
        return max(self.float_gap_m.values(), default=0.0)

    @property
    def legs_off_the_ground(self) -> tuple[LegId, ...]:
        return tuple(leg for leg, gap in self.float_gap_m.items()
                     if gap > GAP_TOLERANCE_M)

    def as_dict(self) -> dict:
        row = {
            "time_s": self.time_s,
            "viewing_body_z_mm": self.body_z_m * 1e3,
            "source": self.source,
            "hard_demands": len([d for d in self.demands if d.is_hard]),
            "lower_bounds": len([d for d in self.demands if not d.is_hard]),
            "demand_spread_mm": self.spread_m * 1e3,
            "worst_float_gap_mm": self.worst_gap_m * 1e3,
            "legs_off_the_ground": ",".join(
                leg.value for leg in self.legs_off_the_ground),
            "lower_bound_slack_mm": (None if self.lower_bound_slack_m is None
                                     else self.lower_bound_slack_m * 1e3),
            "basis": VIEWING_BASIS,
        }
        for leg in LEG_ORDER:
            gap = self.float_gap_m.get(leg)
            row[f"{leg.value}_float_gap_mm"] = None if gap is None else gap * 1e3
        return row


def demands_at_2d(plan: FourLegPlan2D, time_s: float) -> tuple[LegDemand2D, ...]:
    """Every leg's body demand at ``time_s``, read with Step 5's own function.

    The segment lookup is Step 6's ``segment_at`` for the same reason Step 8
    uses it: the half-open ownership rule at a boundary is exactly what a
    second copy gets wrong.
    """

    out: list[LegDemand2D] = []
    for leg in LEG_ORDER:
        if leg not in plan.plans:
            continue
        scheduled = segment_at(plan.schedule.segments_of(leg), float(time_s))
        if scheduled is None:
            continue
        phased = plan.plans[leg].phased[scheduled.segment_index]
        demand = leg_demand_at(scheduled, phased.segment, float(time_s),
                               phase=phased.phase)
        if demand is not None:
            out.append(demand)
    return tuple(out)


def viewing_heights_2d(
    plan: FourLegPlan2D,
    whole: WholeBodyTrajectory2D,
    *,
    nominal_body_z_m: float,
) -> tuple[ViewingHeight2D, ...]:
    """A drawable height for every sample of ``whole``.  Nothing is mutated."""

    out: list[ViewingHeight2D] = []
    for sample in whole.samples:
        demands = demands_at_2d(plan, sample.time_s)
        hard = [d for d in demands if d.is_hard]
        floors = [d for d in demands if not d.is_hard]

        if hard:
            chosen = max(d.body_z_m for d in hard)
            source = FROM_HARD_DEMANDS
            spread = chosen - min(d.body_z_m for d in hard)
        else:
            # Nothing is standing on anything at this instant, so there is no
            # contact to respect and the nominal posture is as good as it gets.
            chosen = float(nominal_body_z_m)
            source = FROM_NOMINAL
            spread = 0.0

        out.append(ViewingHeight2D(
            time_s=sample.time_s,
            body_z_m=float(chosen),
            source=source,
            demands=demands,
            float_gap_m={d.leg: float(chosen - d.body_z_m) for d in hard},
            lower_bound_slack_m=(None if not floors else
                                 float(min(chosen - d.body_z_m for d in floors))),
            spread_m=float(spread),
        ))
    return tuple(out)


def viewing_height_rows(heights: Sequence[ViewingHeight2D]) -> list[dict]:
    """Serialize, one row per sample."""

    return [{"row_kind": "viewing_height", **h.as_dict()} for h in heights]


# --------------------------------------------------------------------------
# The surface each foot is planned to stand on
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class PlannedSurface2D:
    """What surface one leg's current segment is on, and what it aims at."""

    kind: SegmentKind
    #: The surface under the foot right now.  Only meaningful in stance; while
    #: airborne this is an interpolation between two surfaces and nothing is
    #: drawn against it.
    current_z_m: float
    start_z_m: float
    end_z_m: float

    @property
    def raised_z_m(self) -> float | None:
        """The raised end of the segment, if it has one.

        A crossing swing has exactly one: the ascent lands on it, the descent
        leaves it.  That is the surface worth drawing, because in this plan it
        is the only evidence on the figure that an obstacle was ever involved.
        """

        highest = max(self.start_z_m, self.end_z_m)
        return highest if abs(highest) > GAP_TOLERANCE_M else None

    @property
    def raised_is_the_target(self) -> bool:
        return self.end_z_m >= self.start_z_m


def planned_surfaces_2d(
    plan: FourLegPlan2D,
    whole: WholeBodyTrajectory2D,
) -> tuple[dict[LegId, PlannedSurface2D], ...]:
    """Per sample, the surface each leg's own segment refers to.

    On flat ground every value is the ground and the drawing is unchanged.  On
    a crossing this is what makes ``float_gap_m`` mean "above **its own**
    surface" instead of "above z = 0", and what puts the obstacle's top on the
    figure at all.
    """

    out: list[dict[LegId, PlannedSurface2D]] = []
    for sample in whole.samples:
        surfaces: dict[LegId, PlannedSurface2D] = {}
        for leg in LEG_ORDER:
            if leg not in plan.plans:
                continue
            scheduled = segment_at(plan.schedule.segments_of(leg),
                                   sample.time_s)
            if scheduled is None:
                continue
            segment = plan.plans[leg].phased[scheduled.segment_index].segment
            span = scheduled.end_s - scheduled.start_s
            fraction = 0.0 if span <= 0.0 else float(np.clip(
                (sample.time_s - scheduled.start_s) / span, 0.0, 1.0))
            low = float(segment.start_contact.point_world_xz_m[1])
            high = float(segment.end_contact.point_world_xz_m[1])
            surfaces[leg] = PlannedSurface2D(
                kind=segment.kind, current_z_m=low + fraction * (high - low),
                start_z_m=low, end_z_m=high)
        out.append(surfaces)
    return tuple(out)


# --------------------------------------------------------------------------
# Drawing one frame
# --------------------------------------------------------------------------


def _mounts() -> dict[LegId, np.ndarray]:
    return {m.leg: m.offset_body_xyz_m for m in leg_mounts_2d(GAMMA_RAD)}


def _side_legs(mounts: dict[LegId, np.ndarray]) -> tuple[tuple[LegId, ...], ...]:
    """``(left, right)``, front leg first, straight off the mounting sign."""

    left = sorted((l for l in LEG_ORDER if mounts[l][1] > 0.0),
                  key=lambda l: -mounts[l][0])
    right = sorted((l for l in LEG_ORDER if mounts[l][1] < 0.0),
                   key=lambda l: -mounts[l][0])
    return tuple(left), tuple(right)


def hip_world_xz_m(sample: WholeBodySample2D, height: ViewingHeight2D,
                   mount: np.ndarray) -> tuple[float, float]:
    """Where a leg plane's origin is when the body is drawn at ``height``.

    ``ABAD_AXIS_OFFSET`` is the leg plane's height **above** the body origin
    (Day 12 trap 17), so it adds here -- the same direction Step 5 subtracts it
    when it turns a hip requirement into a body requirement.
    """

    return (float(sample.body_position_world_m[0] + mount[0]),
            float(height.body_z_m + mount[2]))


def sagittal_limits_2d(
    whole: WholeBodyTrajectory2D,
    heights: Sequence[ViewingHeight2D],
    *,
    ground_z_m: float = 0.0,
    x_pad_m: float = 0.17,
    z_pad_m: float = 0.16,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Fixed limits for the whole run, so the body's advance is visible.

    Both pads clear the leg-wheel, not the hip: the rim reaches a radius
    *around* the leg-plane origin, so padding sized for the hip cuts the wheel
    in half -- on the side of the frame first, which is where it is hardest to
    notice.
    """

    mounts = _mounts()
    hips = np.array([hip_world_xz_m(s, h, mounts[leg])
                     for s, h in zip(whole.samples, heights)
                     for leg in LEG_ORDER])
    return ((float(hips[:, 0].min() - x_pad_m), float(hips[:, 0].max() + x_pad_m)),
            (float(ground_z_m - 0.06), float(hips[:, 1].max() + z_pad_m)))


def draw_sagittal_2d(
    ax,
    sample: WholeBodySample2D,
    height: ViewingHeight2D,
    legs: Sequence[LegId],
    plotters: Sequence[PlotLeg],
    *,
    mounts: dict[LegId, np.ndarray],
    x_limits: tuple[float, float],
    z_limits: tuple[float, float],
    ground_z_m: float = 0.0,
    label: str = "",
    show_ylabel: bool = True,
    surfaces: dict[LegId, PlannedSurface2D] | None = None,
) -> None:
    """One side of the robot, in the plane the whole pipeline is written in."""

    ax.clear()
    ax.fill_between(x_limits, z_limits[0], ground_z_m, color="#efece4", zorder=0)
    ax.axhline(ground_z_m, color=GROUND_COLOR, lw=1.2, zorder=1)
    # World-fixed marks: the body advances against them, so the drawing shows
    # travel and not just leg-waving.
    for x in np.arange(np.floor(x_limits[0] * 10) / 10, x_limits[1], 0.10):
        ax.plot([x, x], [ground_z_m - 0.022, ground_z_m], color=GROUND_COLOR,
                lw=0.8, zorder=1)

    hips = {leg: hip_world_xz_m(sample, height, mounts[leg]) for leg in legs}
    xs = [hips[leg][0] for leg in legs]
    zs = [hips[leg][1] for leg in legs]
    ax.plot(xs, zs, color=SUPPORT_COLOR, lw=4.0, solid_capstyle="round",
            zorder=4, alpha=0.85)

    for leg, plotter in zip(legs, plotters):
        state = sample.legs.get(leg)
        if state is None:
            continue
        hip = hips[leg]
        plotter.plot_leg(state.theta_rad, state.beta_rad, list(hip), ax)
        airborne = state.mode is LegMode.AIRBORNE
        ax.scatter([hip[0]], [hip[1]], marker="X", s=70, zorder=9,
                   color=SWING_COLOR if airborne else SUPPORT_COLOR,
                   edgecolor="white", linewidth=0.6)
        ax.annotate(f"{leg.value}{'  SWING' if airborne else ''}",
                    xy=hip, xytext=(-14, 12), textcoords="offset points",
                    fontsize=9, fontweight="bold",
                    color=SWING_COLOR if airborne else SUPPORT_COLOR)

        contact_x = float(state.contact_world_xy_m[0])
        planned = None if surfaces is None else surfaces.get(leg)

        # An airborne crossing swing has no foot on the ground to draw, but it
        # does have the surface it is aiming at (or has just left) -- and in
        # this plan that raised line is the only thing on the figure that an
        # obstacle was ever involved in.
        if airborne and planned is not None and planned.raised_z_m is not None:
            raised = planned.raised_z_m
            ax.plot([contact_x - 0.07, contact_x + 0.07], [raised] * 2,
                    color=CROSSING_COLOR, lw=1.8, ls="--", zorder=7)
            verb = "lands on" if planned.raised_is_the_target else "left"
            ax.annotate(f"{planned.kind.value} {verb} {raised * 1e3:+.0f} mm",
                        xy=(contact_x, raised), xytext=(-30, 8),
                        textcoords="offset points", fontsize=8.5,
                        color=CROSSING_COLOR)

        gap = height.float_gap_m.get(leg)
        if gap is None:
            continue
        # The gap is measured from the surface this leg's own segment stands
        # on, so it has to be drawn from there too -- against z = 0 a leg on an
        # obstacle would read as floating by the obstacle's height.
        surface_z = (ground_z_m if planned is None else planned.current_z_m)
        foot_z = surface_z + gap
        if abs(surface_z - ground_z_m) > GAP_TOLERANCE_M:
            ax.plot([contact_x - 0.06, contact_x + 0.06], [surface_z] * 2,
                    color=CROSSING_COLOR, lw=1.6, ls="--", zorder=7)
            ax.annotate(f"stands on {surface_z * 1e3:+.0f} mm",
                        xy=(contact_x, surface_z), xytext=(-8, -14),
                        textcoords="offset points", fontsize=8,
                        color=CROSSING_COLOR)
        visible = gap > GAP_TOLERANCE_M
        ax.scatter([contact_x], [foot_z], marker="v", s=46, zorder=9,
                   color=GAP_COLOR if visible else "#2a6f4e",
                   edgecolor="white", linewidth=0.5)
        if visible:
            ax.plot([contact_x, contact_x], [ground_z_m, foot_z],
                    color=GAP_COLOR, lw=2.0, zorder=8)
            # Above the marker, never below: the strip's side label lives in
            # the ground band and the two collide there.
            ax.annotate(f"+{gap * 1e3:.1f} mm", xy=(contact_x, foot_z),
                        xytext=(6, 6), textcoords="offset points",
                        fontsize=8.5, color=GAP_COLOR)

    ax.set_xlim(*x_limits)
    ax.set_ylim(*z_limits)
    ax.set_aspect("equal", adjustable="box")
    if show_ylabel:
        ax.set_ylabel("z  [m]", fontsize=8)
    ax.tick_params(labelsize=8)
    if label:
        ax.text(0.006, 0.045, label, transform=ax.transAxes, fontsize=9,
                color="#5a5648", family="monospace", zorder=12)


def draw_top_view_2d(
    ax,
    sample: WholeBodySample2D,
    height: ViewingHeight2D,
    *,
    half_width_m: float = 0.55,
) -> None:
    """Support polygon in the horizontal plane, with Step 6's own margin."""

    ax.clear()
    contacts = {leg: np.asarray(state.contact_world_xy_m, dtype=float)
                for leg, state in sample.legs.items()}
    support = [contacts[leg] for leg in sample.support_legs if leg in contacts]
    if len(support) >= 3:
        points = np.array(support)
        centre = points.mean(axis=0)
        order = np.argsort(np.arctan2(points[:, 1] - centre[1],
                                      points[:, 0] - centre[0]))
        ax.add_patch(Polygon(points[order], closed=True, facecolor="#dbeafe",
                             edgecolor="#1a4d8f", lw=1.4, zorder=2))

    for leg, point in contacts.items():
        airborne = sample.legs[leg].mode is LegMode.AIRBORNE
        ax.scatter([point[0]], [point[1]], s=58, zorder=4,
                   color=SWING_COLOR if airborne else SUPPORT_COLOR,
                   marker="o" if not airborne else "x")
        ax.annotate(leg.value, xy=point, xytext=(4, 5),
                    textcoords="offset points", fontsize=8,
                    color=SWING_COLOR if airborne else SUPPORT_COLOR)

    body_x = float(sample.body_position_world_m[0])
    ax.scatter([body_x], [0.0], marker="s", s=52, color="#c5221f", zorder=5)
    margin = sample.stability_margin_m
    shown = "n/a" if margin is None else f"{margin * 1e3:.2f} mm"
    ax.text(0.02, 0.90, f"top view   margin {shown}", transform=ax.transAxes,
            fontsize=9, color="#444", family="monospace", zorder=12)
    ax.set_xlim(body_x - half_width_m, body_x + half_width_m)
    ax.set_ylim(-0.40, 0.40)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x  [m]", fontsize=8)
    ax.set_ylabel("y  [m]", fontsize=8)
    ax.tick_params(labelsize=8)


def _mode_runs(whole: WholeBodyTrajectory2D, leg: LegId):
    """Maximal runs of one ``(segment_kind, mode)`` for one leg."""

    runs: list[tuple[float, float, str, bool]] = []
    for sample in whole.samples:
        state = sample.legs.get(leg)
        if state is None:
            continue
        key = (state.segment_kind.value, state.mode is LegMode.AIRBORNE)
        if runs and (runs[-1][2], runs[-1][3]) == key:
            runs[-1] = (runs[-1][0], sample.time_s, *key)
        else:
            runs.append((sample.time_s, sample.time_s, *key))
    return runs


def draw_timeline_2d(ax, whole: WholeBodyTrajectory2D, *,
                     cursor_time_s: float | None = None) -> None:
    """Who is on the ground when, plus the margin trace underneath."""

    ax.clear()
    for row, leg in enumerate(LEG_ORDER):
        for start, end, kind, airborne in _mode_runs(whole, leg):
            ax.barh(row, max(end - start, 1e-3), left=start, height=0.55,
                    color=SEGMENT_COLORS.get(kind, DEFAULT_SEGMENT_COLOR),
                    alpha=0.35 if airborne else 0.85,
                    hatch="///" if airborne else None,
                    edgecolor="white", linewidth=0.4, zorder=2)
    ax.set_yticks(range(len(LEG_ORDER)))
    ax.set_yticklabels([leg.value for leg in LEG_ORDER], fontsize=8)
    ax.invert_yaxis()
    ax.set_xlabel("t  [s]", fontsize=8)
    ax.tick_params(labelsize=8)

    times = np.array([s.time_s for s in whole.samples])
    margins = np.array([np.nan if s.stability_margin_m is None
                        else s.stability_margin_m * 1e3 for s in whole.samples])
    twin = getattr(ax, "_margin_twin", None)
    if twin is None:
        twin = ax.twinx()
        ax._margin_twin = twin
    twin.clear()
    # ``clear`` puts the twin's ticks back on the left, on top of the leg
    # labels; they have to be sent right again on every frame.
    twin.yaxis.tick_right()
    twin.yaxis.set_label_position("right")
    twin.plot(times, margins, color="#c5221f", lw=1.3, zorder=3)
    twin.axhline(10.0, color="#c5221f", ls="--", lw=0.9, alpha=0.7)
    twin.set_ylabel("margin [mm]", fontsize=8, color="#c5221f")
    twin.tick_params(axis="y", labelsize=8, labelcolor="#c5221f")
    finite = margins[np.isfinite(margins)]
    top = 12.0 if finite.size == 0 else max(12.0, float(finite.max()) * 1.2)
    twin.set_ylim(-1.0, top)

    if cursor_time_s is not None:
        ax.axvline(cursor_time_s, color="#111", lw=1.4, zorder=6)
    ax.set_xlim(float(times.min()), float(times.max()))
    ax.set_title("stance (solid) / airborne (hatched);  red = Step 6 margin, "
                 "dashed = 10 mm floor", fontsize=9, loc="left", color="#333")


# --------------------------------------------------------------------------
# Whole figures
# --------------------------------------------------------------------------


def _frame_title(sample: WholeBodySample2D, height: ViewingHeight2D) -> str:
    swing = "none" if sample.swing_leg is None else sample.swing_leg.value
    margin = sample.stability_margin_m
    margin_text = "n/a" if margin is None else f"{margin * 1e3:.2f} mm"
    return (f"t = {sample.time_s:5.3f} s    swing {swing:>4s}    "
            f"margin {margin_text:>9s}    "
            f"body x {sample.body_position_world_m[0] * 1e3:6.1f} mm    "
            f"drawn at body z {height.body_z_m * 1e3:6.2f} mm "
            f"(spread {height.spread_m * 1e3:.2f} mm, "
            f"worst foot gap {height.worst_gap_m * 1e3:.2f} mm)")


def plot_frame_strip_2d(
    whole: WholeBodyTrajectory2D,
    heights: Sequence[ViewingHeight2D],
    path: Path,
    *,
    indices: Sequence[int],
    ground_z_m: float = 0.0,
    width_in: float = 16.0,
    surfaces: Sequence[dict[LegId, PlannedSurface2D]] | None = None,
    subtitle: str | None = None,
) -> None:
    """A few frames as one static figure, for a notebook that must open fast."""

    mounts = _mounts()
    left, right = _side_legs(mounts)
    x_limits, z_limits = sagittal_limits_2d(whole, heights,
                                            ground_z_m=ground_z_m)

    # Equal-aspect axes take the height their data asks for, so the figure is
    # sized from the scene rather than the other way round -- otherwise every
    # row is mostly white space.
    ratios = (1.0, 1.0, 0.42)
    panel_in = width_in * ratios[0] / sum(ratios) * 0.94
    row_in = panel_in * (z_limits[1] - z_limits[0]) / (x_limits[1] - x_limits[0])
    rows = len(indices)

    figure, axes = plt.subplots(
        rows, 3, figsize=(width_in, rows * (row_in + 0.65) + 1.1),
        gridspec_kw={"width_ratios": list(ratios)}, constrained_layout=True,
    )
    axes = np.atleast_2d(axes)
    for row, index in enumerate(indices):
        sample, height = whole.samples[index], heights[index]
        for column, (side, name) in enumerate(((left, "left  (+y)"),
                                               (right, "right (-y)"))):
            draw_sagittal_2d(
                axes[row, column], sample, height, side,
                [PlotLeg() for _ in side], mounts=mounts, x_limits=x_limits,
                z_limits=z_limits, ground_z_m=ground_z_m,
                label=f"{name}   {', '.join(l.value for l in side)}",
                show_ylabel=column == 0,
                surfaces=None if surfaces is None else surfaces[index],
            )
        draw_top_view_2d(axes[row, 2], sample, height)
        axes[row, 0].set_title(_frame_title(sample, height), fontsize=9.5,
                               loc="left", family="monospace", color="#111")

    figure.suptitle(
        subtitle if subtitle is not None else
        "Day 12 -- the assembled whole-body trajectory, drawn.  Poses are "
        "Step 8's (generator frames); the height is a viewing choice, and a "
        "red bar is a foot that choice leaves off the ground.",
        fontsize=11.5,
    )
    figure.savefig(path, dpi=140)
    plt.close(figure)


def animate_whole_body_2d(
    whole: WholeBodyTrajectory2D,
    heights: Sequence[ViewingHeight2D],
    path: Path,
    *,
    stride: int = 3,
    fps: int = 12,
    ground_z_m: float = 0.0,
    width_in: float = 15.0,
    dpi: int = 82,
    surfaces: Sequence[dict[LegId, PlannedSurface2D]] | None = None,
    footnote: str | None = None,
) -> int:
    """Write the GIF.  Returns the number of frames drawn."""

    from matplotlib import animation

    mounts = _mounts()
    left, right = _side_legs(mounts)
    x_limits, z_limits = sagittal_limits_2d(whole, heights,
                                            ground_z_m=ground_z_m)

    indices = list(range(0, len(whole.samples), max(int(stride), 1)))
    if indices[-1] != len(whole.samples) - 1:
        indices.append(len(whole.samples) - 1)

    panel_in = width_in / 2.0 * 0.92
    sagittal_in = panel_in * (z_limits[1] - z_limits[0]) / (x_limits[1] - x_limits[0])
    figure = plt.figure(figsize=(width_in, sagittal_in + 3.9))
    grid = figure.add_gridspec(2, 2, height_ratios=[sagittal_in, 2.9],
                               hspace=0.30, wspace=0.13,
                               left=0.045, right=0.985, top=0.90, bottom=0.16)
    ax_left = figure.add_subplot(grid[0, 0])
    ax_right = figure.add_subplot(grid[0, 1])
    ax_top = figure.add_subplot(grid[1, 0])
    ax_time = figure.add_subplot(grid[1, 1])
    # One PlotLeg per leg: its patches are persistent objects, so two legs
    # sharing an instance would draw the same pose twice.
    plotters = {leg: PlotLeg() for leg in LEG_ORDER}

    assumptions = "\n".join(f"- {note}" for note in whole.assumptions)
    figure.text(0.045, 0.012, "this trajectory is assembled on top of:\n"
                + assumptions, fontsize=7.5, color="#7a3030", va="bottom")
    figure.text(0.985, 0.012,
                VIEWING_BASIS.replace(". ", ".\n") if footnote is None
                else footnote, fontsize=7.5, color="#555", va="bottom",
                ha="right")

    def draw(position: int):
        index = indices[position]
        sample, height = whole.samples[index], heights[index]
        for ax, side, name in ((ax_left, left, "left  (+y)"),
                               (ax_right, right, "right (-y)")):
            draw_sagittal_2d(ax, sample, height, side,
                             [plotters[leg] for leg in side], mounts=mounts,
                             x_limits=x_limits, z_limits=z_limits,
                             ground_z_m=ground_z_m,
                             label=f"{name}   {', '.join(l.value for l in side)}",
                             show_ylabel=ax is ax_left,
                             surfaces=None if surfaces is None
                             else surfaces[index])
        draw_top_view_2d(ax_top, sample, height)
        draw_timeline_2d(ax_time, whole, cursor_time_s=sample.time_s)
        figure.suptitle(_frame_title(sample, height), fontsize=10.5,
                        family="monospace")
        return figure.get_children()

    movie = animation.FuncAnimation(figure, draw, frames=len(indices),
                                    interval=1000 // max(fps, 1), blit=False,
                                    repeat=False)
    movie.save(path, writer=animation.PillowWriter(fps=fps), dpi=dpi)
    plt.close(figure)
    return len(indices)
