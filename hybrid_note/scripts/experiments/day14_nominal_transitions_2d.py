"""Day 14 Step 2: every terrain-transition swing, flown the nominal way.

Day 13 B5 established one case -- climb *onto* an obstacle with the flat-ground
recovery swing (retract to the compact posture, carry the forward rotation on,
extend into the landing) and keep Day 8--9 / Day 10--11 only for *where* the
swing starts and lands.  The project owner wants every airborne move made that
way, and there are four of them:

===============  ==========================  =========================
kind             from                        to
===============  ==========================  =========================
``UP``           the ground before the face  the obstacle top
``DOWN``         the obstacle top            the ground past the back
``TOP``          the obstacle top            the obstacle top, further on
``OVER``         the ground before the face  the ground past the back
===============  ==========================  =========================

One function, :func:`run_nominal_transition_2d`, flies all four.  What differs
is only the endpoints, which is the point: the generator is
``run_recovery_swing_2d`` in every case, so the clearance rules, the ramp
checks and the touchdown check are the ones the flat gait already passes.

Two things Day 14 had to add to the generator for this to be honest:

* airborne frames now measure clearance against the **terrain solid** (ground
  and rectangle) rather than the ground plane, and carry a ``collision`` flag
  -- before this a swing that clipped the front face reported a clean 74 mm;
* the extend ramp is judged against the **landing plane**, so a leg landing on
  a top is not refused for "not approaching the ground".

Landing posture
---------------

The landing is at the **held** hip height above the landing surface, with the
theta that holds it -- not the fixed-theta stance height B5 used.  In the
gait-first planner the next stroke starts from the landing pose and re-solves
theta for the held height, so a landing at the fixed-theta height would put a
17 mm hip step at the seam.  Raising the surface and the hip by the same
amount leaves the leg's geometry identical, which is what makes the two heights
consistent by construction (B5's own argument).
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum

import numpy as np

from legwheel.planners.hybrid import RimId

from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    BodyRequirement2D,
    BodyRequirementKind,
    FrameRef2D,
    MotionSegment2D,
    PointContact2D,
    SegmentKind,
    SwingSampling2D,
    SwingShaping2D,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
    NominalPosture2D,
    RecoveryConfig2D,
    RecoverySwing2D,
    RollStroke2D,
    nominal_stroke_2d,
    run_recovery_swing_2d,
    standing_stroke_2d,
    theta_for_hip_z_2d,
)
from hybrid_note.scripts.experiments.day12_world_registration_2d import (
    forward_beta_for_orientation,
)
from hybrid_note.scripts.experiments.day13_b5_nominal_ascent_2d import (
    obstacle_posture_2d,
)

__all__ = [
    "TransitionKind2D",
    "NominalTransition2D",
    "run_nominal_transition_2d",
    "transition_segment_2d",
    "arc_start_landing_beta_rad",
    "held_landing_pose_2d",
    "SEGMENT_KIND_OF_TRANSITION",
]


class TransitionKind2D(str, Enum):
    UP = "UP"
    DOWN = "DOWN"
    TOP = "TOP"
    OVER = "OVER"
    #: Ground to ground with the obstacle in the scene but not crossed -- the
    #: nominal recovery, flown with the terrain visible so a landing near the
    #: face is checked against it.
    RECOVERY = "RECOVERY"


SEGMENT_KIND_OF_TRANSITION: dict[TransitionKind2D, SegmentKind] = {
    TransitionKind2D.UP: SegmentKind.SWING_UP,
    TransitionKind2D.DOWN: SegmentKind.SWING_DOWN,
    TransitionKind2D.TOP: SegmentKind.TOP_REPOSITION_SWING,
    TransitionKind2D.OVER: SegmentKind.SWING_OVER,
    TransitionKind2D.RECOVERY: SegmentKind.RECOVERY_SWING,
}

#: How far inside a surface a landing contact must be.  A contact within this
#: of an edge is a landing on the corner -- measured: a swing "succeeded" with
#: its contact 0.7 mm short of the front face at the top's height, i.e. balanced
#: on the leading edge -- and that is not a stance the next stroke can roll from.
DEFAULT_LANDING_EDGE_MARGIN_M: float = 0.010


@dataclass(frozen=True)
class NominalTransition2D:
    """One terrain-transition swing, or the reason there is none."""

    kind: TransitionKind2D
    swing: RecoverySwing2D | None
    refusal: str | None
    takeoff_hip_xz_m: tuple[float, float]
    takeoff_contact_xz_m: tuple[float, float]
    landing_contact_xz_m: tuple[float, float] | None
    obstacle_xwh_m: tuple[float, float, float]
    takeoff_surface_z_m: float
    landing_surface_z_m: float
    hip_advance_m: float
    landing_theta_rad: float
    landing_beta_rad: float

    @property
    def success(self) -> bool:
        return self.swing is not None

    @property
    def rotation_clearance_m(self) -> float | None:
        return None if self.swing is None else self.swing.min_clearance_m

    def as_dict(self) -> dict:
        landing = self.landing_contact_xz_m
        return {
            "kind": self.kind.value,
            "success": self.success,
            "refusal": self.refusal,
            "obstacle_x_start_mm": self.obstacle_xwh_m[0] * 1e3,
            "obstacle_top_length_mm": self.obstacle_xwh_m[1] * 1e3,
            "obstacle_height_mm": self.obstacle_xwh_m[2] * 1e3,
            "takeoff_hip_x_mm": self.takeoff_hip_xz_m[0] * 1e3,
            "takeoff_hip_z_mm": self.takeoff_hip_xz_m[1] * 1e3,
            "takeoff_contact_x_mm": self.takeoff_contact_xz_m[0] * 1e3,
            "takeoff_contact_z_mm": self.takeoff_contact_xz_m[1] * 1e3,
            "takeoff_surface_z_mm": self.takeoff_surface_z_m * 1e3,
            "landing_surface_z_mm": self.landing_surface_z_m * 1e3,
            "landing_contact_x_mm": None if landing is None else landing[0] * 1e3,
            "landing_contact_z_mm": None if landing is None else landing[1] * 1e3,
            "hip_advance_mm": self.hip_advance_m * 1e3,
            "landing_theta_deg": float(np.rad2deg(self.landing_theta_rad)),
            "landing_beta_deg": float(np.rad2deg(self.landing_beta_rad)),
            "frames": None if self.swing is None else len(self.swing.frames),
            "rotation_deg": (None if self.swing is None
                             else float(np.rad2deg(self.swing.rotation_rad))),
            "rotation_clearance_mm": (None if self.swing is None
                                      else self.swing.min_clearance_m * 1e3),
        }


# --------------------------------------------------------------------------
# Endpoints
# --------------------------------------------------------------------------


def arc_start_landing_beta_rad(posture: NominalPosture2D,
                               takeoff_beta_rad: float) -> float:
    """The next arc start forward of ``takeoff_beta_rad``.

    A landing at the arc's own start is what makes the following stroke a full
    one.  ``beta`` is a revolution counter and the leg turns one way, so of the
    family of values with the arc-start orientation exactly one is reachable
    without turning back; this is it.  From a full stroke it is one turn on
    from where the stroke began (the nominal recovery's own target); from a
    stroke cut short it is further round, and the swing simply rotates more.
    """

    arc_start = float(nominal_stroke_2d(posture).frames[0].beta_rad)
    return forward_beta_for_orientation(float(takeoff_beta_rad), arc_start)


def held_landing_pose_2d(posture: NominalPosture2D, landing_beta_rad: float,
                         landing_surface_z_m: float,
                         hip_z_above_surface_m: float | None = None,
                         ) -> tuple[float, float]:
    """``(theta, hip_z)`` of the landing: the hip this high above the surface.

    ``hip_z_above_surface_m`` defaults to the posture's held height.  A
    whole-body plan that raises an axle while a pair crosses hands in the
    axle's height instead, and the leg extends or crouches to reach the surface
    from there.  Solved on the flat scene at ground level and then raised by
    the surface height, because raising the surface and the hip together leaves
    the leg's geometry unchanged.
    """

    flat = replace(posture, ground_height_m=0.0, obstacle_xwh_m=None,
                   hold_hip_z_m=None, hold_hip_z_profile=None)
    if hip_z_above_surface_m is None:
        hip_z_above_surface_m = posture.held_hip_z_at(0.0)
    if hip_z_above_surface_m is None:
        hip_z = float(flat.hip_z_for_flat_stance(float(landing_beta_rad)))
        theta = float(flat.theta_rad)
    else:
        hip_z = float(hip_z_above_surface_m)
        solved = theta_for_hip_z_2d(flat, float(landing_beta_rad), hip_z)
        if solved is None:
            raise ValueError(
                f"no theta holds the hip at {hip_z * 1e3:.3f} mm above the "
                f"surface at beta {np.rad2deg(landing_beta_rad):.2f} deg.")
        theta = float(solved)
        # The solve stops within 1e-7 m of the target; the touchdown scene is
        # built at the exact stance height of the theta it found, so the
        # landing cannot penetrate the surface by the solver's residual.
        # Measured: 51 nm of penetration was enough for the contact query to
        # call a crouched landing a collision.
        hip_z = float(replace(flat, theta_rad=theta).hip_z_for_flat_stance(
            float(landing_beta_rad)))
    return theta, hip_z + float(landing_surface_z_m)


# --------------------------------------------------------------------------
# The transition
# --------------------------------------------------------------------------


def run_nominal_transition_2d(
    spec: SharedTerrainSpec2D,
    posture: NominalPosture2D,
    config: RecoveryConfig2D,
    *,
    kind: TransitionKind2D,
    takeoff_theta_rad: float,
    takeoff_beta_rad: float,
    takeoff_hip_xz_m: tuple[float, float],
    landing_hip_x_m: float,
    landing_beta_rad: float | None = None,
    landing_hip_z_above_surface_m: float | None = None,
    lift_hip_before_rotation: bool = False,
    landing_rim: RimId = RimId.FOOT,
    landing_edge_margin_m: float = DEFAULT_LANDING_EDGE_MARGIN_M,
) -> NominalTransition2D:
    """Fly one transition of ``kind`` from the takeoff pose to ``landing_hip_x_m``.

    The takeoff is the pose the leg is actually standing in -- the end of its
    stroke -- re-stood with the obstacle in the scene, because
    ``run_recovery_swing_2d`` takes the terrain from ``stroke.posture`` and a
    landing on a top that is not in the scene is a landing in empty air
    (Day 13 log 34.5).  The landing surface follows from ``kind``.
    """

    kind = TransitionKind2D(kind)
    obstacle = (float(spec.x_start_m), float(spec.top_length_m), float(spec.height_m))
    ground = float(spec.ground_height_m)
    top = float(spec.top_z_m)
    takeoff_surface = top if kind in (TransitionKind2D.DOWN, TransitionKind2D.TOP) else ground
    landing_surface = top if kind in (TransitionKind2D.UP, TransitionKind2D.TOP) else ground
    margin = float(landing_edge_margin_m)

    post = obstacle_posture_2d(posture, spec)
    hip_x0, hip_z0 = (float(v) for v in takeoff_hip_xz_m)
    # The obstacle enters through the STROKE (log 34.5).
    stroke = standing_stroke_2d(post, float(takeoff_theta_rad),
                                float(takeoff_beta_rad), hip_x0, hip_z0)
    takeoff_xz = ((float("nan"), float("nan")) if not stroke.frames
                  else tuple(float(v) for v in stroke.frames[-1].contact_xz_m))
    if landing_beta_rad is None:
        landing_beta_rad = arc_start_landing_beta_rad(posture, float(takeoff_beta_rad))
    advance = float(landing_hip_x_m) - hip_x0
    try:
        landing_theta, landing_hip_z = held_landing_pose_2d(
            posture, float(landing_beta_rad), landing_surface,
            landing_hip_z_above_surface_m)
        unreachable = None
    except ValueError as error:
        # The axle is too low (or too high) above the landing surface for
        # any theta: a refusal of this landing, not an error -- measured on
        # a rolling pair, whose axle is only 143 mm above the top mid-climb.
        landing_theta, landing_hip_z = float("nan"), float("nan")
        unreachable = f"LANDING_HEIGHT_UNREACHABLE:{error}"

    def refused(reason: str) -> NominalTransition2D:
        return NominalTransition2D(
            kind=kind, swing=None, refusal=reason,
            takeoff_hip_xz_m=(hip_x0, hip_z0),
            takeoff_contact_xz_m=takeoff_xz, landing_contact_xz_m=None,
            obstacle_xwh_m=obstacle, takeoff_surface_z_m=takeoff_surface,
            landing_surface_z_m=landing_surface, hip_advance_m=advance,
            landing_theta_rad=landing_theta, landing_beta_rad=float(landing_beta_rad))

    if unreachable is not None:
        return refused(unreachable)
    if not stroke.success:
        return refused(f"TAKEOFF_POSE_REFUSED:{stroke.stop_reason}")
    if advance < 0.0:
        return refused("LANDING_IS_BEHIND_THE_TAKEOFF:the body does not reverse")

    swing = run_recovery_swing_2d(
        stroke, replace(config, hip_advance_m=advance),
        beta_target_rad=float(landing_beta_rad),
        theta_touchdown_rad=landing_theta,
        hip_z_touchdown_m=landing_hip_z,
        lift_hip_before_rotation=bool(lift_hip_before_rotation),
        landing_surface_z_m=landing_surface,
        landing_rim=landing_rim,
        # A terrain transition folds and extends with the body standing
        # still; a nominal recovery keeps walking.
        hip_moves_during="all" if kind is TransitionKind2D.RECOVERY else "rotation",
    )
    if not swing.success:
        return refused(str(swing.failure_reason))
    landing = tuple(float(v) for v in swing.frames[-1].contact_xz_m)
    # The landing has to be on the surface the kind names, not merely on any
    # standable one: an UP that came down on the ground short of the face
    # would otherwise pass as a climb.
    if abs(landing[1] - landing_surface) > posture.contact_tolerance_m:
        return refused(
            f"LANDED_ON_THE_WRONG_SURFACE:z={landing[1] * 1e3:.3f} mm, "
            f"wanted {landing_surface * 1e3:.3f} mm")
    # And well inside that surface: a contact on a corner passes the height
    # check and is not a stance.
    x_min, x_max = float(spec.x_start_m), float(spec.x_max_m)
    if landing_surface == top:
        if not (x_min + margin <= landing[0] <= x_max - margin):
            return refused(
                f"LANDED_ON_AN_EDGE:contact x={landing[0] * 1e3:.3f} mm on a top "
                f"spanning [{x_min * 1e3:.1f}, {x_max * 1e3:.1f}] mm "
                f"(margin {margin * 1e3:.0f} mm)")
    elif x_min - margin < landing[0] < x_max + margin:
        return refused(
            f"LANDED_AGAINST_THE_OBSTACLE:contact x={landing[0] * 1e3:.3f} mm "
            f"within {margin * 1e3:.0f} mm of the block "
            f"[{x_min * 1e3:.1f}, {x_max * 1e3:.1f}] mm")
    return NominalTransition2D(
        kind=kind, swing=swing, refusal=None,
        takeoff_hip_xz_m=(hip_x0, hip_z0),
        takeoff_contact_xz_m=takeoff_xz, landing_contact_xz_m=landing,
        obstacle_xwh_m=obstacle, takeoff_surface_z_m=takeoff_surface,
        landing_surface_z_m=landing_surface, hip_advance_m=advance,
        landing_theta_rad=landing_theta, landing_beta_rad=float(landing_beta_rad))


# --------------------------------------------------------------------------
# Into the schema
# --------------------------------------------------------------------------


def _point(frame) -> PointContact2D:
    if frame.contact_xz_m is None:
        # An airborne frame (a transition written as fold / rotate / extend
        # pieces meets at such frames): the "point" is the hip itself, the
        # same on both sides of the boundary, which is all the chain asks.
        return PointContact2D(
            rim=RimId(frame.rim) if frame.rim else RimId.FOOT, alpha_rad=0.0,
            point_world_xz_m=np.asarray(frame.hip_xz_m, dtype=float),
            surface_id="airborne",
            theta_rad=float(frame.theta_rad), beta_rad=float(frame.beta_rad),
            hip_xz_m=np.asarray(frame.hip_xz_m, dtype=float))
    return PointContact2D(
        rim=frame.rim, alpha_rad=float(frame.alpha_rad),
        point_world_xz_m=np.asarray(frame.contact_xz_m, dtype=float),
        surface_id=str(frame.surface_id),
        theta_rad=float(frame.theta_rad), beta_rad=float(frame.beta_rad),
        hip_xz_m=np.asarray(frame.hip_xz_m, dtype=float))


def transition_segment_2d(
    swing: RecoverySwing2D,
    *,
    kind: SegmentKind,
    source_id: str,
    frame_offset: int = 0,
    phase_label: str | None = None,
    duration_s: float | None = None,
) -> MotionSegment2D:
    """A nominal-style transition as the terrain-transition kind it *is*.

    The schema requires a terrain-transition swing to carry Day 8--9's
    ``SwingSampling2D`` and ``SwingShaping2D``.  The sampling is real (frame
    count, arc samples, the largest joint step actually taken); the shaping
    knobs are recorded as **zero, meaning not used** -- a nominal swing has no
    apex to lift over, its clearance comes from retracting, and writing a
    plausible number would claim a knob that does not exist (B5's reasoning).
    """

    frames = swing.frames
    if len(frames) < 3:
        raise ValueError("a transition needs at least three frames to describe.")
    start, end = frames[0], frames[-1]
    hip = np.array([f.hip_xz_m for f in frames], dtype=float)
    steps = [max(abs(float(b.theta_rad - a.theta_rad)),
                 abs(float(b.beta_rad - a.beta_rad)))
             for a, b in zip(frames, frames[1:])]
    return MotionSegment2D(
        kind=kind,
        phase_label=kind.value if phase_label is None else phase_label,
        start_contact=_point(start),
        end_contact=_point(end),
        sampling=SwingSampling2D(
            arc_samples=int(swing.posture.arc_samples),
            sample_count=len(frames),
            leg_arc_samples=int(swing.posture.arc_samples),
            max_joint_step_rad=float(max(max(steps), 1e-9)),
        ),
        body_requirement=BodyRequirement2D(
            kind=BodyRequirementKind.LOWER_BOUND,
            x_range_m=(float(hip[:, 0].min()), float(hip[:, 0].max())),
            hip_z_min_m=float(hip[:, 1].max()),
        ),
        frames=FrameRef2D(
            source_id=source_id,
            indices=tuple(range(frame_offset, frame_offset + len(frames))),
        ),
        duration_s=duration_s,
        swing_shaping=SwingShaping2D(
            apex_clearance_m=0.0, liftoff_rise_m=0.0, touchdown_drop_m=0.0,
            duration_scale=1.0, mid_fractions=(0.35, 0.65)),
    )
