"""Day 12 Step 1: the nominal flat-ground Hybrid cycle.

Plan §8.  One cycle is ``FOOT_RIM_ROLL + RECOVERY_SWING``: a finite rolling
stroke along the usable foot-rim arc, then an airborne compact recovery that
carries the leg-wheel on in the *same* forward rotation sense until the foot
rim comes back down where the next stroke starts.

What this module reuses, and why it is not a second engine
----------------------------------------------------------

The plan asks, before implementation, whether the Day 8--9 Cartesian swing
generator can express the compact recovery.  **It cannot**, for two independent
reasons, and both are structural rather than missing-parameter:

1. **``theta`` is an IK output there, not an input.**  ``generate_swing_2d``
   builds a contact-point path and then solves IK per sample;
   :class:`SwingConstraints2D` carries only ``theta_min`` / ``theta_max``
   *bounds*.  There is no theta waypoint and no theta profile, so "retract to
   ``theta_compact``, then extend to ``theta_touchdown``" has no field to go
   in.  Day 10--11 trap 16 is the same fact seen from the other side: the
   touchdown theta is whatever the IK returns, which is why Step 2b had to
   rebuild the pose from the final sample and re-check it.

2. **The recovery is defined by a near-full turn of ``beta``.**  To bring the
   foot rim back to the start of its arc while still moving forward, the
   leg-wheel turns about 280 deg (measured below).  A Cartesian path from one
   contact point to another does not determine a winding number, and the IK
   will find the short way round.  ``SwingRequest2D`` has no way to demand a
   rotation sense, let alone a revolution.

Day 6--7 already has the right motion:
:func:`run_airborne_retract_and_foot_reset_branch_2d`, whose
``branch="forward_continuation"`` is exactly this rotation sense and whose
``theta_target_rad`` already defaults to wheel mode.  But it raises on
``terrain.obstacle is None``, requires the start frame to be a completed
right-rim top roll, and its touchdown filter accepts only surfaces of kind
``obstacle_top``.  It is the right *semantics* wired to one terrain.

So this module reuses the **primitives** both of those drivers are built from
-- ``build_single_leg_rolling_scene_2d``, ``query_single_leg_rolling_scene_2d``,
``_translated_scene_with_sample_on_target_2d``, ``_lowest_contact_sample``,
``_solve_flat_roll_rotation``, ``_airborne_beta_target_2d`` -- and adds no
geometry of its own.  The rolling stroke in particular is the *same* no-slip
step the Day 6--7 approach takes; the only thing that changes is the stop
condition, and even that is one the existing code already reports
(``_flat_roll_template`` returns ``None`` when the contact would leave the
current rim region -- the approach calls it ``RIM_ARC_EXHAUSTED``).

The measurements this design rests on
--------------------------------------

At the 2D pipeline's nominal posture (``theta = 60 deg``), on flat ground:

===============================  ==========
usable foot-rim arc              ``alpha`` -40 deg to +40 deg
contact advance over one stroke  202.5 mm
hip advance over one stroke      297.1 mm
hip height over one stroke       202.2 mm to 219.4 mm  (17.3 mm of travel)
rolling rotation                 79.7 deg
airborne rotation to close       280.3 deg
===============================  ==========

The two rotations sum to 360 deg, and that is not a coincidence: a cycle that
returns the contact to the same rim position **is** one revolution of the
leg-wheel.  It is what makes ``recovery_beta_target_2d`` a subtraction rather
than a search.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from functools import lru_cache

import numpy as np
from numpy.typing import NDArray

from legwheel.config import RobotParams

#: The motor travel, from the project's own constants.
THETA_MIN_RAD: float = float(np.deg2rad(RobotParams.MIN_THETA_DEG))
THETA_MAX_RAD: float = float(np.deg2rad(RobotParams.MAX_THETA_DEG))
from legwheel.planners.hybrid import RimId

from hybrid_note.scripts.experiments.cartesian_swing_contract_2d import (
    rim_alpha_limits_rad,
)
from hybrid_note.scripts.experiments.day10_11_concession_2d import (
    BodyRequirementKind,
)
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    BodyRequirement2D,
    FrameRef2D,
    MotionSegment2D,
    PointContact2D,
    RecoveryShaping2D,
    RollSampling2D,
    RollingContact2D,
    SegmentKind,
)
from hybrid_note.scripts.experiments.single_leg_rolling_scene_2d import (
    SingleLegRollingScene2D,
    _airborne_beta_target_2d,
    _translated_scene_with_sample_on_target_2d,
    build_single_leg_rolling_scene_2d,
    query_single_leg_rolling_scene_2d,
)
from hybrid_note.scripts.experiments.trailing_edge_roll_down_2d import (
    _candidate_for_sample,
    _flat_roll_template,
    _lowest_contact_sample,
    _solve_flat_roll_rotation,
)

__all__ = [
    "NominalPosture2D",
    "RecoveryConfig2D",
    "CycleFrame2D",
    "RollStroke2D",
    "RecoverySwing2D",
    "NominalCycle2D",
    "recovery_beta_target_2d",
    "run_foot_rim_roll_2d",
    "theta_for_hip_z_2d",
    "run_recovery_swing_2d",
    "run_nominal_cycles_2d",
    "cycle_segments_2d",
    "cycle_frame_rows",
    "cycle_summary_rows",
]


# --------------------------------------------------------------------------
# Configuration -- every number a parameter, none of them a branch on terrain
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class HipZProfile2D:
    """A body height to follow, indexed by **hip x**.

    Why hip x and not time: position scheduling already defines time as hip
    travel divided by one body speed, and ``hip_x = mount_x + body_x`` for
    every leg, so a profile indexed by hip x *is* a body trajectory -- read
    without needing a clock, and without the circularity of asking a stroke
    how long it takes before it has been generated (log 1.15, 1.21).

    Outside the sampled range the nearest end is held, deliberately: a stance
    leg that rolls a little past where the profile was sampled should keep
    standing at the last height it was told, not extrapolate one.
    """

    hip_x_m: tuple[float, ...]
    hip_z_m: tuple[float, ...]

    def __post_init__(self) -> None:
        if len(self.hip_x_m) != len(self.hip_z_m):
            raise ValueError("a profile needs one height per position.")
        if not self.hip_x_m:
            raise ValueError("an empty profile holds nothing.")
        if any(b < a for a, b in zip(self.hip_x_m, self.hip_x_m[1:])):
            raise ValueError("profile positions must not go backwards.")

    @classmethod
    def constant(cls, hip_z_m: float) -> "HipZProfile2D":
        """The flat case: one height, everywhere.

        This is what makes the change a generalisation rather than a
        replacement -- ``hold_hip_z_m`` is exactly this profile.
        """

        return cls((0.0,), (float(hip_z_m),))

    @property
    def is_constant(self) -> bool:
        return len(self.hip_z_m) == 1 or (
            max(self.hip_z_m) - min(self.hip_z_m) <= 0.0)

    def at(self, hip_x_m: float) -> float:
        """The height this profile asks for at ``hip_x_m``."""

        if len(self.hip_x_m) == 1:
            return float(self.hip_z_m[0])
        x = float(hip_x_m)
        if x <= self.hip_x_m[0]:
            return float(self.hip_z_m[0])
        if x >= self.hip_x_m[-1]:
            return float(self.hip_z_m[-1])
        return float(np.interp(x, self.hip_x_m, self.hip_z_m))


@dataclass(frozen=True)
class NominalPosture2D:
    """The posture the gait rolls in, and how finely it is stepped.

    ``theta_rad`` defaults to 60 deg because that is what **this pipeline**
    already stands at: Day 6--7's traversal approaches at it and Day 8--9's
    ``REGRESSION_THETA_RAD`` is the same value.  It is quoted, not chosen.

    It is deliberately *not* derived from ``TrajectoryParams.STAND_HEIGHT``.
    The project's three gait planners carry three different stand heights
    (0.25, 0.30, 0.31 m) and two different hip-height conventions -- one
    subtracts the foot radius, another adds ``ABAD_AXIS_OFFSET`` -- none of
    which this 2D model represents.  Picking one and calling it "the" nominal
    posture would be inventing agreement that does not exist.  The stand height
    that *does* follow from 60 deg here is 219.4 mm, and
    :meth:`hip_z_for_flat_stance` is how it is obtained rather than asserted.
    """

    theta_rad: float = float(np.deg2rad(60.0))
    gamma_rad: float = 0.0
    ground_height_m: float = 0.0
    #: Day 10--11 trap 2: below about 140 the seam width becomes a sampling
    #: artefact and a rim hand-over is falsely rejected.  241 is what Day 6--7
    #: Step 10R was computed with.
    arc_samples: int = 241
    #: How far the contact should advance per rolling step.
    roll_step_m: float = 0.004
    #: Fraction of the usable foot-rim arc to leave unspent, 0.0 to 1.0.
    #:
    #: Day 13.  The nominal stroke rolls until ``RIM_ARC_EXHAUSTED``, so the
    #: spare arc is 0.000 mm (measured) and no landing point can be adjusted
    #: without changing the whole cycle count -- which is why four separate
    #: attempts to stagger a pair of legs all failed by acting on *time*
    #: instead (log 25, 26.8).  ``run_foot_rim_roll_2d`` has always taken a
    #: ``max_distance_m``; its own docstring calls it "a distance request
    #: rather than a hard-wired full arc".  This is the cycle asking for one.
    #:
    #: Cost, measured: 5% reserve leaves 8.4 mm at 96% of today's body speed,
    #: 10% leaves 16.9 mm at 92%, 20% leaves 38.0 mm at 83%.  The staggers that
    #: were needed are 16.5 mm and 35.7 mm.
    #:
    #: **0.0 reproduces every frozen Day 12 number.**
    arc_reserve: float = 0.0
    #: The obstacle this posture's scenes contain, or ``None`` for flat ground.
    #:
    #: Day 13 B5.  ``scene_kwargs`` hard-coded ``obstacle_x_start_m=None``
    #: because Step 1's nominal cycle must not have a terrain feature leak into
    #: it -- that is still the default and still right.  But the project owner
    #: wants the *nominal swing* to be what climbs onto an obstacle ("I want to
    #: change to the nominal locomotion swing, the flat-ground one"), and a
    #: swing cannot land on a top that is not in its scene: asking for a
    #: touchdown 40 mm up on flat ground reaches into empty air, which is
    #: reported as ``TOUCHDOWN_IS_NOT_A_VALID_GROUND_CONTACT`` and reads like a
    #: refusal of the posture rather than of the question.
    #:
    #: ``(x_start_m, width_m, height_m)``.  ``None`` reproduces Step 1 exactly.
    obstacle_xwh_m: tuple[float, float, float] | None = None
    #: Day 13 opt-in: hold the hip at this height through the rolling stroke by
    #: modulating ``theta``.  ``None`` keeps the Day 12 behaviour -- a fixed
    #: ``theta``, and a hip that follows the rim arc up and down -- which every
    #: frozen Day 12 number was measured with.
    #:
    #: **Why this exists.**  Step 5 found the four-leg body height INFEASIBLE:
    #: three stance legs at different points of that arc demand three different
    #: body heights, and all three are hard requirements.  The arc is not a
    #: property of the robot, it is a consequence of rolling at a fixed
    #: ``theta`` -- the leg has the joint to cancel it, and the motor budget to
    #: spare (measured: about 32 deg/s of the 1980 deg/s the two joints share).
    hold_hip_z_m: float | None = None
    #: The whole-body generalisation of ``hold_hip_z_m``: a height that
    #: **changes with where the hip is**, so a stance leg follows one shared
    #: body trajectory instead of dictating its own.  ``None`` falls back to
    #: ``hold_hip_z_m``, and ``HipZProfile2D.constant(z)`` reproduces it
    #: exactly, so nothing measured before this existed moves.
    #:
    #: This is the hook the stance/body formulation change hangs on: the
    #: crossing leg's own frames run 143.8-198.0 mm while the flat nominal
    #: stance is 219.4 mm, and Step 5 could only call that INFEASIBLE because
    #: *both* legs were allowed to demand a body height.  A leg reading a
    #: profile demands nothing (log 1.20, 1.21).
    hold_hip_z_profile: HipZProfile2D | None = None
    surface_offset_m: float = 1e-9
    contact_tolerance_m: float = 1e-3
    collision_tolerance_m: float = 1e-3

    def __post_init__(self) -> None:
        if not np.isclose(self.gamma_rad, 0.0, atol=1e-12):
            raise ValueError("Day 12 fixes gamma = 0; it is Day 13-14 that frees it.")
        if self.arc_samples < 3:
            raise ValueError("arc_samples must be at least 3.")
        if self.roll_step_m <= 0.0:
            raise ValueError("roll_step_m must be positive.")
        low = float(np.deg2rad(RobotParams.MIN_THETA_DEG))
        high = float(np.deg2rad(RobotParams.MAX_THETA_DEG))
        if not (low - 1e-12 <= self.theta_rad <= high + 1e-12):
            raise ValueError(
                f"theta {np.rad2deg(self.theta_rad):.1f} deg is outside the "
                f"joint range [{RobotParams.MIN_THETA_DEG}, "
                f"{RobotParams.MAX_THETA_DEG}] deg."
            )

    @property
    def holds_hip_z(self) -> bool:
        """Whether this posture levels the hip at all."""

        return (self.hold_hip_z_profile is not None
                or self.hold_hip_z_m is not None)

    def held_hip_z_at(self, hip_x_m: float) -> float | None:
        """The height to hold at ``hip_x_m``; ``None`` when not levelling.

        The profile wins when both are set, and that is the only precedence
        worth having: a caller that supplies a trajectory means the trajectory.
        """

        if self.hold_hip_z_profile is not None:
            return self.hold_hip_z_profile.at(hip_x_m)
        if self.hold_hip_z_m is not None:
            return float(self.hold_hip_z_m)
        return None

    @property
    def scene_kwargs(self) -> dict:
        """Flat ground.  ``obstacle_x_start_m=None`` is the builder's own way
        of saying there is no obstacle -- Step 1 does not register one, so a
        terrain feature cannot leak into the nominal cycle by accident."""

        if self.obstacle_xwh_m is None:
            return {
                "gamma_rad": self.gamma_rad,
                "ground_height_m": self.ground_height_m,
                "obstacle_x_start_m": None,
                "arc_samples": self.arc_samples,
            }
        x0, width, height = self.obstacle_xwh_m
        return {
            "gamma_rad": self.gamma_rad,
            "ground_height_m": self.ground_height_m,
            "obstacle_x_start_m": float(x0),
            "obstacle_width_m": float(width),
            "obstacle_height_m": float(height),
            "arc_samples": self.arc_samples,
        }

    def hip_z_for_flat_stance(self, beta_rad: float) -> float:
        """Hip height that puts the lowest leg point on the ground.

        Solved from the sampled geometry, not from a stand-height constant, so
        it stays correct for any ``theta`` the caller sets.
        """

        template = build_single_leg_rolling_scene_2d(
            self.theta_rad, float(beta_rad), 0.0, 0.0, **self.scene_kwargs
        )
        lowest = float(template.geometry.points_hip_xz_m[:, 1].min())
        return float(self.ground_height_m - lowest + self.surface_offset_m)

    def scene(self, beta_rad: float, hip_x_m: float, hip_z_m: float,
              *, theta_rad: float | None = None) -> SingleLegRollingScene2D:
        return build_single_leg_rolling_scene_2d(
            self.theta_rad if theta_rad is None else float(theta_rad),
            float(beta_rad), float(hip_x_m), float(hip_z_m), **self.scene_kwargs
        )

    def query(self, scene: SingleLegRollingScene2D):
        return query_single_leg_rolling_scene_2d(
            scene,
            contact_tolerance_m=self.contact_tolerance_m,
            collision_tolerance_m=self.collision_tolerance_m,
        )


@dataclass(frozen=True)
class RecoveryConfig2D:
    """The airborne half's knobs.  Plan §0.2: 17 deg lives *here*, once.

    ``theta_compact_rad`` defaults to wheel mode's value, which is what the
    Day 12 MVP asks for -- but it is a **different quantity** from
    ``WHEEL_MODE_THETA_RAD``.  That constant is what ``WHEEL_ROLL`` *means*;
    this is how far the leg retracts to get out of the way.  They coincide
    today and may not tomorrow, so they are not the same name.
    """

    theta_compact_rad: float = float(np.deg2rad(RobotParams.THETA0_DEG))
    theta_step_rad: float = float(np.deg2rad(2.0))
    beta_step_rad: float = float(np.deg2rad(4.0))
    #: How far the hip travels while the leg is airborne.  Zero by default:
    #: Step 1 does not model the body advancing during recovery, because the
    #: body trajectory is Step 5's business and a number invented here would be
    #: read as a result.  Net forward progress does not depend on it -- the hip
    #: is already 297 mm ahead by liftoff.
    hip_advance_m: float = 0.0
    #: The clearance the airborne leg must keep from the terrain.
    min_clearance_m: float = 0.010

    def __post_init__(self) -> None:
        low = float(np.deg2rad(RobotParams.MIN_THETA_DEG))
        high = float(np.deg2rad(RobotParams.MAX_THETA_DEG))
        if not (low - 1e-12 <= self.theta_compact_rad <= high + 1e-12):
            raise ValueError("theta_compact_rad is outside the joint range.")
        for name in ("theta_step_rad", "beta_step_rad", "min_clearance_m"):
            if float(getattr(self, name)) <= 0.0:
                raise ValueError(f"{name} must be positive.")
        if self.hip_advance_m < 0.0:
            raise ValueError(
                "hip_advance_m must not be negative: the body does not reverse "
                "during a nominal recovery."
            )


# --------------------------------------------------------------------------
# Frames
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class CycleFrame2D:
    """One sampled pose of the nominal cycle, rolling or airborne."""

    index: int
    phase: str
    theta_rad: float
    beta_rad: float
    hip_xz_m: tuple[float, float]
    airborne: bool
    #: Contact data, present only while in stance.
    rim: str | None
    alpha_rad: float | None
    contact_xz_m: tuple[float, float] | None
    surface_id: str | None
    #: Smallest gap from any sampled leg point to the terrain.  Zero in stance
    #: by definition; the number that matters while airborne.
    clearance_m: float
    collision: bool

    def as_dict(self) -> dict:
        return {
            "index": self.index,
            "phase": self.phase,
            "airborne": self.airborne,
            "theta_deg": float(np.rad2deg(self.theta_rad)),
            "beta_deg": float(np.rad2deg(self.beta_rad)),
            "hip_x_mm": self.hip_xz_m[0] * 1e3,
            "hip_z_mm": self.hip_xz_m[1] * 1e3,
            "rim": self.rim,
            "alpha_deg": (
                None if self.alpha_rad is None else float(np.rad2deg(self.alpha_rad))
            ),
            "contact_x_mm": (
                None if self.contact_xz_m is None else self.contact_xz_m[0] * 1e3
            ),
            "contact_z_mm": (
                None if self.contact_xz_m is None else self.contact_xz_m[1] * 1e3
            ),
            "surface_id": self.surface_id,
            "clearance_mm": self.clearance_m * 1e3,
            "collision": self.collision,
        }


def _clearance_m(scene: SingleLegRollingScene2D, ground_height_m: float) -> float:
    """Lowest sampled leg point above the ground, links included.

    Links are included because a compact posture swings the linkage as well as
    the rim, and a rim-only check would call a linkage strike clearance.
    """

    points = np.asarray(scene.geometry.points_world_xz_m, dtype=float)
    lowest = float(points[:, 1].min())
    for segment in scene.geometry.link_segments_world_xz_m:
        lowest = min(lowest, float(np.asarray(segment, dtype=float)[:, 1].min()))
    return float(lowest - ground_height_m)


def _standable_surface_ids(scene: SingleLegRollingScene2D) -> tuple[str, ...]:
    """The surfaces a leg may legitimately be *standing on*.

    The ground, and any obstacle top.  Day 13 B5: a nominal-style swing that
    climbs an obstacle lands on the top, and a leg that has landed there is in
    stance on it -- restricting stance to ``ground_surface_id`` is what made a
    perfectly good top landing raise "has no ground contact on its support
    sample" one frame after the touchdown check had already accepted it.

    A landing belongs on the ground or on a top, never on a vertical face, so
    the set is selected by suffix over the scene's own surface ids ('ground',
    'day6_7_obstacle_top', '..._front', '..._back') rather than by a guessed
    attribute name -- ``obstacle_top_surface_id`` does not exist.
    """

    return tuple(
        sid for sid in scene.terrain.surface_ids
        if sid == scene.terrain.ground_surface_id or sid.endswith("_top")
    )


def _stance_frame(
    index: int, phase: str, posture: NominalPosture2D,
    scene: SingleLegRollingScene2D, sample: int,
) -> CycleFrame2D:
    result = posture.query(scene)
    candidate = _candidate_for_sample(
        result, sample, surface_ids=_standable_surface_ids(scene)
    )
    if candidate is None:
        raise ValueError(
            f"frame {index} ({phase}) has no ground contact on its support "
            "sample; the stroke driver must not have got here."
        )
    return CycleFrame2D(
        index=index, phase=phase,
        theta_rad=float(scene.theta_rad), beta_rad=float(scene.beta_rad),
        hip_xz_m=tuple(float(v) for v in scene.hip_pose.position_world_xz_m),
        airborne=False,
        rim=RimId(candidate.rim).value,
        alpha_rad=float(candidate.alpha_rad),
        contact_xz_m=tuple(float(v) for v in candidate.point_world_xz_m),
        surface_id=str(candidate.terrain_surface_id),
        clearance_m=0.0,
        collision=bool(result.collision),
    )


def _leg_points_world_xz_m(scene: SingleLegRollingScene2D,
                           link_samples: int = 5) -> np.ndarray:
    """Every sampled point of the leg: the three rims plus the linkage.

    Links are sampled along their length, not only at their ends, because a
    straight link can cross an obstacle corner between two endpoints that are
    both clear of it.
    """

    points = [np.asarray(scene.geometry.points_world_xz_m, dtype=float)]
    fractions = np.linspace(0.0, 1.0, int(link_samples))[:, None]
    for segment in scene.geometry.link_segments_world_xz_m:
        seg = np.asarray(segment, dtype=float)
        if len(seg) >= 2:
            a, b = seg[0], seg[-1]
            points.append(a[None, :] + fractions * (b - a)[None, :])
    return np.vstack(points)


def _terrain_clearance_m(scene: SingleLegRollingScene2D) -> float:
    """Smallest signed gap from any leg point to the terrain **solid**.

    Day 14.  :func:`_clearance_m` measures against the ground plane alone, so an
    airborne leg that passed through an obstacle's front face reported a healthy
    clearance and ``collision=False`` -- the swing generator could not see the
    obstacle it was asked to clear.  This measures against the union of the
    ground half-space and the rectangle: the signed distance to a box is exact
    for the sampled points, positive outside and negative inside, and the union
    of the two solids is the minimum of the two distances.

    With no obstacle in the scene it reduces to :func:`_clearance_m` exactly,
    which is what keeps every frozen flat number where it was.
    """

    terrain = scene.terrain
    points = _leg_points_world_xz_m(scene)
    ground = float(terrain.ground_height_m)
    gap = points[:, 1] - ground
    obstacle = terrain.obstacle
    if obstacle is not None:
        x0, x1 = float(obstacle.x_min_m), float(obstacle.x_max_m)
        z0, z1 = ground, ground + float(obstacle.height_m)
        x, z = points[:, 0], points[:, 1]
        dx = np.maximum(np.maximum(x0 - x, x - x1), 0.0)
        dz = np.maximum(np.maximum(z0 - z, z - z1), 0.0)
        outside = np.hypot(dx, dz)
        inside = -np.minimum(np.minimum(x - x0, x1 - x), np.minimum(z - z0, z1 - z))
        box = np.where(outside > 0.0, outside, inside)
        gap = np.minimum(gap, box)
    return float(gap.min())


def _airborne_frame(
    index: int, phase: str, posture: NominalPosture2D,
    scene: SingleLegRollingScene2D,
) -> CycleFrame2D:
    # Flat scene: the ground plane is the terrain, as Step 1 measured it.
    # With an obstacle in the scene the gap is to the terrain solid, and a
    # negative one is a collision -- the thing an airborne frame used to be
    # unable to report (Day 14 Step 2).
    if scene.terrain.obstacle is None:
        clearance = _clearance_m(scene, posture.ground_height_m)
        collision = False
    else:
        clearance = _terrain_clearance_m(scene)
        collision = bool(clearance < -posture.collision_tolerance_m)
    return CycleFrame2D(
        index=index, phase=phase,
        theta_rad=float(scene.theta_rad), beta_rad=float(scene.beta_rad),
        hip_xz_m=tuple(float(v) for v in scene.hip_pose.position_world_xz_m),
        airborne=True, rim=None, alpha_rad=None, contact_xz_m=None,
        surface_id=None,
        clearance_m=clearance,
        collision=collision,
    )


# --------------------------------------------------------------------------
# The rolling stroke
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class RollStroke2D:
    """One finite foot-rim rolling stroke on flat ground."""

    frames: tuple[CycleFrame2D, ...]
    success: bool
    stop_reason: str
    posture: NominalPosture2D
    #: The scene the stroke ends in, so a recovery can start from it without
    #: rebuilding a pose from numbers.
    final_scene: SingleLegRollingScene2D
    final_sample: int

    @property
    def start(self) -> CycleFrame2D:
        return self.frames[0]

    @property
    def end(self) -> CycleFrame2D:
        return self.frames[-1]

    @property
    def contact_advance_m(self) -> float:
        return float(self.end.contact_xz_m[0] - self.start.contact_xz_m[0])

    @property
    def hip_advance_m(self) -> float:
        return float(self.end.hip_xz_m[0] - self.start.hip_xz_m[0])

    @property
    def rotation_rad(self) -> float:
        """Forward rotation, positive.  ``beta`` decreases while rolling
        forward, so the sign is flipped once, here, rather than at each use."""

        return float(self.start.beta_rad - self.end.beta_rad)

    @property
    def hip_z_travel_m(self) -> float:
        z = [f.hip_xz_m[1] for f in self.frames]
        return float(max(z) - min(z))

    @property
    def alpha_range_rad(self) -> tuple[float, float]:
        a = [f.alpha_rad for f in self.frames]
        return (float(min(a)), float(max(a)))

    @property
    def hip_z_tracking_error_m(self) -> float:
        """Worst gap between the hip and the height the posture asked for.

        Zero when nothing was asked.  For a **constant** hold it is the
        levelling residual (measured 0.000176 mm).  For a **sloped** profile it
        also carries the one-step lag: the theta for each step is solved
        against the height at the hip the leg is standing at, which is one roll
        step behind the pose being solved.  Reported rather than assumed
        negligible -- a 54 mm climb over 658 mm of hip travel is 0.08 mm per mm,
        and a step is 6.4 mm of hip.
        """

        worst = 0.0
        for frame in self.frames:
            want = self.posture.held_hip_z_at(float(frame.hip_xz_m[0]))
            if want is None:
                return 0.0
            worst = max(worst, abs(float(frame.hip_xz_m[1]) - float(want)))
        return worst


def run_foot_rim_roll_2d(
    posture: NominalPosture2D | None = None,
    *,
    start_beta_rad: float | None = None,
    hip_x_m: float = 0.0,
    max_distance_m: float | None = None,
    max_steps: int = 500,
) -> RollStroke2D:
    """Roll the foot rim forward until the usable arc runs out.

    ``start_beta_rad`` defaults to the start of the foot arc, so the stroke is
    the longest one the rim allows.  The stop condition is
    ``_solve_flat_roll_rotation`` returning ``None`` -- the same condition the
    Day 6--7 approach reports as ``RIM_ARC_EXHAUSTED_DURING_APPROACH`` -- which
    is a property of the rim, not of any terrain feature.

    ``max_distance_m`` stops the stroke early instead.  That is what a leg
    approaching a transition needs, and it is why this is a distance request
    rather than a hard-wired full arc.
    """

    posture = NominalPosture2D() if posture is None else posture
    kwargs = posture.scene_kwargs
    # ``arc_reserve`` turns "roll until the rim runs out" into "roll all but
    # this fraction of it".  Applied only when the caller did not ask for a
    # distance of its own: an explicit ``max_distance_m`` is already a more
    # specific request (an approach aligning on a transition, say), and
    # shortening it further would silently miss the target it was aiming at.
    #
    # The full arc is measured once, with the reserve off, so the fraction is
    # of the arc the rim actually has rather than of a guess.
    if max_distance_m is None and posture.arc_reserve > 0.0:
        full = run_foot_rim_roll_2d(
            replace(posture, arc_reserve=0.0),
            start_beta_rad=start_beta_rad, hip_x_m=hip_x_m,
            max_steps=max_steps,
        )
        if full.success:
            max_distance_m = float(
                full.contact_advance_m * (1.0 - posture.arc_reserve))
    beta = (
        _foot_arc_start_beta_rad(posture) if start_beta_rad is None
        else float(start_beta_rad)
    )
    theta = float(posture.theta_rad)
    held_start = posture.held_hip_z_at(float(hip_x_m))
    if held_start is None:
        hip_z = posture.hip_z_for_flat_stance(beta)
    else:
        # Start already at the held height, or the stroke would spend its first
        # steps climbing to it and the "held" claim would be false at the ends.
        hip_z = float(held_start)
        solved_theta = theta_for_hip_z_2d(posture, beta, hip_z)
        if solved_theta is None:
            return RollStroke2D(
                (), False, "LEVELLING_UNREACHABLE_AT_START", posture,
                posture.scene(beta, hip_x_m, posture.hip_z_for_flat_stance(beta)),
                0,
            )
        theta = solved_theta
        # The solve stops within 1e-7 m; if the theta it found stands a hair
        # *higher* than the held height, a scene built at the held height has
        # the rim inside the surface by that hair, and the contact query calls
        # any penetration a collision (measured: 51 nm was enough).  Lift by
        # the residual in that case only -- never lower, and never on a pose
        # that already stands clear, so every frozen flat number is untouched.
        exact = float(replace(posture, theta_rad=theta).hip_z_for_flat_stance(beta))
        if exact > hip_z:
            hip_z = exact
    scene = posture.scene(beta, hip_x_m, hip_z, theta_rad=theta)
    sample = _lowest_contact_sample(scene.geometry)

    frames = [_stance_frame(0, "FOOT_RIM_ROLL", posture, scene, sample)]
    if frames[0].collision:
        return RollStroke2D(tuple(frames), False, "STROKE_START_IN_COLLISION",
                            posture, scene, sample)
    start_contact_x = float(frames[0].contact_xz_m[0])

    stop_reason = "STEP_LIMIT_REACHED"
    for _ in range(max_steps):
        if (
            max_distance_m is not None
            and float(frames[-1].contact_xz_m[0]) - start_contact_x >= max_distance_m
        ):
            stop_reason = "REQUESTED_DISTANCE_REACHED"
            break
        contact = scene.geometry.points_world_xz_m[sample]
        solved = _solve_flat_roll_rotation(
            theta, beta, sample, scene.geometry.points_hip_xz_m, kwargs,
            target_advance_m=posture.roll_step_m,
            # Forward rolling decreases beta.  This is the sign the whole
            # pipeline uses -- Day 6--7's approach passes the same -1.0 -- and
            # keeping it identical is what makes "same forward rolling
            # direction" checkable rather than a matter of convention.
            beta_direction=-1.0,
            coarse_step_rad=float(np.deg2rad(1.0)),
            max_rotation_rad=float(np.deg2rad(30.0)),
            iterations=24,
            rotation_resolution_rad=float(np.deg2rad(0.02)),
        )
        if solved is None:
            stop_reason = "RIM_ARC_EXHAUSTED"
            break
        template, next_sample, arc, rotation = solved

        if posture.holds_hip_z:
            # Re-solve the step with the theta that holds the hip at the new
            # beta.  ``_flat_roll_template`` averages the rim arc measured in
            # the old and the new geometry, so a step that changes shape is
            # still no-slip by its own construction -- which is why the arc is
            # recomputed here rather than carried over from the fixed-theta
            # solve above.
            #
            # Read the height at the hip the step is *going* to, not the one
            # it is leaving.  Reading it at the current hip leaves a one-step
            # lag -- measured 1.85 mm on a 0.25 mm/mm slope, which is exactly
            # the 6.4 mm hip step times the slope, so it is the lag and not
            # noise.  The step ahead is taken from the step just taken rather
            # than from a constant: it is the same quantity, already measured.
            here = float(frames[-1].hip_xz_m[0])
            if len(frames) >= 2:
                step = here - float(frames[-2].hip_xz_m[0])
            else:
                # No previous step to learn from.  Rather than read the height
                # a step too early -- which put the whole tracking error into
                # frame 1 and nowhere else, measured 0.638 mm of a 0.638 mm
                # worst case -- place the fixed-theta template on its target
                # and ask *it* where the hip lands.  That is a translation of
                # geometry already computed, not another solve.
                probe_target = np.array(
                    [float(contact[0]) + arc,
                     posture.ground_height_m + posture.surface_offset_m],
                    dtype=float,
                )
                probe = _translated_scene_with_sample_on_target_2d(
                    template, next_sample, probe_target)
                step = float(probe.hip_pose.position_world_xz_m[0]) - here
            held = posture.held_hip_z_at(here + step)
            next_theta = theta_for_hip_z_2d(
                posture, beta - rotation, float(held),
                theta_seed_rad=theta,
            )
            if next_theta is None:
                stop_reason = "LEVELLING_UNREACHABLE"
                break
            relevelled = _flat_roll_template(
                next_theta, beta - rotation, sample,
                scene.geometry.points_hip_xz_m, kwargs,
            )
            if relevelled is None:
                stop_reason = "LEVELLING_LOST_THE_STEP"
                break
            template, next_sample, arc = relevelled
            theta = next_theta

        target = np.array(
            [float(contact[0]) + arc,
             posture.ground_height_m + posture.surface_offset_m],
            dtype=float,
        )
        next_scene = _translated_scene_with_sample_on_target_2d(
            template, next_sample, target
        )
        next_result = posture.query(next_scene)
        next_candidate = _candidate_for_sample(
            next_result, next_sample,
            surface_ids=(next_scene.terrain.ground_surface_id,),
        )
        if next_result.collision and next_scene.terrain.obstacle is not None:
            # Day 14: the next step would put the leg into the obstacle.  The
            # frames so far are a legitimate, shortened stroke -- this is how
            # a leg learns the latest takeoff its terrain allows -- so it is a
            # success with its own reason, not a lost contact.  Never reached
            # on flat ground: a flat scene has nothing to collide with.
            stop_reason = "OBSTACLE_AHEAD"
            break
        if next_result.collision or next_candidate is None:
            stop_reason = "LOST_GROUND_CONTACT"
            break
        scene, sample, beta = next_scene, next_sample, beta - rotation
        frames.append(
            _stance_frame(len(frames), "FOOT_RIM_ROLL", posture, scene, sample)
        )

    success = stop_reason in ("RIM_ARC_EXHAUSTED", "REQUESTED_DISTANCE_REACHED",
                              "OBSTACLE_AHEAD")
    return RollStroke2D(tuple(frames), success, stop_reason, posture, scene, sample)


#: ``d(hip_z)/d(theta)`` near the nominal posture, measured at 1.796 mm/deg
#: between 50 and 70 degrees.  A seed for the secant, not a model: the solve
#: below iterates on the real geometry and reports its own residual.
_HIP_Z_PER_THETA_M_PER_RAD: float = float(1.796e-3 / np.deg2rad(1.0))


def theta_for_hip_z_2d(
    posture: NominalPosture2D,
    beta_rad: float,
    target_hip_z_m: float,
    *,
    theta_seed_rad: float | None = None,
    tolerance_m: float = 1e-7,
    max_iterations: int = 40,
) -> float | None:
    """The ``theta`` that puts the hip at ``target_hip_z_m`` at this ``beta``.

    Solved on the sampled geometry through
    :meth:`NominalPosture2D.hip_z_for_flat_stance`, so it stays true to whatever
    the leg model says rather than to a linearisation.  ``None`` when the
    solution leaves the motor's travel -- a refusal, not a clamp: a clamped
    theta would silently stop holding the hip and report success.
    """

    def error_at(value: float) -> float:
        return float(replace(posture, theta_rad=value).hip_z_for_flat_stance(
            float(beta_rad))) - float(target_hip_z_m)

    theta = float(posture.theta_rad if theta_seed_rad is None else theta_seed_rad)
    for _ in range(max_iterations):
        if not (THETA_MIN_RAD <= theta <= THETA_MAX_RAD):
            # A Newton step outside the travel cannot be evaluated (the
            # posture refuses such a theta); the bisection below decides.
            break
        error = error_at(theta)
        if abs(error) <= tolerance_m:
            if not (THETA_MIN_RAD <= theta <= THETA_MAX_RAD):
                return None
            return theta
        theta -= error / _HIP_Z_PER_THETA_M_PER_RAD
    # Newton with a fixed slope gives up on the far end of the travel (measured
    # 2026-09-08: theta 145.47 deg for a hip 319.4 mm up at the arc-end beta
    # came back None after 12 steps and solved in 30), and a None here reads
    # downstream as "beyond the joint's travel".  So bisect over the travel:
    # hip height is monotone in theta on this leg, and None is then only for
    # a height the leg really cannot reach.
    lo, hi = float(THETA_MIN_RAD), float(THETA_MAX_RAD)
    e_lo, e_hi = error_at(lo), error_at(hi)
    if abs(e_lo) <= tolerance_m:
        return lo
    if abs(e_hi) <= tolerance_m:
        return hi
    if e_lo * e_hi > 0.0:
        return None
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        e_mid = error_at(mid)
        if abs(e_mid) <= tolerance_m:
            return mid
        if e_mid * e_lo > 0.0:
            lo, e_lo = mid, e_mid
        else:
            hi, e_hi = mid, e_mid
    return None


def _foot_arc_start_beta_rad(posture: NominalPosture2D) -> float:
    """The beta whose ground contact sits at the *back* end of the foot arc.

    Two wrong answers were measured on the way to this one, and both are worth
    keeping because they are the kind of thing that reads as correct.

    **Not by rolling backwards.**  ``_flat_roll_template`` requires the new
    support sample to be *greater* than the old one, so it only steps forward
    and a backward call returns ``None`` immediately.  The first version did
    that and silently reported the arc start as ``beta = 0`` -- the middle of
    the arc -- halving every stroke.

    **Not by where the foot region ends either.**  Rotating further back does
    not keep rolling: past ``beta ~= 40 deg`` the support sample **stops
    moving** and stays pinned on the foot rim's first sample, the foot/left
    seam corner, while the leg turns about it.  That is a corner pivot -- Day
    10--11's :class:`RollingMode.CORNER_PIVOT`, and its own trap 31 -- and it
    runs another 20 degrees, to ``beta ~= 60 deg``, before the left rim takes
    over.  Starting a stroke there buys **no rolling distance at all** and
    costs 28.5 mm of extra hip drop, measured: ``hip_z`` falls from 202.0 mm to
    173.5 mm across the pivot with ``alpha`` pinned at -40 deg the whole way.

    So the arc start is where **rolling** starts: the smallest beta at which the
    contact reaches the foot rim's own lower alpha limit, taken from
    :func:`rim_alpha_limits_rad` rather than written down here.  Beyond it the
    leg pivots; before it, it rolls.
    """

    kwargs = posture.scene_kwargs
    alpha_min = float(rim_alpha_limits_rad(RimId.FOOT)[0])
    tolerance = float(np.deg2rad(0.01))

    def alpha_at(beta: float) -> float | None:
        template = build_single_leg_rolling_scene_2d(
            posture.theta_rad, float(beta), 0.0, 0.0, **kwargs
        )
        hip_z = posture.hip_z_for_flat_stance(beta)
        scene = posture.scene(beta, 0.0, hip_z)
        sample = _lowest_contact_sample(scene.geometry)
        if str(scene.geometry.contact_regions[sample]) != RimId.FOOT.value:
            return None
        candidate = _candidate_for_sample(
            posture.query(scene), sample,
            surface_ids=(scene.terrain.ground_surface_id,),
        )
        return None if candidate is None else float(candidate.alpha_rad)

    def still_rolling(beta: float) -> bool:
        alpha = alpha_at(beta)
        return alpha is not None and alpha > alpha_min + tolerance

    if not still_rolling(0.0):
        raise ValueError(
            f"at theta = {np.rad2deg(posture.theta_rad):.1f} deg the leg is "
            "already at the foot rim's alpha limit with beta = 0; there is no "
            "foot-rim rolling stroke to start."
        )
    coarse = float(np.deg2rad(1.0))
    rolling, pivoting = 0.0, None
    for step in range(1, 181):
        beta = step * coarse
        if still_rolling(beta):
            rolling = beta
        else:
            pivoting = beta
            break
    if pivoting is None:
        raise ValueError("the foot rim's alpha limit was never reached.")
    resolution = float(np.deg2rad(0.05))
    while pivoting - rolling > resolution:
        middle = 0.5 * (rolling + pivoting)
        if still_rolling(middle):
            rolling = middle
        else:
            pivoting = middle
    return float(pivoting)


# --------------------------------------------------------------------------
# The recovery
# --------------------------------------------------------------------------


def recovery_beta_target_2d(stroke: RollStroke2D) -> float:
    """The beta the recovery has to reach: one full forward revolution on from
    where the stroke *started*.

    A cycle that puts the contact back at the arc position it began at is, by
    definition, one revolution of the leg-wheel -- so this is a subtraction and
    not a search.  ``2*pi`` is subtracted because forward rolling decreases
    beta; the result is always below the stroke's end beta, which is what
    "keeps turning the same way" means.

    Day 6--7's :func:`_airborne_beta_target_2d` is the special case of this for
    a stroke that starts at ``alpha = 0``: it returns the next multiple of
    ``2*pi`` below, which is the same value when the start beta is zero.  A
    test asserts that agreement rather than leaving it to the reader.
    """

    return float(stroke.start.beta_rad - 2.0 * np.pi)


@dataclass(frozen=True)
class RecoverySwing2D:
    """One airborne compact recovery between two rolling strokes."""

    frames: tuple[CycleFrame2D, ...]
    success: bool
    failure_reason: str | None
    config: RecoveryConfig2D
    posture: NominalPosture2D
    final_scene: SingleLegRollingScene2D
    final_sample: int | None

    @property
    def start(self) -> CycleFrame2D:
        return self.frames[0]

    @property
    def end(self) -> CycleFrame2D:
        return self.frames[-1]

    @property
    def min_clearance_m(self) -> float:
        """Smallest terrain gap during the **rotation**.

        Not over every airborne frame: the retract and extend ramps start and
        end at contact, so a minimum taken over them is zero for every recovery
        and answers nothing.  The rotation is the phase where the leg swings
        past the ground, which is the phase where a clearance means something.
        """

        rotating = [
            f.clearance_m for f in self.frames if f.phase == "RECOVERY_ROTATE"
        ]
        return float(min(rotating)) if rotating else 0.0

    @property
    def ramp_min_clearance_m(self) -> float:
        """Smallest gap over the retract/extend ramps -- expected to be ~0.

        Reported separately so that "the leg touches the ground at liftoff and
        touchdown" stays visible instead of being averaged into the number
        above.
        """

        ramps = [
            f.clearance_m for f in self.frames
            if f.phase in ("RECOVERY_RETRACT", "RECOVERY_EXTEND")
        ]
        return float(min(ramps)) if ramps else 0.0

    @property
    def rotation_rad(self) -> float:
        return float(self.start.beta_rad - self.end.beta_rad)

    @property
    def theta_min_rad(self) -> float:
        return float(min(f.theta_rad for f in self.frames))


def run_recovery_swing_2d(
    stroke: RollStroke2D,
    config: RecoveryConfig2D | None = None,
    *,
    beta_target_rad: float | None = None,
    theta_touchdown_rad: float | None = None,
    hip_z_touchdown_m: float | None = None,
    lift_hip_before_rotation: bool = False,
    hip_lift_step_m: float = 0.004,
    landing_surface_z_m: float | None = None,
    landing_rim: RimId = RimId.FOOT,
    hip_moves_during: str = "all",
) -> RecoverySwing2D:
    """Retract, carry the rotation on forward, extend, and land.

    ``hip_moves_during``: ``"all"`` advances the hip (and height) linearly
    through every airborne frame -- the body keeps walking while the leg
    folds and extends, which is the nominal gait.  ``"rotation"`` keeps the
    hip still while the leg folds and while it extends, and moves it only
    during the rotation: the body **stops** for the fold and the extension
    (the planner times those frames as dwells).  A leg folding next to a
    block must not be dragged into it by the body (measured at 60 mm: every
    climb from 91 mm short of the face was RETRACT_PENETRATES_TERRAIN with
    the hip moving, and flew in place with it still).

    Day 14 additions, both defaulting to the nominal cycle's own answer:

    ``landing_surface_z_m``
        the height of the surface the leg lands on.  The extend ramp is
        required to *approach* that surface monotonically; with an obstacle in
        the scene the nearest terrain feature can change from one frame to the
        next (front face, then top), so the approach check reads the landing
        plane and not the nearest surface.  ``None`` is the ground.
    ``landing_rim``
        which rim must carry the touchdown.  The nominal cycle lands on the
        foot rim; an airborne rim swap on an obstacle top lands on the left rim
        so the trailing-edge descent can take over (Day 14 Step 4).

    Three ramps and a touchdown, in the order plan §8.2 gives them.  The
    retraction is what provides the ground clearance: at the compact posture
    the leg is a wheel of about 145 mm radius under a hip 219 mm up, so it
    lifts off by shrinking and no hip rise is needed on flat ground.  That is
    established by measuring the clearance at every frame rather than by the
    argument.

    **Where the clearance requirement applies, and where it must not.**  The
    first version held every airborne frame to ``min_clearance_m`` and failed
    on its own first cycle, because the frames either side of contact have a
    clearance of zero *by construction* -- they are the liftoff and the
    touchdown.  The requirement belongs to the phase where the leg swings past
    the ground, which is the rotation; the two ramps are leaving and
    approaching contact, and there the requirement is only that they do not
    pass **through** it.

    **The hip ramps to the touchdown stance height**, and for the full-arc
    stroke that ramp is exactly zero -- which is worth stating, because the
    obvious guess is wrong in both magnitude and sign.  The hip arches by
    17.3 mm *during* the stroke, rising to 219.4 mm over the middle of the arc,
    but the two **ends** of the arc sit at the same height (202.161 mm at
    ``alpha = -40`` and 202.161 mm at ``alpha = +40``), because the arc is
    symmetric about ``alpha = 0``.  So a full cycle needs no hip motion at all
    while airborne.

    The ramp is not dead code, though: a stroke cut short by ``max_distance_m``
    -- which is what a leg approaching a transition does -- ends part-way up
    the arch and has to come **down** to the touchdown height.  Measured at
    50 mm of stroke, that is -12.7 mm.  Both heights come from
    :meth:`NominalPosture2D.hip_z_for_flat_stance`; neither is assumed.
    """

    config = RecoveryConfig2D() if config is None else config
    posture = stroke.posture
    if not stroke.success:
        return RecoverySwing2D(
            stroke.frames[-1:], False, f"STROKE_FAILED:{stroke.stop_reason}",
            config, posture, stroke.final_scene, None,
        )

    beta_end = float(stroke.end.beta_rad)
    # The three overrides turn this into the general "swing the leg from the
    # posture it is in to the posture something else needs" primitive.  They
    # default to the nominal cycle's own answer, so every existing caller is
    # unchanged -- see ``run_posture_transition_2d``, which is the only caller
    # that passes them, and the reason they exist: a crossing does not begin
    # at the posture a nominal recovery lands in, and until this existed there
    # was no segment that got the leg from one to the other (log 1.10).
    beta_target = (recovery_beta_target_2d(stroke) if beta_target_rad is None
                   else float(beta_target_rad))
    if beta_target >= beta_end:
        return RecoverySwing2D(
            stroke.frames[-1:], False, "RECOVERY_TARGET_IS_NOT_FORWARD",
            config, posture, stroke.final_scene, None,
        )

    hip_x0 = float(stroke.end.hip_xz_m[0])
    hip_x1 = hip_x0 + config.hip_advance_m
    hip_z0 = float(stroke.end.hip_xz_m[1])
    theta_compact = float(config.theta_compact_rad)
    # Plan section 8.4: the touchdown posture is what the *next* contact needs,
    # not the compact one.  On flat ground the next stroke rolls at the nominal
    # theta, so it is read off the posture -- but it is read, not assumed equal
    # to theta_compact.
    #
    # Under ``hold_hip_z_m`` the next stroke does **not** start at the nominal
    # theta: it starts at whatever theta holds the hip at the arc's beginning
    # (measured: 72.486 deg against a nominal 60).  Landing at the nominal one
    # left the next stroke's first contact 14.4 mm away -- the boundary
    # reported it, which is what the check is for.  So the touchdown posture is
    # solved from the same constraint the stroke uses.
    theta_liftoff = float(stroke.end.theta_rad)
    if theta_touchdown_rad is not None:
        # An explicit destination posture: the caller knows what the next
        # segment starts at, which is more than this function can solve for.
        theta_touchdown = float(theta_touchdown_rad)
        hip_z1 = (float(hip_z_touchdown_m) if hip_z_touchdown_m is not None
                  else (float(posture.hold_hip_z_m)
                        if posture.hold_hip_z_m is not None
                        else posture.hip_z_for_flat_stance(beta_target)))
    elif posture.hold_hip_z_m is None:
        hip_z1 = posture.hip_z_for_flat_stance(beta_target)
        theta_touchdown = float(posture.theta_rad)
    else:
        hip_z1 = float(posture.hold_hip_z_m)
        solved = theta_for_hip_z_2d(
            posture, beta_target, hip_z1, theta_seed_rad=theta_liftoff
        )
        if solved is None:
            return RecoverySwing2D(
                stroke.frames[-1:], False, "TOUCHDOWN_LEVELLING_UNREACHABLE",
                config, posture, stroke.final_scene, None,
            )
        theta_touchdown = float(solved)
        # Same residual guard as the stroke start: land at the exact stance
        # height of the solved theta when that is higher than the held one.
        exact = float(replace(posture, theta_rad=theta_touchdown)
                      .hip_z_for_flat_stance(beta_target))
        if exact > hip_z1:
            hip_z1 = exact

    frames = [stroke.end]
    index = 1

    # The hip is part of the body, and the body keeps moving while the leg
    # retracts and extends, so the hip advances through *every* airborne
    # frame -- the frames are played uniformly in time.  Day 12 advanced it
    # over the rotation only, which put the airborne hip up to 40% of the
    # advance away from where the body's clock had it (Day 14: the two legs
    # of one axle disagreed by 11 mm in height because of it).  The joint
    # sequences are unchanged; only where the hip is drawn at each frame.
    retract_thetas = _ramp(theta_liftoff, theta_compact, config.theta_step_rad)
    lift_zs = ([] if not (lift_hip_before_rotation and hip_z1 > hip_z0)
               else _ramp(hip_z0, hip_z1, float(hip_lift_step_m))[1:])
    rotate_betas = _ramp(beta_end, beta_target, config.beta_step_rad)
    extend_thetas = _ramp(theta_compact, theta_touchdown, config.theta_step_rad)[:-1]
    airborne_count = len(retract_thetas) + len(lift_zs) + len(rotate_betas) + len(extend_thetas) + 1

    if hip_moves_during not in ("all", "rotation"):
        raise ValueError("hip_moves_during is 'all' or 'rotation'.")
    n_fold = len(retract_thetas) + len(lift_zs)
    n_rotate = len(rotate_betas)

    def hip_x_at(k: int) -> float:
        if hip_moves_during == "rotation":
            if k <= n_fold:
                return hip_x0
            if k >= n_fold + n_rotate:
                return hip_x1
            return hip_x0 + (hip_x1 - hip_x0) * ((k - n_fold) / max(1, n_rotate - 1))
        return hip_x0 + (hip_x1 - hip_x0) * (k / airborne_count)

    # Likewise the height: a swing whose landing is higher than its takeoff
    # (an axle rising with the first leg of a pair) climbs linearly through
    # every airborne frame, so the swing's hip path is one straight line in
    # (x, z) and the stance partner's levelled height follows it exactly
    # (Day 14: a rise confined to the rotation phase was 0.8 mm of height
    # per mm of travel, and a 15 mm shift of the swing between planning
    # passes left the two hips of one axle 12 mm apart).  With
    # ``lift_hip_before_rotation`` the lift phase keeps its own ramp.
    def hip_z_at(k: int, z_from: float, z_to: float) -> float:
        if lift_zs:
            return z_from if z_to == hip_z0 else z_to
        if hip_moves_during == "rotation":
            if k <= n_fold:
                return hip_z0
            if k >= n_fold + n_rotate:
                return hip_z1
            return hip_z0 + (hip_z1 - hip_z0) * ((k - n_fold) / max(1, n_rotate - 1))
        return hip_z0 + (hip_z1 - hip_z0) * (k / airborne_count)

    def step(theta: float, beta: float, hip_x: float, hip_z: float, phase: str):
        scene = posture.scene(beta, hip_x, hip_z, theta_rad=theta)
        return scene, _airborne_frame(index, phase, posture, scene)

    def fail(reason: str, scene) -> RecoverySwing2D:
        return RecoverySwing2D(
            tuple(frames), False, reason, config, posture, scene, None
        )

    # -- 1. retract to the compact posture; clearance grows from zero -------
    for theta in retract_thetas:
        scene, frame = step(theta, beta_end, hip_x_at(index),
                            hip_z0 if lift_zs else hip_z_at(index, hip_z0, hip_z0),
                            "RECOVERY_RETRACT")
        if frame.clearance_m < -posture.contact_tolerance_m or frame.collision:
            return fail("RETRACT_PENETRATES_TERRAIN", scene)
        frames.append(frame)
        index += 1

    # -- 1b. lift, when retracting is not what provides the clearance -------
    #
    # A nominal recovery gets its clearance by *shrinking*: at the compact
    # posture the leg is a 145 mm wheel under a hip 219 mm up.  A leg leaving a
    # crossing is already compact and its hip is 143.8 mm up -- the wheel is
    # resting on the ground, retracting has nothing left to retract, and the
    # rotation loses its clearance on its very first frame.  There the
    # clearance has to come from the body instead, so the hip rises first and
    # the rotation happens at the raised height.
    #
    # Off by default: the nominal cycle does not need it and every frozen
    # number was measured without it.
    hip_z_rotate = hip_z0
    if lift_zs:
        for hip_z in lift_zs:
            scene, frame = step(theta_compact, beta_end, hip_x_at(index), hip_z,
                                "RECOVERY_LIFT")
            if frame.collision:
                return fail("LIFT_PENETRATES_TERRAIN", scene)
            frames.append(frame)
            index += 1
        hip_z_rotate = hip_z1

    # -- 2. carry the rotation on, in the same forward sense ----------------
    span = beta_end - beta_target
    for beta in rotate_betas:
        fraction = 0.0 if span == 0.0 else (beta_end - beta) / span
        scene, frame = step(
            theta_compact, beta,
            hip_x_at(index),
            (hip_z_rotate + fraction * (hip_z1 - hip_z_rotate)) if lift_zs
            else hip_z_at(index, hip_z0, hip_z1),
            "RECOVERY_ROTATE",
        )
        if frame.clearance_m < config.min_clearance_m or frame.collision:
            return fail("ROTATION_CLEARANCE_LOST", scene)
        frames.append(frame)
        index += 1

    # -- 3. extend into what the next touchdown needs; clearance falls to zero
    landing_plane = (float(posture.ground_height_m) if landing_surface_z_m is None
                     else float(landing_surface_z_m))
    previous = _clearance_m(scene, landing_plane)
    for theta in extend_thetas:
        scene, frame = step(theta, beta_target, hip_x_at(index),
                            hip_z1 if lift_zs else hip_z_at(index, hip_z1, hip_z1),
                            "RECOVERY_EXTEND")
        if frame.clearance_m < -posture.contact_tolerance_m or frame.collision:
            return fail("EXTEND_PENETRATES_TERRAIN_BEFORE_TOUCHDOWN", scene)
        # The approach is judged against the landing plane: with an obstacle
        # present the nearest surface can change mid-ramp, and that is not
        # the leg reaching away from where it is going to land.
        to_plane = _clearance_m(scene, landing_plane)
        if to_plane > previous + posture.contact_tolerance_m:
            return fail("EXTEND_DOES_NOT_APPROACH_THE_GROUND", scene)
        previous = to_plane
        frames.append(frame)
        index += 1

    # -- 4. touchdown -------------------------------------------------------
    scene = posture.scene(beta_target, hip_x1, hip_z1,
                          theta_rad=theta_touchdown)
    sample = _lowest_contact_sample(scene.geometry)
    result = posture.query(scene)
    # A nominal swing lands on the ground; a B5 swing lands on the obstacle
    # top.  Both are "the surface the leg stands on next" -- see
    # :func:`_standable_surface_ids`.
    candidate = _candidate_for_sample(
        result, sample, surface_ids=_standable_surface_ids(scene)
    )
    if result.collision or candidate is None:
        return fail("TOUCHDOWN_IS_NOT_A_VALID_GROUND_CONTACT", scene)
    if RimId(candidate.rim) is not RimId(landing_rim):
        return fail(
            f"TOUCHDOWN_ON_{RimId(candidate.rim).value.upper()}_NOT_"
            f"{RimId(landing_rim).value.upper()}_RIM", scene
        )
    frames.append(_stance_frame(index, "RECOVERY_TOUCHDOWN", posture, scene, sample))
    return RecoverySwing2D(tuple(frames), True, None, config, posture, scene, sample)


def standing_pose_penetration_m(
    posture: NominalPosture2D,
    theta_rad: float,
    beta_rad: float,
    hip_x_m: float,
    hip_z_m: float,
) -> float:
    """How far below the ground this pose reaches; ``0.0`` when it is clear.

    A pose handed over from another day's generator can miss Day 12's ground by
    a rounding edge -- the crossing's own exit pose misses it by **0.0001 mm**
    -- and that is enough for the contact query to call it a collision.  This
    is the number to lift by, measured rather than guessed at with a grid.
    """

    scene = posture.scene(float(beta_rad), float(hip_x_m), float(hip_z_m),
                          theta_rad=float(theta_rad))
    return float(max(0.0, -_clearance_m(scene, posture.ground_height_m)))


def standing_stroke_2d(
    posture: NominalPosture2D,
    theta_rad: float,
    beta_rad: float,
    hip_x_m: float,
    hip_z_m: float,
) -> RollStroke2D:
    """A one-frame "stroke" for a leg standing at a given pose.

    :func:`run_recovery_swing_2d` starts from wherever a stroke ended, which is
    all a nominal cycle ever needs.  A **crossing** ends somewhere else -- on a
    different rim, at a different ``theta``, with the hip 76 mm lower -- and the
    leg still has to get back to the nominal posture from there (log 1.10).
    This is how that end state is handed to the same generator, rather than by
    writing a second swing generator that would get the three ramps' clearance
    rules wrong in a fresh way.

    The pose must be a real ground contact; a pose that is airborne or in
    collision comes back as a failed stroke rather than as a frame that claims
    a contact it does not have.
    """

    scene = posture.scene(float(beta_rad), float(hip_x_m), float(hip_z_m),
                          theta_rad=float(theta_rad))
    sample = _lowest_contact_sample(scene.geometry)
    result = posture.query(scene)
    candidate = _candidate_for_sample(
        result, sample, surface_ids=_standable_surface_ids(scene)
    )
    if result.collision or candidate is None:
        return RollStroke2D(
            (), False, "STANDING_POSE_IS_NOT_A_GROUND_CONTACT",
            posture, scene, sample,
        )
    frame = _stance_frame(0, "STANDING", posture, scene, sample)
    return RollStroke2D((frame,), True, "STANDING", posture, scene, sample)


def _ramp(start: float, stop: float, step: float) -> list[float]:
    """Inclusive values from ``start`` to ``stop``, stepping by at most ``step``.

    The final value is always exactly ``stop``: a ramp that stopped one
    truncated step short would leave the pose slightly off the thing the next
    phase assumes it reached.
    """

    span = float(stop) - float(start)
    if abs(span) <= 1e-12:
        return [float(stop)]
    count = int(np.ceil(abs(span) / float(step)))
    return [float(v) for v in np.linspace(float(start), float(stop), count + 1)]


# --------------------------------------------------------------------------
# The cycle
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class NominalCycle2D:
    """``FOOT_RIM_ROLL + RECOVERY_SWING`` -- plan §0.2's nominal cycle."""

    stroke: RollStroke2D
    recovery: RecoverySwing2D

    @property
    def success(self) -> bool:
        return self.stroke.success and self.recovery.success

    @property
    def frames(self) -> tuple[CycleFrame2D, ...]:
        # The recovery's first frame *is* the stroke's last, re-labelled; it is
        # a hand-over, not a new pose, so it is not counted twice.
        return self.stroke.frames + self.recovery.frames[1:]

    @property
    def hip_advance_m(self) -> float:
        return float(self.frames[-1].hip_xz_m[0] - self.frames[0].hip_xz_m[0])

    @property
    def total_rotation_rad(self) -> float:
        return float(self.frames[0].beta_rad - self.frames[-1].beta_rad)


def _translate_frame(frame: CycleFrame2D, dx_m: float,
                     dbeta_rad: float) -> CycleFrame2D:
    return replace(
        frame,
        beta_rad=float(frame.beta_rad) + dbeta_rad,
        hip_xz_m=(float(frame.hip_xz_m[0]) + dx_m, float(frame.hip_xz_m[1])),
        contact_xz_m=(None if frame.contact_xz_m is None else
                      (float(frame.contact_xz_m[0]) + dx_m,
                       float(frame.contact_xz_m[1]))),
    )


@lru_cache(maxsize=8)
def nominal_stroke_2d(posture: NominalPosture2D | None = None) -> RollStroke2D:
    """The full rolling stroke from the arc's own start -- cached.

    Callers ask for this constantly and only to read one number off it: the
    contact lead, the cycle advance, the hip:contact ratio, the pose a nominal
    run begins in.  Regenerating it each time dominated everything else: a
    profile of one leg's world-registered chain spent **170 s of 181 s** inside
    ``run_foot_rim_roll_2d``, across five calls, of which two were these probes.

    Only the no-argument stroke is cached, because only that one is the same
    every time.  Anything with a ``start_beta_rad``, a ``hip_x_m`` or a
    ``max_distance_m`` goes to :func:`run_foot_rim_roll_2d` as before.
    """

    return run_foot_rim_roll_2d(NominalPosture2D() if posture is None
                                else posture)


def translate_cycle_2d(cycle: NominalCycle2D, dx_m: float,
                       dbeta_rad: float) -> NominalCycle2D:
    """The same cycle, moved along the ground and on round the beta counter.

    Every nominal cycle on flat ground **is** the first one translated: the
    geometry is periodic in ``2*pi`` and invariant in ``x``, and the carry rule
    starts each cycle exactly one period on.  Measured, not assumed: cycles 1
    and 2 of a three-cycle run reproduce cycle 0 shifted by
    (325.9158 mm, -360 deg) and (651.8315 mm, -720 deg) with a residual of
    **0.0000 um and 0.0000 udeg** -- bit-identical.  ``test_a_translated_cycle
    _matches_a_generated_one`` holds that, and it is what lets a run of N
    cycles cost one.

    ``final_scene`` is rebuilt at the translated pose rather than carried, so
    no field of the result quietly describes the untranslated one.
    """

    def move(frames):
        return tuple(_translate_frame(f, dx_m, dbeta_rad) for f in frames)

    stroke, recovery = cycle.stroke, cycle.recovery
    moved_stroke_frames = move(stroke.frames)
    moved_recovery_frames = move(recovery.frames)
    end = moved_recovery_frames[-1]
    scene = recovery.posture.scene(
        float(end.beta_rad), float(end.hip_xz_m[0]), float(end.hip_xz_m[1]),
        theta_rad=float(end.theta_rad))
    stroke_end = moved_stroke_frames[-1]
    stroke_scene = stroke.posture.scene(
        float(stroke_end.beta_rad), float(stroke_end.hip_xz_m[0]),
        float(stroke_end.hip_xz_m[1]), theta_rad=float(stroke_end.theta_rad))
    return NominalCycle2D(
        replace(stroke, frames=moved_stroke_frames, final_scene=stroke_scene),
        replace(recovery, frames=moved_recovery_frames, final_scene=scene),
    )


def run_nominal_cycles_2d(
    cycles: int = 2,
    posture: NominalPosture2D | None = None,
    config: RecoveryConfig2D | None = None,
    *,
    hip_x_m: float = 0.0,
    start_beta_rad: float | None = None,
) -> list[NominalCycle2D]:
    """Chain ``cycles`` nominal cycles, each starting where the last landed.

    ``start_beta_rad`` continues an existing revolution counter.  ``None``
    starts at the arc's own beginning, which is right for a run that begins the
    world but wrong for one that resumes after a crossing: leaving it to
    default there restarts the counter and the seam comes out a whole number
    of turns wide -- 6*pi for a leg that had rolled one cycle before the
    obstacle, 8*pi for one that had rolled three (log 1.11).
    """

    if cycles < 1:
        raise ValueError("ask for at least one cycle.")
    posture = NominalPosture2D() if posture is None else posture
    config = RecoveryConfig2D() if config is None else config

    beta = None if start_beta_rad is None else float(start_beta_rad)
    hip_x = float(hip_x_m)
    first_stroke = run_foot_rim_roll_2d(posture, start_beta_rad=beta,
                                        hip_x_m=hip_x)
    first = NominalCycle2D(first_stroke,
                           run_recovery_swing_2d(first_stroke, config))
    out: list[NominalCycle2D] = [first]
    if not first.recovery.success or cycles == 1:
        return out

    # Every later cycle is this one translated -- see ``translate_cycle_2d``.
    # The step is read off the cycle itself rather than written down, so a
    # different ``config`` (a non-zero ``hip_advance_m``, say) still gets its
    # own correct period instead of a constant that was right once.
    #
    # The next stroke starts from the pose the recovery landed in.  Beta is
    # carried forward unwrapped so the rotation keeps accumulating in one
    # direction; the geometry is periodic in 2*pi, the bookkeeping is not.
    dx = float(first.recovery.end.hip_xz_m[0]
               - first.stroke.start.hip_xz_m[0])
    dbeta = float(first.recovery.end.beta_rad - first.stroke.start.beta_rad)
    for index in range(1, cycles):
        out.append(translate_cycle_2d(first, dx * index, dbeta * index))
    return out


# --------------------------------------------------------------------------
# Into the Day 12 segment contract
# --------------------------------------------------------------------------


def _point(frame: CycleFrame2D) -> PointContact2D:
    return PointContact2D(
        rim=frame.rim, alpha_rad=frame.alpha_rad,
        point_world_xz_m=frame.contact_xz_m, surface_id=frame.surface_id,
        theta_rad=frame.theta_rad, beta_rad=frame.beta_rad,
        hip_xz_m=frame.hip_xz_m,
    )


def roll_segment_2d(
    stroke: RollStroke2D, *, source_id: str, frame_offset: int = 0,
    kind: SegmentKind = SegmentKind.FOOT_RIM_ROLL,
    phase_label: str = "NOMINAL_FOOT_RIM_ROLL",
) -> MotionSegment2D:
    """One rolling stroke as a chainable segment.

    Extracted from :func:`cycle_segments_2d` rather than copied: a partial
    approach stroke (log 1.11) needs exactly this and a second copy of the
    ``BodyRequirement2D`` / ``RollingContact2D`` construction would drift.
    """

    n_roll = len(stroke.frames)
    roll_hip_z = np.array([f.hip_xz_m[1] for f in stroke.frames], dtype=float)
    roll_x = [f.hip_xz_m[0] for f in stroke.frames]
    return MotionSegment2D(
        kind=kind,
        phase_label=phase_label,
        start_contact=_point(stroke.start),
        end_contact=_point(stroke.end),
        sampling=RollSampling2D(
            arc_samples=stroke.posture.arc_samples,
            beta_step_rad=-abs(stroke.rotation_rad) / max(1, n_roll - 1),
            theta_step_rad=None,
        ),
        # What this stroke asks of the body -- and the answer depends on
        # whether the stroke was *told* a height or *found* one.
        #
        # Rolling at a fixed theta, the hip height is an output of the contact
        # geometry: the hip rides the rim arc and the leg has no say, so the
        # body has to track it.  That is the reading Day 10--11 gives every
        # rolling segment, and it is right for that case.
        #
        # Rolling at a *held* height, it is an input.  The leg was handed a
        # height and spent theta to stay on it, so it is not asking the body
        # for anything -- it is already following.  Leaving it as TRACK is what
        # made Step 5 refuse the crossing: the leg on the obstacle demands
        # 143.8-198.0 mm, three legs on the ground demand 219.4 mm, both sides
        # hard, 109 instants of INFEASIBLE (log 1.20).  Three of those four
        # were never really demanding anything.
        body_requirement=BodyRequirement2D(
            kind=(BodyRequirementKind.NONE if stroke.posture.holds_hip_z
                  else BodyRequirementKind.TRACK),
            x_range_m=(float(min(roll_x)), float(max(roll_x))),
            hip_z_profile_m=(None if stroke.posture.holds_hip_z
                             else roll_hip_z),
        ),
        frames=FrameRef2D(
            source_id=source_id,
            indices=tuple(range(frame_offset, frame_offset + n_roll)),
        ),
        rolling=RollingContact2D(
            rim=RimId.FOOT,
            surface_ids=(stroke.start.surface_id,),
            alpha_range_rad=stroke.alpha_range_rad,
            beta_range_rad=(float(stroke.end.beta_rad),
                            float(stroke.start.beta_rad)),
            theta_range_rad=(stroke.posture.theta_rad, stroke.posture.theta_rad),
            contact_start_xz_m=stroke.start.contact_xz_m,
            contact_end_xz_m=stroke.end.contact_xz_m,
        ),
    )


def swing_segment_2d(
    recovery: RecoverySwing2D, *, source_id: str, frame_offset: int = 0,
    kind: SegmentKind = SegmentKind.RECOVERY_SWING,
    phase_label: str = "NOMINAL_RECOVERY_SWING",
) -> MotionSegment2D:
    """One airborne swing as a chainable segment.

    ``kind`` and ``phase_label`` are arguments because the *same* motion serves
    the nominal recovery and the two crossing transitions (log 1.11); what
    differs is what the chain calls it, not how it is built.
    """

    n_rec = len(recovery.frames)
    rec_x = [f.hip_xz_m[0] for f in recovery.frames]
    return MotionSegment2D(
        kind=kind,
        phase_label=phase_label,
        start_contact=_point(recovery.start),
        end_contact=_point(recovery.end),
        sampling=RollSampling2D(
            arc_samples=recovery.posture.arc_samples,
            beta_step_rad=-abs(recovery.config.beta_step_rad),
            theta_step_rad=abs(recovery.config.theta_step_rad),
        ),
        body_requirement=BodyRequirement2D(
            # Airborne, so the body is not being asked to track anything: the
            # hip only has to stay high enough for the leg to clear.  That is a
            # lower bound, which is the same reading Day 10--11 gives a swing
            # interior.
            kind=BodyRequirementKind.LOWER_BOUND,
            x_range_m=(float(min(rec_x)), float(max(rec_x))),
            hip_z_min_m=float(max(f.hip_xz_m[1] for f in recovery.frames)),
        ),
        frames=FrameRef2D(
            source_id=source_id,
            indices=tuple(range(frame_offset, frame_offset + n_rec)),
        ),
        recovery_shaping=RecoveryShaping2D(
            theta_compact_rad=recovery.config.theta_compact_rad,
            theta_touchdown_rad=float(recovery.end.theta_rad),
            airborne_rotation_rad=recovery.rotation_rad,
            min_clearance_m=recovery.min_clearance_m,
            hip_advance_m=recovery.config.hip_advance_m,
        ),
    )


def cycle_segments_2d(
    cycle: NominalCycle2D, *, source_id: str, frame_offset: int = 0,
) -> tuple[MotionSegment2D, MotionSegment2D]:
    """The cycle as two chainable segments.

    ``duration_s`` stays ``None`` on both.  The generator advances in beta, not
    in time, exactly as Day 6--7's traversal does (trap 33), so assigning a
    duration here would be Step 3's modelling decision made early and in the
    wrong place.
    """

    if not cycle.success:
        raise ValueError(
            "refusing to build segments from a failed cycle: "
            f"stroke={cycle.stroke.stop_reason}, "
            f"recovery={cycle.recovery.failure_reason}"
        )
    n_roll = len(cycle.stroke.frames)
    return (
        roll_segment_2d(cycle.stroke, source_id=source_id,
                        frame_offset=frame_offset),
        swing_segment_2d(cycle.recovery, source_id=source_id,
                         frame_offset=frame_offset + n_roll),
    )


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------


def cycle_frame_rows(cycles: Sequence[NominalCycle2D]) -> list[dict]:
    rows = []
    for i, cycle in enumerate(cycles):
        for frame in cycle.frames:
            rows.append({"cycle": i, **frame.as_dict()})
    return rows


def cycle_summary_rows(cycles: Sequence[NominalCycle2D]) -> list[dict]:
    rows = []
    for i, cycle in enumerate(cycles):
        stroke, recovery = cycle.stroke, cycle.recovery
        rows.append({
            "cycle": i,
            "success": cycle.success,
            "stroke_stop_reason": stroke.stop_reason,
            "recovery_failure_reason": recovery.failure_reason,
            "stroke_frames": len(stroke.frames),
            "recovery_frames": len(recovery.frames),
            "alpha_from_deg": float(np.rad2deg(stroke.alpha_range_rad[0])),
            "alpha_to_deg": float(np.rad2deg(stroke.alpha_range_rad[1])),
            "stroke_contact_advance_mm": stroke.contact_advance_m * 1e3,
            "stroke_hip_advance_mm": stroke.hip_advance_m * 1e3,
            "stroke_hip_z_travel_mm": stroke.hip_z_travel_m * 1e3,
            "stroke_rotation_deg": float(np.rad2deg(stroke.rotation_rad)),
            "recovery_rotation_deg": float(np.rad2deg(recovery.rotation_rad)),
            "cycle_rotation_deg": float(np.rad2deg(cycle.total_rotation_rad)),
            "recovery_theta_min_deg": float(np.rad2deg(recovery.theta_min_rad)),
            "recovery_rotation_min_clearance_mm": recovery.min_clearance_m * 1e3,
            "recovery_ramp_min_clearance_mm": recovery.ramp_min_clearance_m * 1e3,
            "cycle_hip_advance_mm": cycle.hip_advance_m * 1e3,
        })
    return rows
