"""B5: climb onto an obstacle with the **nominal-locomotion swing**.

The project owner's instruction (Day 13 log 34.1):

    "I want to change to the nominal locomotion swing, the flat-ground one --
     but Day 8-9 is not thrown away, because you still need to know the start
     and end positions."

That is exactly the split this module implements.  Day 8--9 (via the Day 10--11
composer) still says **where** the ascent begins and ends: the approach pose on
the ground and the landing pose on the top, both built by
:func:`standing_scene_2d` so the two ends stay comparable with every other
segment.  What changes is **how** the leg gets between them: instead of
:func:`generate_swing_2d`'s Cartesian lift-carry-place, the leg is flown by
:func:`run_recovery_swing_2d` -- retract to the compact posture, carry the same
forward rotation on, extend into the landing.

**Why the owner wants this.**  The Cartesian swing goes theta 60 -> 54.2 ->
41.0 -> 36.6 -> 48.1 -> 58.7: it shrinks and then reaches out, so the leg is at
its longest just as it crosses the obstacle's leading edge.  The nominal swing
gets its clearance by shrinking to theta 17 and keeping it there through the
rotation, which is the posture least likely to catch the front face.

**What had to be true for this to work at all**, both measured, not assumed:

* ``run_recovery_swing_2d`` reads its posture from ``stroke.posture``, not from
  a parameter.  The obstacle therefore has to be in the **stroke's** posture --
  putting it only on the swing's is what made three earlier probes report
  ``TOUCHDOWN_IS_NOT_A_VALID_GROUND_CONTACT`` and read like a refusal of the
  landing when the landing had never been asked for (log 34.5).
* Stance frames had to stop insisting the support surface is the ground.  A leg
  that has landed on the top is standing on the top (log 34.6).

**The touchdown pair is not over-specified.**  Passing both
``theta_touchdown_rad`` and ``hip_z_touchdown_m`` looks like the trap the notes
warn about, but here the two are consistent *by construction*: raising the
support surface and the hip by the same height leaves the leg's geometry
identical, so the landing theta is the nominal theta and the landing hip is the
flat-stance hip plus the obstacle height.  Measured: the landing lands at
(430.7, 40.0) with the top at 40.0.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

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
    standing_scene_2d,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
    NominalPosture2D,
    RecoveryConfig2D,
    RecoverySwing2D,
    run_recovery_swing_2d,
    standing_stroke_2d,
)


@dataclass(frozen=True)
class NominalAscent2D:
    """The outcome of flying one ascent with the nominal swing."""

    #: ``None`` when the ascent was refused; the reason is in :attr:`refusal`.
    swing: RecoverySwing2D | None
    refusal: str | None
    #: Where the leg started and where it landed, in world metres.  Recorded
    #: even on a refusal, because "it refused" is not a measurement without
    #: the question it refused.
    start_contact_xz_m: tuple[float, float]
    landing_contact_xz_m: tuple[float, float] | None
    #: The obstacle this ascent was asked to climb, ``(x_start, width, height)``.
    obstacle_xwh_m: tuple[float, float, float]

    @property
    def success(self) -> bool:
        return self.swing is not None

    def as_dict(self) -> dict:
        landing = self.landing_contact_xz_m
        return {
            "success": self.success,
            "refusal": self.refusal,
            "obstacle_height_mm": self.obstacle_xwh_m[2] * 1e3,
            "obstacle_x_start_mm": self.obstacle_xwh_m[0] * 1e3,
            "start_contact_x_mm": self.start_contact_xz_m[0] * 1e3,
            "landing_contact_x_mm": None if landing is None else landing[0] * 1e3,
            "landing_contact_z_mm": None if landing is None else landing[1] * 1e3,
            "frames": None if self.swing is None else len(self.swing.frames),
        }


def obstacle_posture_2d(
    posture: NominalPosture2D, spec: SharedTerrainSpec2D
) -> NominalPosture2D:
    """The nominal posture, with ``spec``'s obstacle put into its scenes.

    ``NominalPosture2D`` builds flat-ground scenes by default, and it must keep
    doing so: Step 1's nominal cycle would be a different measurement if a
    terrain feature leaked into it.  This is the explicit opt-in for the one
    caller that needs the obstacle present -- an ascent cannot land on a top
    that is not in its own scene.
    """

    return replace(
        posture,
        obstacle_xwh_m=(
            float(spec.x_start_m), float(spec.top_length_m), float(spec.height_m)
        ),
    )


def _stance_hip_z_m(posture: NominalPosture2D, beta_rad: float) -> float:
    """The hip height this posture actually stands at, on flat ground.

    ``hip_z_for_flat_stance`` answers for a *fixed*-theta posture, whose hip
    rides the rim arc up and down.  The Hybrid posture holds the hip level by
    modulating theta instead, and stands 17.2 mm higher than that (219.4 mm
    against 202.2 mm).  Asking the wrong one of the two puts the approach pose
    below the ground.
    """

    if posture.hold_hip_z_m is not None:
        return float(posture.hold_hip_z_m)
    return float(posture.hip_z_for_flat_stance(float(beta_rad)))


def run_nominal_ascent_2d(
    spec: SharedTerrainSpec2D,
    posture: NominalPosture2D,
    config: RecoveryConfig2D,
    *,
    approach_hip_x_m: float,
    landing_hip_x_m: float,
    beta_takeoff_rad: float,
    beta_landing_rad: float,
    theta_takeoff_rad: float | None = None,
) -> NominalAscent2D:
    """Fly the ascent from the ground pose to the top pose, nominal-style.

    ``approach_hip_x_m`` and ``landing_hip_x_m`` are the Day 8--9 / Day 10--11
    endpoints -- this function does not choose them, which is the whole point
    of keeping those days.  Everything between them is the nominal swing's.
    """

    obstacle = (
        float(spec.x_start_m), float(spec.top_length_m), float(spec.height_m)
    )
    post = obstacle_posture_2d(posture, spec)
    theta_takeoff = (
        float(posture.theta_rad) if theta_takeoff_rad is None
        else float(theta_takeoff_rad)
    )

    # The obstacle enters through the STROKE, because run_recovery_swing_2d
    # takes ``posture = stroke.posture``.  This line is the fix for log 34.5.
    #
    # The approach height is the posture's OWN stance height, not
    # ``hip_z_for_flat_stance``.  Under the Hybrid posture those differ by
    # 17.2 mm (219.4 held against 202.2 flat-stance), because ``hold_hip_z_m``
    # modulates theta to keep the hip level instead of letting it follow the
    # rim arc.  Standing the approach at the flat-stance height put the leg
    # 17 mm into the ground and the whole ascent read as
    # ``APPROACH_POSE_REFUSED`` at a height and placement that had already
    # been measured to work.
    ground_hip_z = _stance_hip_z_m(post, float(beta_takeoff_rad))
    stroke = standing_stroke_2d(
        post, theta_takeoff, float(beta_takeoff_rad),
        float(approach_hip_x_m), ground_hip_z,
    )
    start_xz = (
        (float("nan"), float("nan")) if not stroke.frames
        else tuple(float(v) for v in stroke.frames[-1].contact_xz_m)
    )
    if not stroke.success:
        return NominalAscent2D(
            None, f"APPROACH_POSE_REFUSED:{stroke.stop_reason}",
            start_xz, None, obstacle,
        )

    # The landing pair: same theta as a flat stance, hip raised by exactly the
    # obstacle height.  Consistent by construction -- see the module docstring.
    #
    # The landing height must be consistent with the landing THETA, and the
    # landing theta is ``posture.theta_rad`` -- the fixed-theta stance the next
    # segment starts from.  So this one is ``hip_z_for_flat_stance``, not the
    # held height: pairing the held height (219.4) with theta 60 asks the leg
    # to reach 17.2 mm further than that theta can, and the landing misses the
    # top.  The approach and the landing legitimately use different heights
    # because they are different postures -- that is not an inconsistency, it
    # is the swing's whole job.
    landing_hip_z = (
        post.hip_z_for_flat_stance(float(beta_landing_rad)) + float(spec.height_m)
    )
    swing = run_recovery_swing_2d(
        stroke,
        replace(config, hip_advance_m=float(landing_hip_x_m) - float(approach_hip_x_m)),
        beta_target_rad=float(beta_landing_rad),
        theta_touchdown_rad=float(posture.theta_rad),
        hip_z_touchdown_m=landing_hip_z,
    )
    if not swing.success:
        return NominalAscent2D(
            None, str(swing.failure_reason), start_xz, None, obstacle
        )
    landing = tuple(float(v) for v in swing.frames[-1].contact_xz_m)
    return NominalAscent2D(swing, None, start_xz, landing, obstacle)


def segment_from_nominal_ascent_2d(
    ascent: NominalAscent2D,
    *,
    source_id: str,
    duration_s: float,
    arc_samples: int,
    max_joint_step_rad: float,
) -> MotionSegment2D:
    """Write a nominal ascent into the Day 10--11 schema as a ``SWING_UP``.

    It is a ``SWING_UP`` because that is what it *is* -- an airborne segment
    that ends on the top -- and every consumer downstream (the transition
    mapping, the segment contract, the exporter) already knows that kind.  What
    changed is the generator behind it, not the segment's role.

    **The shaping knobs are recorded as zero, and that means "not used".**
    ``apex_clearance_m`` / ``liftoff_rise_m`` / ``touchdown_drop_m`` are
    Cartesian-planner parameters: they shape a hip trajectory that lifts the
    foot over the obstacle.  A nominal swing has no such trajectory -- its
    clearance comes from retracting to ``theta_compact_rad`` and it reaches its
    height by rotating, not by rising.  Writing a plausible-looking number here
    would claim a knob was tuned when no such knob exists, which is exactly the
    kind of fabricated quantity the Day 13 notes warn about.  The real
    clearance is measured per frame and lives in the frames themselves.
    """

    if ascent.swing is None:
        raise ValueError(
            f"a refused ascent has no segment to write: {ascent.refusal}"
        )
    frames = ascent.swing.frames
    start, end = frames[0], frames[-1]
    hip = np.array([f.hip_xz_m for f in frames], dtype=float)

    return MotionSegment2D(
        kind=SegmentKind.SWING_UP,
        phase_label=SegmentKind.SWING_UP.value,
        start_contact=PointContact2D(
            rim=start.rim,
            alpha_rad=float(start.alpha_rad),
            point_world_xz_m=np.asarray(start.contact_xz_m, dtype=float),
            surface_id=str(start.surface_id),
            theta_rad=float(start.theta_rad),
            beta_rad=float(start.beta_rad),
            hip_xz_m=np.asarray(start.hip_xz_m, dtype=float),
        ),
        end_contact=PointContact2D(
            rim=end.rim,
            alpha_rad=float(end.alpha_rad),
            point_world_xz_m=np.asarray(end.contact_xz_m, dtype=float),
            surface_id=str(end.surface_id),
            theta_rad=float(end.theta_rad),
            beta_rad=float(end.beta_rad),
            hip_xz_m=np.asarray(end.hip_xz_m, dtype=float),
        ),
        sampling=SwingSampling2D(
            arc_samples=int(arc_samples),
            sample_count=len(frames),
            leg_arc_samples=int(arc_samples),
            max_joint_step_rad=float(max_joint_step_rad),
        ),
        # Same LOWER_BOUND meaning as a Cartesian swing: the hip must be at
        # least as high as this segment takes it.
        body_requirement=BodyRequirement2D(
            kind=BodyRequirementKind.LOWER_BOUND,
            x_range_m=(float(hip[:, 0].min()), float(hip[:, 0].max())),
            hip_z_min_m=float(hip[:, 1].max()),
        ),
        frames=FrameRef2D(
            source_id=source_id,
            indices=tuple(int(f.index) for f in frames),
        ),
        duration_s=float(duration_s),
        swing_shaping=SwingShaping2D(
            apex_clearance_m=0.0,
            liftoff_rise_m=0.0,
            touchdown_drop_m=0.0,
            duration_scale=1.0,
            mid_fractions=(0.35, 0.65),
        ),
    )
