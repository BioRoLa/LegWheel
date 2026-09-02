"""Day 10--11 Step 7: compose one full sequence per surviving strategy.

Step 7's task list was written before Steps 2b, 3 and 5 ran, and it names three
pairs to compose: ``ROLL+ROLL``, ``ROLL+SWING`` and ``SWING+ROLL``.  **Two of
those three no longer exist.**

    #2 ROLL_UP  + SWING_DOWN   refuted by Step 3 D/E
    #3 SWING_UP + ROLL_DOWN    refuted by Step 2b

So the honest reading of "compose three" is: compose the three that survive
(``#1``, ``#4``, ``#5``) and produce the two refusals **as first-class
outputs** rather than skipping them.  :func:`compose_2d` does exactly that --
a refuted strategy returns a :class:`ComposedSequence2D` with no sequence and
the reason attached, so the composer's output covers all five.

**Cells are chosen by Step 5's rule, not by hand.**  Each strategy is composed
at a cell where ``decide_2d`` actually picks it, so the sequences are evidence
for the decision map rather than a separate demonstration.

**Two checks the spec asks for beyond frame continuity.**

``seam margin``
    How far a hand-over sits from the ``+-40`` and ``+-180 deg`` rim seams
    (spec 6.3).  A hand-over near a seam is one where the contact-point
    reported by the query can jump.

``rim geometry gap``
    ``LegModel.rim_point`` uses 0.145 m; the drawn arc the contact pipeline
    uses is 0.1438 m on the upper tyres.  Step 0 measured that 1.2 mm; this
    step checks it does not **accumulate** across hand-overs.
"""

from __future__ import annotations

import time
from collections.abc import Sequence
from dataclasses import dataclass, replace

import numpy as np

from legwheel.planners.hybrid import HipPose2D, RimId
from hybrid_note.scripts.experiments.cartesian_swing_contract_2d import (
    HipTrajectory2D,
    build_swing_request_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_ik_2d import (
    rim_point_model_gap_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_planner_2d import (
    generate_swing_2d,
)
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (
    BLOCKED_PAIRS,
    REFUTATIONS,
    Availability,
    Verdict,
    DecisionTables2D,
    StrategyId,
    decide_2d,
)
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    MotionSequence2D,
    PointContact2D,
    SegmentKind,
    TransitionKind,
    TransitionRequirement2D,
)
from hybrid_note.scripts.experiments.day10_11_sequence_builders_2d import (
    HandoffReport2D,
    handoff_report_2d,
    segment_from_swing_plan_2d,
    sequence_from_traversal_frames_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
    approach_hip_x_for_clearance_2d,
    rolling_inputs_2d,
    standing_scene_2d,
)
from hybrid_note.scripts.experiments.right_up_left_down_full_traversal_2d import (
    check_right_up_left_down_traversal,
)
from hybrid_note.scripts.experiments.right_up_left_down_sweep_2d import (
    SweepSettings2D,
)

#: The rim boundaries.  ``+-40 deg`` separates the foot rim from the two upper
#: tyres; ``+-180 deg`` is where the two upper tyres meet behind the leg.
RIM_SEAMS_DEG: tuple[float, ...] = (-180.0, -40.0, 40.0, 180.0)

#: Step 0's measurement, kept as the value this step checks against.
NOMINAL_RIM_GAP_M = 1.2e-3

#: Step 2 / Step 3's grid values, so a composed swing is the one the sweeps
#: priced rather than a fresh guess.
#: Where the obstacle sits in the frame every composed crossing is generated
#: in.  The composed segments carry **absolute** contacts in that frame, so
#: anything that has to place a crossing in a world -- Day 12 appendix B
#: registers one against the terrain -- needs this number rather than a second
#: copy of it.
COMPOSER_FRAME_X_START_M = 0.10

SWING_LANDING_DISTANCE_M = 0.16
SWING_APEX_CLEARANCE_M = 0.03
SWING_DURATION_S = 0.6
SWING_SAMPLE_COUNT = 31
SWING_ARC_SAMPLES = 121
SWING_COLLISION_ARC_SAMPLES = 61


# --------------------------------------------------------------------------
# The extra hand-over checks
# --------------------------------------------------------------------------


def seam_margin_deg(alpha_rad: float) -> float:
    """Degrees from ``alpha`` to the nearest rim seam.  Spec 6.3."""

    alpha_deg = float(np.rad2deg(alpha_rad))
    return float(min(abs(alpha_deg - seam) for seam in RIM_SEAMS_DEG))


@dataclass(frozen=True)
class SequenceHandoff2D:
    """One hand-over, with the two checks spec 6.3 and Step 0 ask for."""

    base: HandoffReport2D
    seam_margin_before_deg: float
    seam_margin_after_deg: float
    rim_gap_before_m: float
    rim_gap_after_m: float

    @property
    def rim_gap_change_m(self) -> float:
        """How the gap moves across this hand-over.

        A change of exactly the nominal 1.2 mm is **expected** wherever the
        contact moves between the foot rim (gap 0) and an upper tyre (gap
        1.2 mm).  It is not evidence of accumulation; the evidence for that
        would be a *value* above the nominal, which is what
        :attr:`largest_rim_gap_m` reports.
        """

        return float(self.rim_gap_after_m - self.rim_gap_before_m)

    @property
    def largest_rim_gap_m(self) -> float:
        """The check that actually answers whether the gap accumulates."""

        return float(max(self.rim_gap_before_m, self.rim_gap_after_m))

    def as_dict(self) -> dict:
        row = self.base.as_dict()
        row.update({
            "seam_margin_before_deg": self.seam_margin_before_deg,
            "seam_margin_after_deg": self.seam_margin_after_deg,
            "rim_gap_before_mm": self.rim_gap_before_m * 1e3,
            "rim_gap_after_mm": self.rim_gap_after_m * 1e3,
            "rim_gap_change_mm": self.rim_gap_change_m * 1e3,
            "largest_rim_gap_mm": self.largest_rim_gap_m * 1e3,
        })
        return row


def _rim_gap(contact: PointContact2D) -> float:
    return float(rim_point_model_gap_2d(
        contact.theta_rad, contact.beta_rad, RimId(contact.rim), contact.alpha_rad
    ))


def sequence_handoffs_2d(sequence: MotionSequence2D) -> list[SequenceHandoff2D]:
    out = []
    for base, (before, after) in zip(
        handoff_report_2d(sequence),
        zip(sequence.segments, sequence.segments[1:]),
    ):
        end, start = before.end_contact, after.start_contact
        out.append(SequenceHandoff2D(
            base=base,
            seam_margin_before_deg=seam_margin_deg(end.alpha_rad),
            seam_margin_after_deg=seam_margin_deg(start.alpha_rad),
            rim_gap_before_m=_rim_gap(end),
            rim_gap_after_m=_rim_gap(start),
        ))
    return out


# --------------------------------------------------------------------------
# The composed result
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class ComposedSequence2D:
    """One strategy at one cell: a sequence, or the reason there is none."""

    strategy: StrategyId
    height_m: float
    top_length_m: float
    sequence: MotionSequence2D | None
    refusal: str | None
    handoffs: tuple[SequenceHandoff2D, ...] = ()
    parameters: tuple[tuple[str, float], ...] = ()
    collision_free: bool | None = None
    min_clearance_m: float | None = None
    #: How much of the top the sequence actually uses, so it can be compared
    #: with ``L_transition`` -- the spec's third completion criterion.
    top_length_used_m: float | None = None
    #: Spec 5.5's label.  ``PHYSICALLY_INFEASIBLE`` is never produced here.
    verdict: Verdict | None = None
    #: Spec 5.6.  What this pair would need that nothing here can generate.
    #: Present on a blocked pair, empty everywhere else.
    unresolved: tuple[TransitionRequirement2D, ...] = ()
    #: The frames the sequence's ``FrameRef2D``s point at.  The schema keeps
    #: frames out of the segments on purpose (spec 5.4), so the reference has
    #: to point *somewhere* -- this is that somewhere, and the driver writes it.
    frame_rows: tuple[dict, ...] = ()
    #: True when the sequence covers only the stages that succeeded, because
    #: the run failed part way.  A partial sequence is **not** a plan: it is
    #: what the leg managed before it stopped, and it exists so a stage that
    #: works inside a strategy that does not can still be priced.
    partial: bool = False
    notes: str = ""
    seconds: float = 0.0

    @property
    def composed(self) -> bool:
        """A complete plan.  A partial sequence does not count as one."""

        return self.sequence is not None and not self.partial

    def as_dict(self) -> dict:
        row = {
            "strategy": self.strategy.value,
            "obstacle_mm": self.height_m * 1e3,
            "top_length_m": self.top_length_m,
            "composed": self.composed,
            "partial": self.partial,
            "segments": 0 if self.sequence is None else len(self.sequence.segments),
            "frames": (
                0 if self.sequence is None else len(self.sequence.frame_indices)
            ),
            "collision_free": self.collision_free,
            "min_clearance_mm": (
                None if self.min_clearance_m is None else self.min_clearance_m * 1e3
            ),
            "top_length_used_mm": (
                None if self.top_length_used_m is None
                else self.top_length_used_m * 1e3
            ),
            "max_theta_jump_deg": (
                None if not self.handoffs
                else max(abs(np.rad2deg(h.base.theta_jump_rad)) for h in self.handoffs)
            ),
            "max_beta_jump_deg": (
                None if not self.handoffs
                else max(abs(np.rad2deg(h.base.beta_jump_rad)) for h in self.handoffs)
            ),
            "max_contact_jump_mm": (
                None if not self.handoffs
                else max(h.base.contact_jump_m for h in self.handoffs) * 1e3
            ),
            "min_seam_margin_deg": (
                None if not self.handoffs
                else min(min(h.seam_margin_before_deg, h.seam_margin_after_deg)
                         for h in self.handoffs)
            ),
            "max_rim_gap_change_mm": (
                None if not self.handoffs
                else max(abs(h.rim_gap_change_m) for h in self.handoffs) * 1e3
            ),
            # The accumulation check: the gap must never exceed the nominal
            # 1.2 mm Step 0 measured, however many hand-overs it crosses.
            "largest_rim_gap_mm": (
                None if not self.handoffs
                else max(h.largest_rim_gap_m for h in self.handoffs) * 1e3
            ),
            # Only a blocked pair sets this explicitly.  An ordinary refusal
            # -- a top too short, a stride too long -- is OUT_OF_ENVELOPE, not
            # a hand-over failure; guessing otherwise is the same
            # over-generalisation spec 5.5 exists to stop.
            "verdict": (
                (Verdict.COMPOSED if self.composed else Verdict.OUT_OF_ENVELOPE).value
                if self.verdict is None else self.verdict.value
            ),
            "unresolved_transitions": len(self.unresolved),
            "unresolved_kinds": ", ".join(t.kind.value for t in self.unresolved),
            "refusal": self.refusal,
            "notes": self.notes,
            "seconds": self.seconds,
        }
        row.update({name: value for name, value in self.parameters})
        return row


# --------------------------------------------------------------------------
# #1 ROLL_UP + ROLL_DOWN
# --------------------------------------------------------------------------


def _traversal_rows(result) -> list[dict]:
    """``TraversalFrame2D`` objects in the shape Step 6's builder reads.

    The builder takes the CSV row shape rather than the frame object so that a
    sequence can be rebuilt from a file alone; producing the same shape here
    keeps one reader instead of two.
    """

    rows = []
    for frame in result.trajectory:
        rows.append({
            "index": frame.index,
            "stage": frame.stage,
            "phase": frame.phase,
            "theta_deg": float(np.rad2deg(frame.theta_rad)),
            "beta_deg": float(np.rad2deg(frame.beta_rad)),
            "hip_x_m": float(frame.hip_position_world_xz_m[0]),
            "hip_z_m": float(frame.hip_position_world_xz_m[1]),
            "active_rim": frame.active_rim,
            "alpha_deg": (
                None if frame.alpha_rad is None
                else float(np.rad2deg(frame.alpha_rad))
            ),
            "contact_x_m": (
                None if frame.contact_point_world_xz_m is None
                else float(frame.contact_point_world_xz_m[0])
            ),
            "contact_z_m": (
                None if frame.contact_point_world_xz_m is None
                else float(frame.contact_point_world_xz_m[1])
            ),
            "contact_surface": frame.terrain_surface_id,
            "collision_margin_m": frame.collision_margin_m,
            "accepted": "True" if frame.accepted else "False",
        })
    return rows


def compose_roll_roll_2d(
    height_m: float,
    top_length_m: float,
    *,
    theta_climb_deg: float,
    approach_clearance_m: float,
    keep_partial: bool = False,
) -> ComposedSequence2D:
    """Run the whole Day 6--7 traversal at this cell and write it to the schema.

    ``keep_partial`` builds a sequence from the accepted frames even when the
    traversal fails, marked :attr:`ComposedSequence2D.partial`.  That is how
    Step 8 prices ``ROLL_UP`` at ``h = 160 mm``, where every theta climbs and
    none descends: the ascent is real work the leg did, and the pair it belongs
    to (``#2``) is refuted for reasons that have nothing to do with it.

    The obstacle, the start pose and the constraints all come from Day 6--7's
    own :class:`SweepSettings2D`, not from values restated here.  That is not
    tidiness: the first version built them by hand and the traversal failed
    where the sweep says it succeeds, because two coupled settings did not
    match.  ``arc_samples`` defaults to 241 on ``ObstacleSpec2D`` but the sweep
    ran at 121, and ``max_seam_bridge_m`` has to be **paired** with it --
    ``seam_bridge_for_sampling_m(121)`` is 17.5 mm against the 5 mm default
    (implementation log trap 2).  Composing through the sweep's own bundle
    makes that impossible to get wrong again.
    """

    started = time.perf_counter()
    settings = SweepSettings2D(
        obstacle_width_m=float(top_length_m),
        approach_start_clearance_m=float(approach_clearance_m),
    )
    theta_climb_rad = float(np.deg2rad(theta_climb_deg))
    obstacle = settings.obstacle(float(height_m))
    result = check_right_up_left_down_traversal(
        obstacle=obstacle,
        initial_state=settings.initial_state_for(float(height_m), theta_climb_rad),
        theta_climb=theta_climb_rad,
        constraints=settings.constraints,
    )
    elapsed = time.perf_counter() - started
    parameters = (
        ("theta_climb_deg", float(theta_climb_deg)),
        ("approach_clearance_m", float(approach_clearance_m)),
        ("arc_samples", float(settings.arc_samples)),
        ("max_seam_bridge_m", float(settings.constraints.max_seam_bridge_m)),
    )
    refusal = (
        None if result.full_success
        else f"the traversal failed at {result.failure_stage}: "
             f"{result.failure_reason}"
    )
    rows = _traversal_rows(result)
    accepted = [r for r in rows if r["accepted"] == "True"]
    if refusal is not None and not (keep_partial and len(accepted) >= 2):
        return ComposedSequence2D(
            strategy=StrategyId.ROLL_ROLL, height_m=height_m,
            top_length_m=top_length_m, sequence=None, refusal=refusal,
            parameters=parameters, seconds=elapsed,
        )

    sequence = sequence_from_traversal_frames_2d(
        rows, terrain_id=obstacle.obstacle_id,
        source_id=f"step7_roll_roll_h{height_m * 1e3:.0f}_L{top_length_m * 1e3:.0f}",
        arc_samples=settings.arc_samples,
    )
    handoffs = sequence_handoffs_2d(sequence)
    margins = [
        frame.collision_margin_m for frame in result.trajectory
        if frame.accepted and frame.collision_margin_m is not None
    ]
    return ComposedSequence2D(
        strategy=StrategyId.ROLL_ROLL, height_m=height_m,
        top_length_m=top_length_m, sequence=sequence, refusal=refusal,
        handoffs=tuple(handoffs), parameters=parameters,
        collision_free=not any(f.collision for f in result.trajectory if f.accepted),
        min_clearance_m=min(margins) if margins else None,
        frame_rows=tuple(accepted),
        partial=refusal is not None,
        top_length_used_m=(
            None if result.l_transition_m is None else float(result.l_transition_m)
        ),
        notes=(
            "top_length_used is L_transition: retract + wheel-mode roll."
            if refusal is None else
            "partial: only the stages that succeeded before the failure."
        ),
        seconds=elapsed,
    )


def swing_frame_rows(
    plan, request, *, kind: SegmentKind, index_offset: int = 0
) -> list[dict]:
    """The per-sample data a swing segment's ``FrameRef2D`` refers to.

    The schema keeps frames out of the segments on purpose (spec 5.4), so the
    reference has to point somewhere; this builds that somewhere in the same
    row shape the rolling side writes, which is what lets one reader load
    either.
    """

    samples = plan.result.samples
    fractions = np.linspace(0.0, 1.0, len(samples))
    rows = []
    for sample, fraction in zip(samples, fractions):
        hip = request.hip_trajectory.pose_at(fraction).position_world_xz_m
        rows.append({
            "index": int(sample.index) + index_offset,
            "stage": kind.value,
            "phase": kind.value,
            "time_s": float(sample.time_s),
            "theta_deg": (
                None if sample.theta_rad is None
                else float(np.rad2deg(sample.theta_rad))
            ),
            "beta_deg": (
                None if sample.beta_rad is None
                else float(np.rad2deg(sample.beta_rad))
            ),
            "hip_x_m": float(hip[0]),
            "hip_z_m": float(hip[1]),
            "active_rim": RimId(sample.rim).value,
            "alpha_deg": float(np.rad2deg(sample.alpha_rad)),
            "contact_x_m": float(sample.position_world_xz_m[0]),
            "contact_z_m": float(sample.position_world_xz_m[1]),
            "contact_surface": request.target.target_terrain_surface_id,
            "collision_margin_m": sample.terrain_clearance_m,
            "ik_converged": sample.ik_converged,
            "joint_step_deg": (
                None if sample.joint_step_rad is None
                else float(np.rad2deg(sample.joint_step_rad))
            ),
            "accepted": "True",
        })
    return rows


# --------------------------------------------------------------------------
# #4 SWING_UP + SWING_DOWN
# --------------------------------------------------------------------------


def compose_swing_swing_2d(
    height_m: float,
    top_length_m: float,
    *,
    approach_clearance_m: float,
    min_hip_lift_m: float,
    takeoff_distance_m: float,
    min_hip_hold_fraction: float,
    ascent_liftoff_rise_m: float = 0.0,
    ascent_touchdown_drop_m: float = 0.0,
    ascent_duration_scale: float = 1.0,
    descent_liftoff_rise_m: float = 0.0,
    descent_touchdown_drop_m: float = 0.0,
    descent_duration_scale: float = 1.0,
    landing_distance_m: float = SWING_LANDING_DISTANCE_M,
    theta_deg: float = 60.0,
) -> ComposedSequence2D:
    """Two swings over one obstacle, with whatever the top leaves between them.

    The ascent lands ``landing_distance_m`` past the leading edge and the
    descent leaves ``takeoff_distance_m`` short of the trailing one, so the
    two meet exactly when ``L_top = landing + takeoff``.  On a longer top they
    do not, and the gap between them is a segment **nobody has planned**: the
    engine that would roll the leg forward there needs a stop condition the
    research plan lists for Day 15--16.  That gap is measured and reported
    rather than papered over.
    """

    started = time.perf_counter()
    spec = SharedTerrainSpec2D(
        height_m=float(height_m), top_length_m=float(top_length_m),
        x_start_m=COMPOSER_FRAME_X_START_M, arc_samples=SWING_ARC_SAMPLES,
    )
    theta = float(np.deg2rad(theta_deg))
    parameters = (
        ("approach_clearance_m", float(approach_clearance_m)),
        ("min_hip_lift_m", float(min_hip_lift_m)),
        ("takeoff_distance_m", float(takeoff_distance_m)),
        ("min_hip_hold_fraction", float(min_hip_hold_fraction)),
        ("landing_distance_m", float(landing_distance_m)),
        ("ascent_liftoff_rise_m", float(ascent_liftoff_rise_m)),
        ("descent_touchdown_drop_m", float(descent_touchdown_drop_m)),
        ("descent_duration_scale", float(descent_duration_scale)),
    )

    def refuse(reason: str) -> ComposedSequence2D:
        return ComposedSequence2D(
            strategy=StrategyId.SWING_SWING, height_m=height_m,
            top_length_m=top_length_m, sequence=None, refusal=reason,
            parameters=parameters, seconds=time.perf_counter() - started,
        )

    landing_hip_x = spec.x_start_m + float(landing_distance_m)
    takeoff_hip_x = spec.x_start_m + float(top_length_m) - float(takeoff_distance_m)
    if takeoff_hip_x < landing_hip_x - 1e-12:
        return refuse(
            "the ascent lands past where the descent must leave: this top is "
            "too short for the pair."
        )

    # -- ascent ----------------------------------------------------------
    approach_hip_x = approach_hip_x_for_clearance_2d(
        spec, theta, float(approach_clearance_m)
    )
    start_scene = standing_scene_2d(
        spec, theta, hip_x_m=approach_hip_x, support_height_m=0.0
    )
    top_scene = standing_scene_2d(
        spec, theta, hip_x_m=landing_hip_x, support_height_m=spec.top_z_m
    )
    up_request = build_swing_request_2d(
        start_scene, top_scene, clearance_m=SWING_APEX_CLEARANCE_M,
        swing_duration_s=SWING_DURATION_S, sample_count=SWING_SAMPLE_COUNT,
    )
    if min_hip_lift_m > 0.0:
        up_request = replace(up_request, hip_trajectory=HipTrajectory2D(
            start_scene.hip_pose,
            HipPose2D(top_scene.hip_pose.position_world_xz_m
                      + np.array([0.0, float(min_hip_lift_m)])),
        ))
    if ascent_duration_scale != 1.0:
        up_request = replace(
            up_request,
            swing_duration_s=up_request.swing_duration_s * float(ascent_duration_scale),
        )
    up_plan = generate_swing_2d(
        up_request, arc_samples=SWING_COLLISION_ARC_SAMPLES,
        liftoff_rise_m=float(ascent_liftoff_rise_m),
        touchdown_drop_m=float(ascent_touchdown_drop_m),
    )
    if not up_plan.valid:
        return refuse(f"the ascent failed: {up_plan.failure.name}")

    # -- descent ---------------------------------------------------------
    takeoff_scene = standing_scene_2d(
        spec, theta, hip_x_m=takeoff_hip_x, support_height_m=spec.top_z_m
    )
    down_landing_x = spec.x_start_m + float(top_length_m) + 0.20
    down_scene = standing_scene_2d(
        spec, theta, hip_x_m=down_landing_x, support_height_m=0.0
    )
    down_request = build_swing_request_2d(
        takeoff_scene, down_scene, clearance_m=SWING_APEX_CLEARANCE_M,
        swing_duration_s=SWING_DURATION_S, sample_count=SWING_SAMPLE_COUNT,
    )
    if min_hip_hold_fraction > 0.0:
        down_request = replace(down_request, hip_trajectory=HipTrajectory2D(
            takeoff_scene.hip_pose,
            HipPose2D(down_scene.hip_pose.position_world_xz_m
                      + np.array([0.0, float(min_hip_hold_fraction) * float(height_m)])),
        ))
    if descent_duration_scale != 1.0:
        down_request = replace(
            down_request,
            swing_duration_s=(
                down_request.swing_duration_s * float(descent_duration_scale)
            ),
        )
    try:
        down_plan = generate_swing_2d(
            down_request, arc_samples=SWING_COLLISION_ARC_SAMPLES,
            liftoff_rise_m=float(descent_liftoff_rise_m),
            touchdown_drop_m=float(descent_touchdown_drop_m),
        )
    except ValueError as error:
        # Day 8--9's planner refuses to judge a leg that finished in the air.
        return refuse(f"the descent planner refused: {error}")
    if not down_plan.valid:
        return refuse(f"the descent failed: {down_plan.failure.name}")

    source_id = (
        f"step7_swing_swing_h{height_m * 1e3:.0f}_L{top_length_m * 1e3:.0f}"
    )
    up_segment = segment_from_swing_plan_2d(
        up_plan, up_request, kind=SegmentKind.SWING_UP, source_id=source_id,
        arc_samples=SWING_ARC_SAMPLES,
        leg_arc_samples=SWING_COLLISION_ARC_SAMPLES,
        apex_clearance_m=SWING_APEX_CLEARANCE_M,
        liftoff_rise_m=float(ascent_liftoff_rise_m),
        touchdown_drop_m=float(ascent_touchdown_drop_m),
        duration_scale=float(ascent_duration_scale),
    )
    down_segment = segment_from_swing_plan_2d(
        down_plan, down_request, kind=SegmentKind.SWING_DOWN, source_id=source_id,
        arc_samples=SWING_ARC_SAMPLES,
        leg_arc_samples=SWING_COLLISION_ARC_SAMPLES,
        apex_clearance_m=SWING_APEX_CLEARANCE_M,
        liftoff_rise_m=float(descent_liftoff_rise_m),
        touchdown_drop_m=float(descent_touchdown_drop_m),
        duration_scale=float(descent_duration_scale),
    )
    # The two swings index their own samples from zero, so the descent's are
    # renumbered: within one sequence a frame index names one frame.
    down_segment = replace(down_segment, frames=replace(
        down_segment.frames,
        indices=tuple(
            i + up_segment.frames.indices[-1] + 1
            for i in down_segment.frames.indices
        ),
    ))

    sequence = MotionSequence2D(
        terrain_id=f"day10_11_obstacle_h{height_m * 1e3:.0f}",
        segments=(up_segment, down_segment),
        notes=(
            "the top crossing between the two swings is not a planned segment; "
            "see gap_on_top_m."
        ),
    )
    handoffs = sequence_handoffs_2d(sequence)
    clearances = [
        p.collision.minimum_clearance_m for p in (up_plan, down_plan)
        if p.collision is not None
    ]
    gap_on_top_m = float(takeoff_hip_x - landing_hip_x)
    return ComposedSequence2D(
        strategy=StrategyId.SWING_SWING, height_m=height_m,
        top_length_m=top_length_m, sequence=sequence, refusal=None,
        handoffs=tuple(handoffs),
        parameters=parameters + (("gap_on_top_m", gap_on_top_m),),
        collision_free=True,
        min_clearance_m=min(clearances) if clearances else None,
        top_length_used_m=float(landing_distance_m) + float(takeoff_distance_m),
        frame_rows=tuple(
            swing_frame_rows(up_plan, up_request, kind=SegmentKind.SWING_UP)
            + swing_frame_rows(
                down_plan, down_request, kind=SegmentKind.SWING_DOWN,
                index_offset=up_segment.frames.indices[-1] + 1,
            )
        ),
        notes=(
            "top_length_used is landing_distance + takeoff_distance."
            + ("" if gap_on_top_m <= 1e-9 else
               f"  {gap_on_top_m * 1e3:.0f} mm of top is crossed by a segment "
               "no planner in this project can yet generate.")
        ),
        seconds=time.perf_counter() - started,
    )


# --------------------------------------------------------------------------
# #5 SWING_OVER
# --------------------------------------------------------------------------


def compose_swing_over_2d(
    height_m: float,
    top_length_m: float,
    *,
    theta_deg: float,
    approach_clearance_m: float = 0.04,
    duration_scale: float = 1.0,
) -> ComposedSequence2D:
    """One swing across the whole obstacle, touching nothing on top."""

    started = time.perf_counter()
    spec = SharedTerrainSpec2D(
        height_m=float(height_m), top_length_m=float(top_length_m),
        x_start_m=COMPOSER_FRAME_X_START_M, arc_samples=SWING_ARC_SAMPLES,
    )
    theta = float(np.deg2rad(theta_deg))
    parameters = (
        ("theta_deg", float(theta_deg)),
        ("approach_clearance_m", float(approach_clearance_m)),
    )
    try:
        hip_x = approach_hip_x_for_clearance_2d(
            spec, theta, float(approach_clearance_m)
        )
        trailing_x = spec.x_start_m + float(top_length_m)
        landing_x = trailing_x + (spec.x_start_m - hip_x)
        start_scene = standing_scene_2d(
            spec, theta, hip_x_m=hip_x, support_height_m=0.0
        )
        target_scene = standing_scene_2d(
            spec, theta, hip_x_m=landing_x, support_height_m=0.0
        )
        request = build_swing_request_2d(
            start_scene, target_scene, clearance_m=SWING_APEX_CLEARANCE_M,
            swing_duration_s=SWING_DURATION_S, sample_count=SWING_SAMPLE_COUNT,
        )
    except (ValueError, KeyError) as error:
        return ComposedSequence2D(
            strategy=StrategyId.SWING_OVER, height_m=height_m,
            top_length_m=top_length_m, sequence=None,
            refusal=f"no legal stance: {error}", parameters=parameters,
            seconds=time.perf_counter() - started,
        )

    if duration_scale != 1.0:
        request = replace(
            request,
            swing_duration_s=request.swing_duration_s * float(duration_scale),
        )
    plan = generate_swing_2d(request, arc_samples=SWING_COLLISION_ARC_SAMPLES)
    if not plan.valid:
        return ComposedSequence2D(
            strategy=StrategyId.SWING_OVER, height_m=height_m,
            top_length_m=top_length_m, sequence=None,
            refusal=f"the over-swing failed: {plan.failure.name}",
            parameters=parameters, seconds=time.perf_counter() - started,
        )

    segment = segment_from_swing_plan_2d(
        plan, request, kind=SegmentKind.SWING_OVER,
        source_id=f"step7_swing_over_h{height_m * 1e3:.0f}"
                  f"_L{top_length_m * 1e3:.0f}",
        arc_samples=SWING_ARC_SAMPLES,
        leg_arc_samples=SWING_COLLISION_ARC_SAMPLES,
        apex_clearance_m=SWING_APEX_CLEARANCE_M, liftoff_rise_m=0.0,
        touchdown_drop_m=0.0, duration_scale=float(duration_scale),
    )
    sequence = MotionSequence2D(
        terrain_id=f"day10_11_obstacle_h{height_m * 1e3:.0f}",
        segments=(segment,),
        notes="one segment: the obstacle top is never touched.",
    )
    return ComposedSequence2D(
        strategy=StrategyId.SWING_OVER, height_m=height_m,
        top_length_m=top_length_m, sequence=sequence, refusal=None,
        handoffs=(), parameters=parameters + (("stride_m", landing_x - hip_x),),
        collision_free=True,
        min_clearance_m=(
            None if plan.collision is None else plan.collision.minimum_clearance_m
        ),
        # The over-swing uses none of the top: that is the whole point of it.
        top_length_used_m=0.0,
        frame_rows=tuple(
            swing_frame_rows(plan, request, kind=SegmentKind.SWING_OVER)
        ),
        notes="uses no top length; bounded instead by the stride it needs.",
        seconds=time.perf_counter() - started,
    )


#: The entry condition each blocked pair's descent needs, in words.  Not a
#: pose: pinning one down is exactly the work spec 5.6 defers.
BLOCKED_TARGET_CONDITION: dict[StrategyId, str] = {
    StrategyId.SWING_ROLL: (
        "LEFT_RIM_READY -- left rim carrying, theta = 17 deg, near the trailing "
        "edge, with a legal trailing-edge approach"
    ),
    StrategyId.ROLL_SWING: (
        "a foot-rim take-off on the top above theta = 35 deg (Step 3 F's floor)"
    ),
}


def top_reposition_requirement_2d(
    strategy: StrategyId, height_m: float, top_length_m: float
) -> TransitionRequirement2D:
    """The unresolved ``TOP_REPOSITION`` a blocked pair would need.  Spec 5.6.

    The source contact is the pose the leg would actually be in on the top --
    a standing pose at Step 2 / Step 3's theta -- so the record points at
    something real.  Everything past that point is deliberately left open:
    the mid-air theta, beta and duration are what a multi-leg stage decides,
    and guessing them here would publish an assumption as a result.
    """

    spec = SharedTerrainSpec2D(
        height_m=float(height_m), top_length_m=float(top_length_m),
        x_start_m=COMPOSER_FRAME_X_START_M, arc_samples=SWING_ARC_SAMPLES,
    )
    theta = float(np.deg2rad(60.0))
    scene = standing_scene_2d(
        spec, theta, hip_x_m=spec.x_start_m + SWING_LANDING_DISTANCE_M,
        support_height_m=spec.top_z_m,
    )
    start = scene.hip_pose.position_world_xz_m
    blocked = BLOCKED_PAIRS[strategy]
    return TransitionRequirement2D(
        kind=TransitionKind.TOP_REPOSITION,
        source_contact=PointContact2D(
            rim=RimId.FOOT, alpha_rad=0.0,
            point_world_xz_m=(float(start[0]), spec.top_z_m),
            surface_id=f"{spec.obstacle_id}_top",
            theta_rad=theta, beta_rad=0.0,
            hip_xz_m=(float(start[0]), float(start[1])),
        ),
        target_condition=BLOCKED_TARGET_CONDITION[strategy],
        evidence=blocked.evidence,
        requires_external_support=True,
        resolved=False,
    )


# --------------------------------------------------------------------------
# The dispatcher
# --------------------------------------------------------------------------


def compose_2d(
    height_m: float,
    top_length_m: float,
    tables: DecisionTables2D,
    *,
    strategy: StrategyId | None = None,
) -> ComposedSequence2D:
    """Compose whatever Step 5's rule picks here -- or say why there is none.

    ``strategy`` forces a particular one, which is how the two refuted pairs
    are asked for explicitly: they answer with their refutation rather than
    being silently absent from the output.
    """

    decision = decide_2d(height_m, top_length_m, tables)
    chosen = strategy if strategy is not None else decision.winner
    if chosen is None:
        return ComposedSequence2D(
            strategy=StrategyId.ROLL_ROLL, height_m=height_m,
            top_length_m=top_length_m, sequence=None,
            refusal="no strategy is available at this cell.",
        )
    if chosen in BLOCKED_PAIRS:
        blocked = BLOCKED_PAIRS[chosen]
        return ComposedSequence2D(
            strategy=chosen, height_m=height_m, top_length_m=top_length_m,
            sequence=None, refusal=blocked.summary, verdict=blocked.verdict,
            # The route back, as data rather than prose: a reader of the CSV
            # alone must be able to see this is unsolved, not impossible.
            unresolved=(
                top_reposition_requirement_2d(chosen, height_m, top_length_m),
            ) if blocked.multileg_route else (),
            notes=(
                "the two primitives will not chain (spec 5.5); recorded with "
                "its route back rather than skipped."
            ),
        )

    cell = next(c for c in decision.cells if c.strategy is chosen)
    if cell.availability is not Availability.FEASIBLE:
        return ComposedSequence2D(
            strategy=chosen, height_m=height_m, top_length_m=top_length_m,
            sequence=None,
            refusal=f"{cell.availability.value} ({cell.limiter.value}): {cell.reason}",
        )
    parameters = dict(cell.parameters)

    if chosen is StrategyId.ROLL_ROLL:
        return compose_roll_roll_2d(
            height_m, top_length_m,
            theta_climb_deg=parameters["theta_climb_deg"],
            approach_clearance_m=0.04,
        )
    if chosen is StrategyId.SWING_SWING:
        return compose_swing_swing_2d(
            height_m, top_length_m,
            approach_clearance_m=parameters["approach_clearance_m"],
            min_hip_lift_m=parameters["min_hip_lift_m"],
            takeoff_distance_m=parameters["takeoff_distance_m"],
            min_hip_hold_fraction=parameters["min_hip_hold_fraction"],
            ascent_liftoff_rise_m=parameters["ascent_liftoff_rise_m"],
            ascent_touchdown_drop_m=parameters["ascent_touchdown_drop_m"],
            ascent_duration_scale=parameters["ascent_duration_scale"],
            descent_liftoff_rise_m=parameters["descent_liftoff_rise_m"],
            descent_touchdown_drop_m=parameters["descent_touchdown_drop_m"],
            descent_duration_scale=parameters["descent_duration_scale"],
        )
    clearance = parameters["approach_clearance_m"]
    return compose_swing_over_2d(
        height_m, top_length_m, theta_deg=parameters["theta_deg"],
        approach_clearance_m=(0.04 if not np.isfinite(clearance) else clearance),
        duration_scale=parameters["duration_scale"],
    )
