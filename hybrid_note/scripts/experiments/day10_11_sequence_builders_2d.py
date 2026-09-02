"""Day 10--11 Step 6: build a :class:`MotionSequence2D` from what already ran.

The schema in ``day10_11_motion_schema_2d`` is only worth anything if the two
things Day 10--11 actually has can be written into it **without loss**.  This
module does that, and provides the checks that say whether it succeeded:

* :func:`sequence_from_traversal_frames_2d` -- Day 6--7 Step 10R's full
  right-up / left-down traversal, wheel-mode segment included;
* :func:`segment_from_swing_plan_2d` -- one Day 8--9 swing, with its
  ``liftoff_rise`` / ``touchdown_drop`` / duration / clearance intact;
* :func:`coverage_report_2d` and :func:`handoff_report_2d` -- the evidence.

**Endpoints are point contacts, and the rolling description sits beside them.**
Spec 5.4 wrote ``start_contact: ContactState | RollingContact``, an either/or.
The data says both are needed at once: Step 7 has to compare joint values
across a hand-over, which needs a definite pose at each end, while spec 2.5's
complaint is about what happens *between* the ends.  So the endpoints are
always :class:`PointContact2D` and :class:`RollingContact2D` describes the
interior.  The union type stays in the schema because the spec allows the other
reading; nothing here produces it.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass
from itertools import groupby

import numpy as np

from hybrid_note.scripts.experiments.cartesian_swing_contract_2d import (
    SwingRequest2D,
)
from hybrid_note.scripts.experiments.cartesian_swing_planner_2d import SwingPlan2D
from hybrid_note.scripts.experiments.day10_11_concession_2d import (
    BodyRequirementKind,
)
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    BodyRequirement2D,
    FrameRef2D,
    MotionSegment2D,
    MotionSequence2D,
    PointContact2D,
    RollSampling2D,
    RollingContact2D,
    RollingMode,
    SegmentKind,
    SwingSampling2D,
    SwingShaping2D,
)

#: Day 6--7 labels its stages with these names; the schema's kinds use the same
#: words for the four that overlap.  Mapped explicitly so a new Day 6--7 phase
#: fails loudly here rather than being silently dropped into a wrong kind.
STAGE_TO_KIND = {
    "APPROACH": SegmentKind.APPROACH,
    "ROLL_UP": SegmentKind.ROLL_UP,
    "WHEEL_TRANSITION": SegmentKind.WHEEL_TRANSITION,
    "ROLL_DOWN": SegmentKind.ROLL_DOWN,
}


def _modal_step(values: np.ndarray) -> float | None:
    """The nominal increment, ignoring the truncated last step.

    Step 10R steps ``beta`` by a fixed increment until a stop condition fires,
    so most phases have one step value and a few have a short final one.  The
    mode is the generating parameter; the exact sequence stays in the frames.
    """

    if values.size == 0:
        return None
    rounded = np.round(values, 9)
    counts = Counter(rounded.tolist())
    return float(counts.most_common(1)[0][0])


def _point_contact(row: dict) -> PointContact2D:
    return PointContact2D(
        rim=row["active_rim"],
        alpha_rad=float(np.deg2rad(float(row["alpha_deg"]))),
        point_world_xz_m=(float(row["contact_x_m"]), float(row["contact_z_m"])),
        surface_id=row["contact_surface"],
        theta_rad=float(np.deg2rad(float(row["theta_deg"]))),
        beta_rad=float(np.deg2rad(float(row["beta_deg"]))),
        hip_xz_m=(float(row["hip_x_m"]), float(row["hip_z_m"])),
    )


def sequence_from_traversal_frames_2d(
    rows: Sequence[dict],
    *,
    terrain_id: str,
    source_id: str,
    arc_samples: int,
    accepted_only: bool = True,
) -> MotionSequence2D:
    """Write Day 6--7 Step 10R's traversal into the schema.

    Segments are cut at ``(stage, phase)`` changes, which is how the traversal
    itself is organised.  Every accepted frame lands in exactly one segment;
    :func:`coverage_report_2d` checks that rather than assuming it.

    ``duration_s`` stays ``None`` throughout: the traversal is quasi-static and
    Day 6--7 never assigned it time.  Inventing one here would put a fabricated
    number exactly where a real one has to go later.
    """

    ordered = sorted(
        (r for r in rows if not accepted_only or r["accepted"] == "True"),
        key=lambda r: int(r["index"]),
    )
    if not ordered:
        raise ValueError("no accepted frames to build a sequence from.")

    segments: list[MotionSegment2D] = []
    for (stage, phase), group in groupby(
        ordered, key=lambda r: (r["stage"], r["phase"])
    ):
        frames = list(group)
        if stage not in STAGE_TO_KIND:
            raise ValueError(
                f"unmapped Day 6--7 stage {stage!r}; add it to STAGE_TO_KIND "
                "rather than letting it fall into a wrong kind."
            )
        alpha = np.deg2rad([float(r["alpha_deg"]) for r in frames])
        beta = np.deg2rad([float(r["beta_deg"]) for r in frames])
        theta = np.deg2rad([float(r["theta_deg"]) for r in frames])
        hip_x = np.array([float(r["hip_x_m"]) for r in frames])
        hip_z = np.array([float(r["hip_z_m"]) for r in frames])

        rolling = RollingContact2D(
            rim=frames[0]["active_rim"],
            surface_ids=tuple(sorted({r["contact_surface"] for r in frames})),
            alpha_range_rad=(float(alpha.min()), float(alpha.max())),
            beta_range_rad=(float(beta.min()), float(beta.max())),
            theta_range_rad=(float(theta.min()), float(theta.max())),
            contact_start_xz_m=(
                float(frames[0]["contact_x_m"]), float(frames[0]["contact_z_m"])
            ),
            contact_end_xz_m=(
                float(frames[-1]["contact_x_m"]), float(frames[-1]["contact_z_m"])
            ),
        )
        theta_step = _modal_step(np.diff(theta))
        segments.append(MotionSegment2D(
            kind=STAGE_TO_KIND[stage],
            phase_label=phase,
            start_contact=_point_contact(frames[0]),
            end_contact=_point_contact(frames[-1]),
            sampling=RollSampling2D(
                arc_samples=arc_samples,
                # A one-frame phase has no step to observe; fall back to the
                # nominal 1 degree the engine uses, and say so by keeping the
                # frame count on the reference.
                beta_step_rad=(
                    _modal_step(np.diff(beta)) or float(np.deg2rad(-1.0))
                ),
                theta_step_rad=(
                    None if theta_step is None or abs(theta_step) < 1e-12
                    else theta_step
                ),
            ),
            body_requirement=BodyRequirement2D(
                kind=BodyRequirementKind.TRACK,
                x_range_m=(float(hip_x.min()), float(hip_x.max())),
                hip_z_profile_m=hip_z,
            ),
            frames=FrameRef2D(
                source_id=source_id,
                indices=tuple(int(r["index"]) for r in frames),
            ),
            duration_s=None,
            rolling=rolling,
        ))

    return MotionSequence2D(
        terrain_id=terrain_id,
        segments=tuple(segments),
        notes=(
            "Day 6-7 Step 10R, quasi-static: no segment carries a duration "
            "because the traversal was never assigned time."
        ),
    )


def segment_from_swing_plan_2d(
    plan: SwingPlan2D,
    request: SwingRequest2D,
    *,
    kind: SegmentKind,
    source_id: str,
    arc_samples: int,
    leg_arc_samples: int,
    apex_clearance_m: float,
    liftoff_rise_m: float,
    touchdown_drop_m: float,
    duration_scale: float = 1.0,
    mid_fractions: tuple[float, float] = (0.35, 0.65),
) -> MotionSegment2D:
    """Write one Day 8--9 swing into the schema, shaping knobs included.

    The body requirement is a ``LOWER_BOUND``: the swing needs the hip to be at
    least as high as its trajectory takes it, and is free to meet that with any
    shape -- which is exactly the asymmetry Step 4's comparison rule turns on.
    """

    if not kind.is_swing:
        raise ValueError(f"{kind.value} is not a swing kind.")
    samples = plan.result.samples
    if len(samples) < 2:
        raise ValueError("a swing segment needs at least two samples.")

    fractions = np.linspace(0.0, 1.0, len(samples))
    hip = np.array(
        [request.hip_trajectory.pose_at(s).position_world_xz_m for s in fractions]
    )
    final = samples[-1]

    return MotionSegment2D(
        kind=kind,
        phase_label=kind.value,
        start_contact=PointContact2D(
            rim=request.start.rim,
            alpha_rad=request.start.alpha_rad,
            point_world_xz_m=request.start.contact_point_world_xz_m,
            surface_id=request.start.terrain_surface_id,
            theta_rad=request.start.theta_rad,
            beta_rad=request.start.beta_rad,
            hip_xz_m=request.start.hip_pose.position_world_xz_m,
        ),
        end_contact=PointContact2D(
            rim=final.rim,
            alpha_rad=final.alpha_rad,
            point_world_xz_m=final.position_world_xz_m,
            surface_id=request.target.target_terrain_surface_id,
            # The touchdown pose is the IK's *output*, not the request's input
            # (implementation log trap 16), so it is read off the final sample.
            theta_rad=float(final.theta_rad),
            beta_rad=float(final.beta_rad),
            hip_xz_m=hip[-1],
        ),
        sampling=SwingSampling2D(
            arc_samples=arc_samples,
            sample_count=request.sample_count,
            leg_arc_samples=leg_arc_samples,
            max_joint_step_rad=request.constraints.max_joint_step_rad,
        ),
        body_requirement=BodyRequirement2D(
            kind=BodyRequirementKind.LOWER_BOUND,
            x_range_m=(float(hip[:, 0].min()), float(hip[:, 0].max())),
            hip_z_min_m=float(hip[:, 1].max()),
        ),
        frames=FrameRef2D(
            source_id=source_id,
            indices=tuple(int(s.index) for s in samples),
        ),
        duration_s=float(request.swing_duration_s),
        swing_shaping=SwingShaping2D(
            apex_clearance_m=apex_clearance_m,
            liftoff_rise_m=liftoff_rise_m,
            touchdown_drop_m=touchdown_drop_m,
            duration_scale=duration_scale,
            mid_fractions=mid_fractions,
        ),
    )


# --------------------------------------------------------------------------
# Is it lossless?
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class CoverageReport2D:
    """Whether the sequence accounts for every frame it claims to describe."""

    source_frame_count: int
    covered_frame_count: int
    missing: tuple[int, ...]
    duplicated: tuple[int, ...]
    #: Frames in the source that the sequence never mentions and that were
    #: rejected anyway -- Step 10R has one (index 11).  Reported separately so
    #: "not covered" is not confused with "lost".
    rejected_and_skipped: tuple[int, ...]

    @property
    def lossless(self) -> bool:
        return not self.missing and not self.duplicated

    def as_dict(self) -> dict:
        return {
            "source_frames": self.source_frame_count,
            "covered_frames": self.covered_frame_count,
            "missing": len(self.missing),
            "duplicated": len(self.duplicated),
            "rejected_and_skipped": len(self.rejected_and_skipped),
            "lossless": self.lossless,
        }


def coverage_report_2d(
    sequence: MotionSequence2D, rows: Sequence[dict]
) -> CoverageReport2D:
    accepted = {int(r["index"]) for r in rows if r["accepted"] == "True"}
    rejected = {int(r["index"]) for r in rows} - accepted
    covered = list(sequence.frame_indices)
    covered_set = set(covered)
    return CoverageReport2D(
        source_frame_count=len(accepted),
        covered_frame_count=len(covered),
        missing=tuple(sorted(accepted - covered_set)),
        duplicated=tuple(
            sorted(i for i, n in Counter(covered).items() if n > 1)
        ),
        rejected_and_skipped=tuple(sorted(rejected - covered_set)),
    )


def _wrap_to_pi(angle_rad: float) -> float:
    """Fold an angle difference into ``(-pi, pi]``.

    ``alpha`` is a rim coordinate that wraps at ``+-180 deg``, so the
    ``WHEEL_MODE_TOP_ROLL -> LEFT_RIM_READY`` hand-over reads as a 358.8 degree
    jump unwrapped.  It is a change of chart, not a motion -- the contact point
    moves 2.9 mm across that same boundary.
    """

    return float((angle_rad + np.pi) % (2.0 * np.pi) - np.pi)


@dataclass(frozen=True)
class HandoffReport2D:
    """One boundary between consecutive segments."""

    from_kind: str
    to_kind: str
    from_phase: str
    to_phase: str
    rim_changed: bool
    contact_jump_m: float
    alpha_jump_rad: float
    theta_jump_rad: float
    beta_jump_rad: float
    hip_jump_m: float

    def as_dict(self) -> dict:
        return {
            "from": f"{self.from_kind}/{self.from_phase}",
            "to": f"{self.to_kind}/{self.to_phase}",
            "rim_changed": self.rim_changed,
            "contact_jump_mm": self.contact_jump_m * 1e3,
            "alpha_jump_deg": float(np.rad2deg(self.alpha_jump_rad)),
            "theta_jump_deg": float(np.rad2deg(self.theta_jump_rad)),
            "beta_jump_deg": float(np.rad2deg(self.beta_jump_rad)),
            "hip_jump_mm": self.hip_jump_m * 1e3,
        }


def handoff_between_2d(
    before: MotionSegment2D, after: MotionSegment2D
) -> HandoffReport2D:
    """One boundary, measured.  Day 12 chains segments across frame sources,
    which :class:`MotionSequence2D` forbids, so the per-boundary measurement is
    factored out here rather than re-implemented there.
    """

    end, start = before.end_contact, after.start_contact
    if not isinstance(end, PointContact2D) or not isinstance(start, PointContact2D):
        raise TypeError(
            "hand-off checking needs definite poses at both ends; see this "
            "module's docstring on why endpoints are always PointContact2D."
        )
    return HandoffReport2D(
        from_kind=before.kind.value, to_kind=after.kind.value,
        from_phase=before.phase_label, to_phase=after.phase_label,
        rim_changed=end.rim is not start.rim,
        contact_jump_m=float(
            np.linalg.norm(start.point_world_xz_m - end.point_world_xz_m)
        ),
        alpha_jump_rad=_wrap_to_pi(start.alpha_rad - end.alpha_rad),
        theta_jump_rad=float(start.theta_rad - end.theta_rad),
        beta_jump_rad=float(start.beta_rad - end.beta_rad),
        hip_jump_m=float(np.linalg.norm(start.hip_xz_m - end.hip_xz_m)),
    )


def handoff_report_2d(sequence: MotionSequence2D) -> list[HandoffReport2D]:
    """What jumps at each segment boundary.

    A large ``contact_jump`` is not automatically a bug: a rim change at the
    ``+-180 deg`` seam moves the contact point 162 mm by construction (trap 4),
    and a corner pivot ends by transferring contact to the lower ground.  What
    must stay small is the **joint** jump -- the leg cannot teleport.
    """

    return [
        handoff_between_2d(before, after)
        for before, after in zip(sequence.segments, sequence.segments[1:])
    ]
