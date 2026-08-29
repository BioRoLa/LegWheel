"""Day 8--9 Step 6: run the Day 3--5 collision checker over a swing trajectory.

Steps 2--3 place a *point*; Step 5 solves the joints that put it there.  Step 6
asks the question neither of them can: does the rest of the leg fit?

A leg-wheel is not a point foot.  The planned contact point can clear an
obstacle by a comfortable margin while the tyre arc on the other side of the
wheel, or a linkage bar, is already inside the vertical face.  So every sample
is expanded back into full leg geometry and handed to the existing terrain
query::

    q_i -> sample_contact_geometry_points -> SampledLegGeometry2D
        -> query_contact(geometry, terrain)
            geometry penetrations   (ground and obstacle-top)
            vertical-face collisions
            link collisions

Nothing here implements geometry or terrain logic.  The leg surface comes from
the same sampler the contact map uses, the checks are the Day 3--5 detectors,
and the clearance metric is the one Day 6--7 already defined for rolling.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from legwheel.planners.hybrid import (
    ContactQueryResult2D,
    HipPose2D,
    legacy_leg_link_segments_2d,
    query_contact,
    sampled_leg_geometry_from_legacy_records,
)

from ..kinematics.ground_contact_single_pose import sample_contact_geometry_points
from .cartesian_swing_contract_2d import (
    SwingFailure,
    SwingResult2D,
    SwingSample2D,
)
from .single_leg_rolling_scene_2d import SingleLegRollingScene2D

# Day 6--7 already defined "how close did the leg come to something it is not
# resting on", including the subtlety that the supporting surface must be
# excluded or the metric is identically zero.  Reused rather than restated;
# a swing is airborne for most of its length, so the same metric reduces to
# plain distance-to-terrain there and stays meaningful at the two endpoints.
from .right_up_left_down_full_traversal_2d import (
    _collision_margin_m as day6_7_collision_margin_m,
)

#: Arc samples per tyre when rebuilding leg geometry.  The terrain query runs
#: per sampled point, so this is the dominant cost of a trajectory check.
#: Measured against 61 samples on the Day 8--9 cases: the verdict, the depth
#: and the minimum clearance agree, but the first colliding sample can shift by
#: one, and where two overlaps have nearly equal depth the *primary* one
#: reported can differ.  121 is kept as the default for that reason; halving it
#: is a deliberate speed/detail trade, not a free one.
DEFAULT_ARC_SAMPLES = 121

#: Penetrations shallower than this are not treated as collisions: Step 5
#: solves the joints to about 10 um, so a 5 um overlap is solver noise, not
#: evidence that the leg hit anything.
DEFAULT_PENETRATION_TOLERANCE_M = 1e-5


def swing_sample_scene_2d(
    result: SwingResult2D,
    sample: SwingSample2D,
    *,
    arc_samples: int = DEFAULT_ARC_SAMPLES,
) -> SingleLegRollingScene2D:
    """Rebuild the full leg geometry of one solved sample, in world coordinates.

    Returns a Day 6--7 ``SingleLegRollingScene2D`` so that every existing
    query, margin and plotting helper applies unchanged.
    """

    if sample.theta_rad is None or sample.beta_rad is None:
        raise ValueError("sample has no joint solution; run Step 5 first.")
    request = result.request
    fraction = float(np.clip(sample.time_s / request.swing_duration_s, 0.0, 1.0))
    hip_pose: HipPose2D = request.hip_trajectory.pose_at(fraction)
    records = sample_contact_geometry_points(
        sample.theta_rad,
        sample.beta_rad,
        arc_samples=int(arc_samples),
        include_reference_points=True,
    )
    geometry = sampled_leg_geometry_from_legacy_records(
        records,
        hip_pose,
        link_segments_hip_xz_m=legacy_leg_link_segments_2d(sample.theta_rad, sample.beta_rad),
    )
    return SingleLegRollingScene2D(
        theta_rad=sample.theta_rad,
        beta_rad=sample.beta_rad,
        gamma_rad=0.0,
        hip_pose=hip_pose,
        terrain=request.terrain,
        geometry=geometry,
    )


def _collision_events(
    query: ContactQueryResult2D,
    penetration_tolerance_m: float,
) -> tuple[tuple[str, str, float], ...]:
    """Return ``(kind, surface_id, depth)`` for every overlap worth reporting."""

    events = []
    for item in query.geometry_penetrations:
        if item.penetration_depth_m > penetration_tolerance_m:
            events.append(
                ("geometry_penetration", item.terrain_surface_id, float(item.penetration_depth_m))
            )
    for item in query.collisions:
        if item.penetration_depth_m > penetration_tolerance_m:
            events.append(
                (
                    "vertical_face_collision",
                    item.terrain_surface_id,
                    float(item.penetration_depth_m),
                )
            )
    for item in query.link_collisions:
        if item.penetration_depth_m > penetration_tolerance_m:
            events.append(
                ("link_collision", item.terrain_surface_id, float(item.penetration_depth_m))
            )
    return tuple(events)


@dataclass(frozen=True)
class SwingCollisionReport2D:
    """The Step 6 output the planning note asks for, plus where it happened."""

    collision_free: bool
    first_collision_index: int | None
    collision_type: str | None
    collision_surface_id: str | None
    collision_depth_m: float | None
    collision_detail: str | None
    minimum_clearance_m: float | None
    minimum_clearance_index: int | None
    checked_sample_count: int
    arc_samples: int

    def as_dict(self) -> dict:
        return {
            "collision_free": self.collision_free,
            "first_collision_index": self.first_collision_index,
            "collision_type": self.collision_type,
            "collision_surface_id": self.collision_surface_id,
            "collision_depth_mm": (
                None if self.collision_depth_m is None else self.collision_depth_m * 1e3
            ),
            "minimum_clearance_mm": (
                None if self.minimum_clearance_m is None else self.minimum_clearance_m * 1e3
            ),
            "minimum_clearance_index": self.minimum_clearance_index,
            "checked_samples": self.checked_sample_count,
            "arc_samples": self.arc_samples,
        }


def check_swing_trajectory_collisions_2d(
    result: SwingResult2D,
    *,
    arc_samples: int = DEFAULT_ARC_SAMPLES,
    penetration_tolerance_m: float = DEFAULT_PENETRATION_TOLERANCE_M,
) -> tuple[SwingResult2D, SwingCollisionReport2D, tuple[ContactQueryResult2D, ...]]:
    """Check every solved sample's full leg geometry against the terrain.

    Every sample is checked even after the first collision, so the extent of
    the problem is visible rather than just its onset; the report names the
    earliest one.  The raw queries come back too, because the interesting
    diagnostics -- which rim, which surface, how deep -- do not belong in the
    frozen sample schema.

    Passing this is still not a valid swing: the touchdown state (Step 7) and
    touchdown velocity (Step 8) have not been checked.
    """

    if not isinstance(result, SwingResult2D):
        raise TypeError("result must be a SwingResult2D.")
    if not result.samples:
        raise ValueError("result carries no samples; run Steps 2-5 first.")
    if any(sample.theta_rad is None for sample in result.samples):
        raise ValueError("result has no joint solution; run Step 5 first.")

    limits = result.request.constraints
    checked_samples = []
    queries = []
    first_collision = None
    for sample in result.samples:
        scene = swing_sample_scene_2d(result, sample, arc_samples=arc_samples)
        query = query_contact(
            scene.geometry,
            scene.terrain,
            contact_tolerance_m=limits.contact_tolerance_m,
            collision_tolerance_m=limits.collision_tolerance_m,
        )
        events = _collision_events(query, penetration_tolerance_m)
        clearance_m = day6_7_collision_margin_m(scene, query)
        if events and first_collision is None:
            first_collision = (sample.index, events)
        checked_samples.append(
            replace(
                sample,
                collision_free=not events,
                terrain_clearance_m=clearance_m,
            )
        )
        queries.append(query)

    samples = tuple(checked_samples)
    clearances = [
        sample.terrain_clearance_m
        for sample in samples
        if sample.terrain_clearance_m is not None
    ]
    minimum_clearance_m = min(clearances) if clearances else None
    minimum_index = None
    if minimum_clearance_m is not None:
        minimum_index = next(
            sample.index
            for sample in samples
            if sample.terrain_clearance_m == minimum_clearance_m
        )

    if first_collision is None:
        report = SwingCollisionReport2D(
            collision_free=True,
            first_collision_index=None,
            collision_type=None,
            collision_surface_id=None,
            collision_depth_m=None,
            collision_detail=None,
            minimum_clearance_m=minimum_clearance_m,
            minimum_clearance_index=minimum_index,
            checked_sample_count=len(samples),
            arc_samples=int(arc_samples),
        )
        failure = result.failure
        failure_index = result.failure_sample_index
        detail = "Joints solved and collision-free; Steps 7-8 have not run."
        if failure not in (SwingFailure.NOT_EVALUATED, SwingFailure.NONE):
            # An earlier step already failed; Step 6 does not overwrite that.
            detail = result.failure_detail
    else:
        index, events = first_collision
        # Report the deepest overlap at that sample as the primary type: the
        # groups are not mutually exclusive and the deepest one is the least
        # likely to be a grazing artefact.
        kind, surface_id, depth_m = max(events, key=lambda item: item[2])
        report = SwingCollisionReport2D(
            collision_free=False,
            first_collision_index=index,
            collision_type=kind,
            collision_surface_id=surface_id,
            collision_depth_m=depth_m,
            collision_detail=", ".join(
                f"{event_kind} on {event_surface} ({event_depth * 1e3:.2f} mm)"
                for event_kind, event_surface, event_depth in events
            ),
            minimum_clearance_m=minimum_clearance_m,
            minimum_clearance_index=minimum_index,
            checked_sample_count=len(samples),
            arc_samples=int(arc_samples),
        )
        failure = SwingFailure.TERRAIN_COLLISION
        failure_index = index
        detail = f"sample {index}: {report.collision_detail}"
        if result.failure not in (SwingFailure.NOT_EVALUATED, SwingFailure.NONE):
            # Keep the earlier, more fundamental failure as the headline.
            failure = result.failure
            failure_index = result.failure_sample_index
            detail = f"{result.failure_detail} Also: sample {index}: {report.collision_detail}"

    return (
        SwingResult2D(
            request=result.request,
            samples=samples,
            valid=False,
            failure=failure,
            failure_sample_index=failure_index,
            failure_detail=detail,
            minimum_terrain_clearance_m=minimum_clearance_m,
        ),
        report,
        tuple(queries),
    )


def collision_frame_rows(
    result: SwingResult2D,
    queries: tuple[ContactQueryResult2D, ...],
) -> list[dict]:
    """Per-sample collision detail, for inspecting where a swing goes wrong."""

    if len(queries) != len(result.samples):
        raise ValueError("queries and samples must have the same length.")
    rows = []
    for sample, query in zip(result.samples, queries):
        rows.append(
            {
                "index": sample.index,
                "time_s": sample.time_s,
                "x_m": float(sample.position_world_xz_m[0]),
                "z_m": float(sample.position_world_xz_m[1]),
                "collision_free": sample.collision_free,
                "clearance_mm": (
                    None
                    if sample.terrain_clearance_m is None
                    else sample.terrain_clearance_m * 1e3
                ),
                "geometry_penetrations": len(query.geometry_penetrations),
                "vertical_face_collisions": len(query.collisions),
                "link_collisions": len(query.link_collisions),
                "contact_candidates": len(query.candidates),
                "statuses": ", ".join(status.value for status in query.statuses),
            }
        )
    return rows
