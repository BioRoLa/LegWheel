"""Sampling-based contact and fixed-pose collision query for the 2D prototype.

The original contact-candidate detector applies a geometric contact tolerance
to existing physical tyre samples.  The Day 6--7 extension keeps that
representation and adds status-preserving rim/link penetration records.  It
still does not select an active contact, compute a rolling path, or decide
ROLL/SWING.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray

from .geometry_2d import (
    SampledLegGeometry2D,
    legacy_rim_edge_margin_rad,
    plot_leg_geometry_with_terrain_2d,
)
from .terrain_2d import TerrainProfile, TerrainSurfaceKind
from .terrain_query_2d import query_point_to_terrain_surfaces_2d
from .types import RimId


CONTACT_REGION_TO_RIM = {
    "foot_rim": RimId.FOOT,
    "left_rim": RimId.LEFT,
    "right_rim": RimId.RIGHT,
}


class ContactStatus2D(str, Enum):
    """Terrain-aware status labels for one fixed single-leg configuration."""

    VALID_RIGHT_RIM_FACE_CONTACT = "VALID_RIGHT_RIM_FACE_CONTACT"
    VALID_RIGHT_RIM_TOP_CONTACT = "VALID_RIGHT_RIM_TOP_CONTACT"
    VALID_OTHER_RIM_CONTACT = "VALID_OTHER_RIM_CONTACT"
    INVALID_LINK_COLLISION = "INVALID_LINK_COLLISION"
    INVALID_GEOMETRY_PENETRATION = "INVALID_GEOMETRY_PENETRATION"
    NO_CONTACT = "NO_CONTACT"


def _immutable_point(value, field_name: str) -> NDArray[np.float64]:
    point = np.asarray(value, dtype=float)
    if point.shape != (2,) or not np.all(np.isfinite(point)):
        raise ValueError(f"{field_name} must be a finite world [x,z] point.")
    point = point.copy()
    point.setflags(write=False)
    return point


@dataclass(frozen=True)
class ContactCandidate2D:
    """One tyre sample geometrically close to one relevant terrain surface."""

    rim: RimId
    alpha_rad: float
    point_world_xz_m: NDArray[np.float64]
    terrain_point_world_xz_m: NDArray[np.float64]
    terrain_surface_id: str
    terrain_gap_m: float
    surface_distance_m: float
    edge_margin_rad: float
    raw_surface_name: str
    sample_index: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "rim", RimId(self.rim))
        object.__setattr__(
            self,
            "point_world_xz_m",
            _immutable_point(self.point_world_xz_m, "point_world_xz_m"),
        )
        object.__setattr__(
            self,
            "terrain_point_world_xz_m",
            _immutable_point(self.terrain_point_world_xz_m, "terrain_point_world_xz_m"),
        )
        for name in ("alpha_rad", "terrain_gap_m", "surface_distance_m", "edge_margin_rad"):
            if not np.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite.")
        if self.surface_distance_m < 0.0 or self.edge_margin_rad < 0.0:
            raise ValueError("surface distance and edge margin must be non-negative.")
        if not self.terrain_surface_id or not self.raw_surface_name:
            raise ValueError("surface identities must not be empty.")
        if self.sample_index < 0:
            raise ValueError("sample_index must be non-negative.")


@dataclass(frozen=True)
class ContactDetectionResult2D:
    """All sample/surface candidates retained for one leg configuration."""

    candidates: tuple[ContactCandidate2D, ...]
    contact_tolerance_m: float
    evaluated_sample_count: int
    excluded_non_contact_sample_count: int

    def __post_init__(self) -> None:
        candidates = tuple(self.candidates)
        if not all(isinstance(item, ContactCandidate2D) for item in candidates):
            raise TypeError("candidates must contain ContactCandidate2D values.")
        if not np.isfinite(self.contact_tolerance_m) or self.contact_tolerance_m < 0.0:
            raise ValueError("contact_tolerance_m must be finite and non-negative.")
        if self.evaluated_sample_count < 0 or self.excluded_non_contact_sample_count < 0:
            raise ValueError("sample counts must be non-negative.")
        object.__setattr__(self, "candidates", candidates)

    @property
    def has_contact_candidates(self) -> bool:
        return bool(self.candidates)

    @property
    def candidate_surface_ids(self) -> tuple[str, ...]:
        return tuple(sorted({candidate.terrain_surface_id for candidate in self.candidates}))


@dataclass(frozen=True)
class VerticalFaceCollision2D:
    """One rim sample penetrating a rectangle vertical face.

    A normal planned right-rim face touch is retained as a
    :class:`ContactCandidate2D` and is deliberately not returned here.  This
    record is reserved for invalid rim penetration.
    """

    rim: RimId
    alpha_rad: float
    point_world_xz_m: NDArray[np.float64]
    terrain_surface_id: str
    raw_surface_name: str
    signed_normal_gap_m: float
    surface_distance_m: float
    penetration_depth_m: float
    sample_index: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "rim", RimId(self.rim))
        object.__setattr__(
            self,
            "point_world_xz_m",
            _immutable_point(self.point_world_xz_m, "point_world_xz_m"),
        )
        for name in (
            "alpha_rad",
            "signed_normal_gap_m",
            "surface_distance_m",
            "penetration_depth_m",
        ):
            if not np.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite.")
        if self.surface_distance_m < 0.0 or self.penetration_depth_m < 0.0:
            raise ValueError("distance and penetration depth must be non-negative.")
        if not self.terrain_surface_id or not self.raw_surface_name:
            raise ValueError("surface identities must not be empty.")
        if self.sample_index < 0:
            raise ValueError("sample_index must be non-negative.")


@dataclass(frozen=True)
class VerticalFaceCollisionResult2D:
    """All blocking vertical-face hits for one leg configuration."""

    collisions: tuple[VerticalFaceCollision2D, ...]
    collision_tolerance_m: float
    evaluated_sample_count: int
    excluded_non_contact_sample_count: int

    def __post_init__(self) -> None:
        collisions = tuple(self.collisions)
        if not all(isinstance(item, VerticalFaceCollision2D) for item in collisions):
            raise TypeError("collisions must contain VerticalFaceCollision2D values.")
        if not np.isfinite(self.collision_tolerance_m) or self.collision_tolerance_m < 0.0:
            raise ValueError("collision_tolerance_m must be finite and non-negative.")
        if self.evaluated_sample_count < 0 or self.excluded_non_contact_sample_count < 0:
            raise ValueError("sample counts must be non-negative.")
        object.__setattr__(self, "collisions", collisions)

    @property
    def has_collision(self) -> bool:
        return bool(self.collisions)

    @property
    def collision_surface_ids(self) -> tuple[str, ...]:
        return tuple(sorted({item.terrain_surface_id for item in self.collisions}))


@dataclass(frozen=True)
class GeometryPenetration2D:
    """One physical rim sample found inside a terrain solid."""

    rim: RimId
    alpha_rad: float
    point_world_xz_m: NDArray[np.float64]
    terrain_surface_id: str
    raw_surface_name: str
    signed_normal_gap_m: float
    penetration_depth_m: float
    sample_index: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "rim", RimId(self.rim))
        object.__setattr__(
            self,
            "point_world_xz_m",
            _immutable_point(self.point_world_xz_m, "point_world_xz_m"),
        )
        for name in (
            "alpha_rad",
            "signed_normal_gap_m",
            "penetration_depth_m",
        ):
            if not np.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite.")
        if self.penetration_depth_m <= 0.0:
            raise ValueError("penetration_depth_m must be positive.")
        if not self.terrain_surface_id or not self.raw_surface_name:
            raise ValueError("surface identities must not be empty.")
        if self.sample_index < 0:
            raise ValueError("sample_index must be non-negative.")


def _immutable_segment(value, field_name: str) -> NDArray[np.float64]:
    segment = np.asarray(value, dtype=float)
    if segment.shape != (2, 2) or not np.all(np.isfinite(segment)):
        raise ValueError(f"{field_name} must be a finite shape-(2, 2) segment.")
    segment = segment.copy()
    segment.setflags(write=False)
    return segment


@dataclass(frozen=True)
class LinkCollision2D:
    """One existing linkage segment whose centerline enters the obstacle."""

    geometry_id: str
    point_world_xz_m: NDArray[np.float64]
    segment_world_xz_m: NDArray[np.float64]
    terrain_surface_id: str
    signed_normal_gap_m: float
    penetration_depth_m: float
    segment_parameter: float

    def __post_init__(self) -> None:
        if not self.geometry_id or not self.terrain_surface_id:
            raise ValueError("geometry_id and terrain_surface_id must not be empty.")
        object.__setattr__(
            self,
            "point_world_xz_m",
            _immutable_point(self.point_world_xz_m, "point_world_xz_m"),
        )
        object.__setattr__(
            self,
            "segment_world_xz_m",
            _immutable_segment(self.segment_world_xz_m, "segment_world_xz_m"),
        )
        for name in (
            "signed_normal_gap_m",
            "penetration_depth_m",
            "segment_parameter",
        ):
            if not np.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite.")
        if self.penetration_depth_m <= 0.0:
            raise ValueError("penetration_depth_m must be positive.")
        if not 0.0 <= self.segment_parameter <= 1.0:
            raise ValueError("segment_parameter must lie in [0, 1].")


@dataclass(frozen=True)
class ContactQueryResult2D:
    """Unified single-configuration result returned by :func:`query_contact`.

    The result deliberately keeps every candidate and every vertical-face hit;
    it does not select an active rim or assign ROLL/SWING.
    """

    candidates: tuple[ContactCandidate2D, ...]
    collisions: tuple[VerticalFaceCollision2D, ...]
    contact_tolerance_m: float
    collision_tolerance_m: float
    evaluated_sample_count: int
    excluded_non_contact_sample_count: int
    geometry_penetrations: tuple[GeometryPenetration2D, ...] = ()
    link_collisions: tuple[LinkCollision2D, ...] = ()
    statuses: tuple[ContactStatus2D, ...] = ()

    def __post_init__(self) -> None:
        candidates = tuple(self.candidates)
        collisions = tuple(self.collisions)
        if not all(isinstance(item, ContactCandidate2D) for item in candidates):
            raise TypeError("candidates must contain ContactCandidate2D values.")
        if not all(isinstance(item, VerticalFaceCollision2D) for item in collisions):
            raise TypeError("collisions must contain VerticalFaceCollision2D values.")
        geometry_penetrations = tuple(self.geometry_penetrations)
        link_collisions = tuple(self.link_collisions)
        statuses = tuple(ContactStatus2D(item) for item in self.statuses)
        if not all(isinstance(item, GeometryPenetration2D) for item in geometry_penetrations):
            raise TypeError("geometry_penetrations must contain GeometryPenetration2D values.")
        if not all(isinstance(item, LinkCollision2D) for item in link_collisions):
            raise TypeError("link_collisions must contain LinkCollision2D values.")
        for name in ("contact_tolerance_m", "collision_tolerance_m"):
            value = getattr(self, name)
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative.")
        if self.evaluated_sample_count < 0 or self.excluded_non_contact_sample_count < 0:
            raise ValueError("sample counts must be non-negative.")
        object.__setattr__(self, "candidates", candidates)
        object.__setattr__(self, "collisions", collisions)
        object.__setattr__(self, "geometry_penetrations", geometry_penetrations)
        object.__setattr__(self, "link_collisions", link_collisions)
        object.__setattr__(self, "statuses", statuses)

    @property
    def has_contact_candidates(self) -> bool:
        return bool(self.candidates)

    @property
    def has_collisions(self) -> bool:
        return bool(self.collisions or self.geometry_penetrations or self.link_collisions)

    @property
    def valid_contact(self) -> bool:
        return bool(self.candidates)

    @property
    def collision(self) -> bool:
        return self.has_collisions

    @property
    def primary_status(self) -> ContactStatus2D:
        return self.statuses[0] if self.statuses else ContactStatus2D.NO_CONTACT

    @property
    def status(self) -> ContactStatus2D:
        """Compatibility-friendly singular view of the first query status."""

        return self.primary_status

    @property
    def candidate_surface_ids(self) -> tuple[str, ...]:
        return tuple(sorted({item.terrain_surface_id for item in self.candidates}))

    @property
    def collision_surface_ids(self) -> tuple[str, ...]:
        surface_ids = {item.terrain_surface_id for item in self.collisions}
        surface_ids.update(item.terrain_surface_id for item in self.geometry_penetrations)
        surface_ids.update(item.terrain_surface_id for item in self.link_collisions)
        return tuple(sorted(surface_ids))


def detect_contact_candidates_2d(
    geometry: SampledLegGeometry2D,
    terrain: TerrainProfile,
    *,
    contact_tolerance_m: float = 1e-3,
) -> ContactDetectionResult2D:
    """Evaluate every F/L/R sample against every relevant terrain surface."""

    if not isinstance(geometry, SampledLegGeometry2D):
        raise TypeError("geometry must be SampledLegGeometry2D.")
    if not isinstance(terrain, TerrainProfile):
        raise TypeError("terrain must be a 2D TerrainProfile.")
    if not np.isfinite(contact_tolerance_m) or contact_tolerance_m < 0.0:
        raise ValueError("contact_tolerance_m must be finite and non-negative.")

    candidates = []
    excluded_count = 0
    points_world = geometry.points_world_xz_m
    for sample_index, (point, contact_region) in enumerate(
        zip(points_world, geometry.contact_regions)
    ):
        rim = CONTACT_REGION_TO_RIM.get(contact_region)
        if rim is None:
            excluded_count += 1
            continue
        point_query = query_point_to_terrain_surfaces_2d(
            point,
            terrain,
            span_tolerance_m=contact_tolerance_m,
        )
        for surface_gap in point_query.relevant_surface_gaps:
            if (
                abs(surface_gap.signed_normal_gap_m) <= contact_tolerance_m
                and surface_gap.euclidean_distance_m <= contact_tolerance_m
            ):
                candidates.append(
                    ContactCandidate2D(
                        rim=rim,
                        alpha_rad=float(geometry.alpha_rad[sample_index]),
                        point_world_xz_m=point,
                        terrain_point_world_xz_m=surface_gap.nearest_point_world_xz_m,
                        terrain_surface_id=surface_gap.surface_id,
                        terrain_gap_m=surface_gap.signed_normal_gap_m,
                        surface_distance_m=surface_gap.euclidean_distance_m,
                        edge_margin_rad=legacy_rim_edge_margin_rad(
                            geometry.surface_names[sample_index],
                            float(geometry.alpha_rad[sample_index]),
                        ),
                        raw_surface_name=geometry.surface_names[sample_index],
                        sample_index=sample_index,
                    )
                )
    return ContactDetectionResult2D(
        candidates=tuple(candidates),
        contact_tolerance_m=float(contact_tolerance_m),
        evaluated_sample_count=len(points_world) - excluded_count,
        excluded_non_contact_sample_count=excluded_count,
    )


def detect_geometry_penetrations_2d(
    geometry: SampledLegGeometry2D,
    terrain: TerrainProfile,
) -> tuple[GeometryPenetration2D, ...]:
    """Report physical rim samples that are inside ground or obstacle solid."""

    if not isinstance(geometry, SampledLegGeometry2D):
        raise TypeError("geometry must be SampledLegGeometry2D.")
    if not isinstance(terrain, TerrainProfile):
        raise TypeError("terrain must be a 2D TerrainProfile.")

    penetrations = []
    points_world = geometry.points_world_xz_m
    for sample_index, (point, contact_region) in enumerate(
        zip(points_world, geometry.contact_regions)
    ):
        rim = CONTACT_REGION_TO_RIM.get(contact_region)
        if rim is None:
            continue
        point_query = query_point_to_terrain_surfaces_2d(point, terrain)
        if point_query.penetrating_surface_gaps:
            surface_gap = min(
                point_query.penetrating_surface_gaps,
                key=lambda item: (item.penetration_depth_m, item.surface_id),
            )
            penetrations.append(
                GeometryPenetration2D(
                    rim=rim,
                    alpha_rad=float(geometry.alpha_rad[sample_index]),
                    point_world_xz_m=point,
                    terrain_surface_id=surface_gap.surface_id,
                    raw_surface_name=geometry.surface_names[sample_index],
                    signed_normal_gap_m=surface_gap.signed_normal_gap_m,
                    penetration_depth_m=surface_gap.penetration_depth_m,
                    sample_index=sample_index,
                )
            )
    return tuple(penetrations)


def _segment_obstacle_interior_hit(
    segment_world_xz_m: NDArray[np.float64],
    terrain: TerrainProfile,
) -> tuple[NDArray[np.float64], float, str] | None:
    """Return an interior hit, segment parameter, and likely entry surface."""

    obstacle = terrain.obstacle
    if obstacle is None:
        return None
    p0, p1 = segment_world_xz_m
    direction = p1 - p0
    lower, upper = 0.0, 1.0
    for coordinate, minimum, maximum in (
        (0, obstacle.x_min_m, obstacle.x_max_m),
        (
            1,
            terrain.ground_height_m,
            terrain.ground_height_m + obstacle.height_m,
        ),
    ):
        value = direction[coordinate]
        start = p0[coordinate]
        if abs(value) <= 1e-15:
            if start < minimum or start > maximum:
                return None
            continue
        t0 = (minimum - start) / value
        t1 = (maximum - start) / value
        if t0 > t1:
            t0, t1 = t1, t0
        lower = max(lower, t0)
        upper = min(upper, t1)
        if upper <= lower + 1e-12:
            return None

    hit_parameter = (lower + upper) / 2.0
    point = p0 + hit_parameter * direction
    epsilon = 1e-12
    if not (
        obstacle.x_min_m + epsilon < point[0] < obstacle.x_max_m - epsilon
        and terrain.ground_height_m + epsilon < point[1]
        < terrain.ground_height_m + obstacle.height_m - epsilon
    ):
        return None

    entry_point = p0 + lower * direction
    if p0[0] <= obstacle.x_min_m and p1[0] > obstacle.x_min_m:
        entry_surface = "front"
    elif p0[0] >= obstacle.x_max_m and p1[0] < obstacle.x_max_m:
        entry_surface = "back"
    elif p0[1] <= terrain.ground_height_m and p1[1] > terrain.ground_height_m:
        entry_surface = "ground"
    elif p0[1] >= terrain.ground_height_m + obstacle.height_m and p1[1] < terrain.ground_height_m + obstacle.height_m:
        entry_surface = "top"
    else:
        distances = {
            "front": abs(entry_point[0] - obstacle.x_min_m),
            "back": abs(entry_point[0] - obstacle.x_max_m),
            "ground": abs(entry_point[1] - terrain.ground_height_m),
            "top": abs(entry_point[1] - (terrain.ground_height_m + obstacle.height_m)),
        }
        entry_surface = min(distances, key=distances.get)
    return point, float(hit_parameter), entry_surface


def detect_link_collisions_2d(
    geometry: SampledLegGeometry2D,
    terrain: TerrainProfile,
) -> tuple[LinkCollision2D, ...]:
    """Detect centerline intersections of existing links with the obstacle."""

    if not isinstance(geometry, SampledLegGeometry2D):
        raise TypeError("geometry must be SampledLegGeometry2D.")
    if not isinstance(terrain, TerrainProfile):
        raise TypeError("terrain must be a 2D TerrainProfile.")
    obstacle = terrain.obstacle
    if obstacle is None:
        return ()

    surface_by_entry = {
        "top": obstacle.surface_ids[0],
        "front": obstacle.surface_ids[1],
        "back": obstacle.surface_ids[2],
        "ground": terrain.ground_surface_id,
    }
    collisions = []
    for link, segment_world in zip(
        geometry.link_segments_hip_xz_m,
        geometry.link_segments_world_xz_m,
    ):
        hit = _segment_obstacle_interior_hit(segment_world, terrain)
        if hit is None:
            continue
        point, segment_parameter, entry_surface = hit
        if entry_surface == "front":
            penetration_depth = point[0] - obstacle.x_min_m
        elif entry_surface == "back":
            penetration_depth = obstacle.x_max_m - point[0]
        elif entry_surface == "ground":
            penetration_depth = point[1] - terrain.ground_height_m
        else:
            penetration_depth = terrain.ground_height_m + obstacle.height_m - point[1]
        penetration_depth = float(max(penetration_depth, 1e-12))
        collisions.append(
            LinkCollision2D(
                geometry_id=link.segment_id,
                point_world_xz_m=point,
                segment_world_xz_m=segment_world,
                terrain_surface_id=surface_by_entry[entry_surface],
                signed_normal_gap_m=-penetration_depth,
                penetration_depth_m=penetration_depth,
                segment_parameter=segment_parameter,
            )
        )
    return tuple(collisions)


def candidate_status_2d(
    candidate: ContactCandidate2D,
    terrain: TerrainProfile,
) -> ContactStatus2D:
    """Return the valid-contact status represented by one candidate."""

    surface_kind = terrain.surface_by_id(candidate.terrain_surface_id).kind
    if candidate.rim is RimId.RIGHT and surface_kind in {
        TerrainSurfaceKind.OBSTACLE_FRONT,
        TerrainSurfaceKind.OBSTACLE_BACK,
    }:
        return ContactStatus2D.VALID_RIGHT_RIM_FACE_CONTACT
    if candidate.rim is RimId.RIGHT and surface_kind is TerrainSurfaceKind.OBSTACLE_TOP:
        return ContactStatus2D.VALID_RIGHT_RIM_TOP_CONTACT
    return ContactStatus2D.VALID_OTHER_RIM_CONTACT


def _query_statuses(
    candidates: tuple[ContactCandidate2D, ...],
    terrain: TerrainProfile,
    face_collisions: tuple[VerticalFaceCollision2D, ...],
    geometry_penetrations: tuple[GeometryPenetration2D, ...],
    link_collisions: tuple[LinkCollision2D, ...],
) -> tuple[ContactStatus2D, ...]:
    statuses = []
    for candidate in candidates:
        status = candidate_status_2d(candidate, terrain)
        if status not in statuses:
            statuses.append(status)
    if link_collisions:
        statuses.append(ContactStatus2D.INVALID_LINK_COLLISION)
    if face_collisions or geometry_penetrations:
        statuses.append(ContactStatus2D.INVALID_GEOMETRY_PENETRATION)
    if not candidates and not face_collisions and not geometry_penetrations and not link_collisions:
        statuses.append(ContactStatus2D.NO_CONTACT)
    return tuple(statuses)


def detect_rectangle_vertical_face_collisions_2d(
    geometry: SampledLegGeometry2D,
    terrain: TerrainProfile,
    *,
    collision_tolerance_m: float = 1e-3,
) -> VerticalFaceCollisionResult2D:
    """Detect F/L/R samples penetrating rectangle vertical faces.

    A normal face touch is a support candidate, including the planned right
    rim face contact.  Only samples with positive terrain penetration are
    returned here, so a planned right-rim touch is not mislabeled as a
    collision.
    """

    if not isinstance(geometry, SampledLegGeometry2D):
        raise TypeError("geometry must be SampledLegGeometry2D.")
    if not isinstance(terrain, TerrainProfile):
        raise TypeError("terrain must be a 2D TerrainProfile.")
    if not np.isfinite(collision_tolerance_m) or collision_tolerance_m < 0.0:
        raise ValueError("collision_tolerance_m must be finite and non-negative.")

    collisions = []
    excluded_count = 0
    points_world = geometry.points_world_xz_m
    for sample_index, (point, contact_region) in enumerate(
        zip(points_world, geometry.contact_regions)
    ):
        rim = CONTACT_REGION_TO_RIM.get(contact_region)
        if rim is None:
            excluded_count += 1
            continue
        # The face's finite z span is strict; tolerance applies to the
        # normal direction, so points just below the ground or above the top
        # edge are not mislabeled as vertical-face hits.
        point_query = query_point_to_terrain_surfaces_2d(point, terrain)
        for surface_gap in point_query.surface_gaps:
            if surface_gap.surface_kind not in {
                TerrainSurfaceKind.OBSTACLE_FRONT,
                TerrainSurfaceKind.OBSTACLE_BACK,
            }:
                continue
            if (
                surface_gap.projection_within_span
                and surface_gap.euclidean_distance_m <= collision_tolerance_m
                and surface_gap.penetration_depth_m > 0.0
            ):
                collisions.append(
                    VerticalFaceCollision2D(
                        rim=rim,
                        alpha_rad=float(geometry.alpha_rad[sample_index]),
                        point_world_xz_m=point,
                        terrain_surface_id=surface_gap.surface_id,
                        raw_surface_name=geometry.surface_names[sample_index],
                        signed_normal_gap_m=surface_gap.signed_normal_gap_m,
                        surface_distance_m=surface_gap.euclidean_distance_m,
                        penetration_depth_m=surface_gap.penetration_depth_m,
                        sample_index=sample_index,
                    )
                )
    return VerticalFaceCollisionResult2D(
        collisions=tuple(collisions),
        collision_tolerance_m=float(collision_tolerance_m),
        evaluated_sample_count=len(points_world) - excluded_count,
        excluded_non_contact_sample_count=excluded_count,
    )


def query_contact(
    geometry: SampledLegGeometry2D,
    terrain: TerrainProfile,
    *,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
) -> ContactQueryResult2D:
    """Query support candidates and rectangle-face collisions together.

    This is a single-configuration geometry query.  ``geometry`` is already
    expressed in the world frame through its hip pose; gait mode selection,
    active-contact selection, and rolling-path feasibility remain separate.
    """

    contact_result = detect_contact_candidates_2d(
        geometry,
        terrain,
        contact_tolerance_m=contact_tolerance_m,
    )
    collision_result = detect_rectangle_vertical_face_collisions_2d(
        geometry,
        terrain,
        collision_tolerance_m=collision_tolerance_m,
    )
    all_geometry_penetrations = detect_geometry_penetrations_2d(geometry, terrain)
    face_collision_keys = {
        (item.sample_index, item.terrain_surface_id)
        for item in collision_result.collisions
    }
    geometry_penetrations = tuple(
        item
        for item in all_geometry_penetrations
        if (item.sample_index, item.terrain_surface_id) not in face_collision_keys
    )
    link_collisions = detect_link_collisions_2d(geometry, terrain)
    if (
        contact_result.evaluated_sample_count
        != collision_result.evaluated_sample_count
        or contact_result.excluded_non_contact_sample_count
        != collision_result.excluded_non_contact_sample_count
    ):
        raise RuntimeError("contact and collision detectors evaluated different samples.")
    return ContactQueryResult2D(
        candidates=contact_result.candidates,
        collisions=collision_result.collisions,
        contact_tolerance_m=contact_result.contact_tolerance_m,
        collision_tolerance_m=collision_result.collision_tolerance_m,
        evaluated_sample_count=contact_result.evaluated_sample_count,
        excluded_non_contact_sample_count=contact_result.excluded_non_contact_sample_count,
        geometry_penetrations=geometry_penetrations,
        link_collisions=link_collisions,
        statuses=_query_statuses(
            contact_result.candidates,
            terrain,
            collision_result.collisions,
            geometry_penetrations,
            link_collisions,
        ),
    )


def vertical_face_collision_rows(result: VerticalFaceCollisionResult2D) -> list[dict]:
    """Return vertical-face hits in a table-friendly representation."""

    return [
        {
            "rim": item.rim.value,
            "alpha_rad": item.alpha_rad,
            "alpha_deg": np.rad2deg(item.alpha_rad),
            "point_x_m": item.point_world_xz_m[0],
            "point_z_m": item.point_world_xz_m[1],
            "terrain_surface_id": item.terrain_surface_id,
            "signed_normal_gap_m": item.signed_normal_gap_m,
            "surface_distance_m": item.surface_distance_m,
            "penetration_depth_m": item.penetration_depth_m,
            "raw_surface_name": item.raw_surface_name,
            "sample_index": item.sample_index,
        }
        for item in result.collisions
    ]


def contact_candidate_rows(result: ContactDetectionResult2D) -> list[dict]:
    """Return candidates in a table-friendly representation."""

    return [
        {
            "rim": candidate.rim.value,
            "alpha_rad": candidate.alpha_rad,
            "alpha_deg": np.rad2deg(candidate.alpha_rad),
            "point_x_m": candidate.point_world_xz_m[0],
            "point_z_m": candidate.point_world_xz_m[1],
            "terrain_surface_id": candidate.terrain_surface_id,
            "terrain_gap_m": candidate.terrain_gap_m,
            "surface_distance_m": candidate.surface_distance_m,
            "edge_margin_rad": candidate.edge_margin_rad,
            "edge_margin_deg": np.rad2deg(candidate.edge_margin_rad),
            "raw_surface_name": candidate.raw_surface_name,
            "sample_index": candidate.sample_index,
        }
        for candidate in result.candidates
    ]


def geometry_penetration_rows(result: ContactQueryResult2D) -> list[dict]:
    """Return invalid rim-penetration records in table form."""

    return [
        {
            "rim": item.rim.value,
            "alpha_rad": item.alpha_rad,
            "alpha_deg": np.rad2deg(item.alpha_rad),
            "point_x_m": item.point_world_xz_m[0],
            "point_z_m": item.point_world_xz_m[1],
            "terrain_surface_id": item.terrain_surface_id,
            "signed_normal_gap_m": item.signed_normal_gap_m,
            "penetration_depth_m": item.penetration_depth_m,
            "raw_surface_name": item.raw_surface_name,
            "sample_index": item.sample_index,
        }
        for item in result.geometry_penetrations
    ]


def link_collision_rows(result: ContactQueryResult2D) -> list[dict]:
    """Return invalid linkage-centerline collision records in table form."""

    return [
        {
            "geometry_id": item.geometry_id,
            "point_x_m": item.point_world_xz_m[0],
            "point_z_m": item.point_world_xz_m[1],
            "terrain_surface_id": item.terrain_surface_id,
            "signed_normal_gap_m": item.signed_normal_gap_m,
            "penetration_depth_m": item.penetration_depth_m,
            "segment_parameter": item.segment_parameter,
        }
        for item in result.link_collisions
    ]


def plot_contact_candidates_2d(
    geometry: SampledLegGeometry2D,
    terrain: TerrainProfile,
    result: ContactDetectionResult2D,
    *,
    ax=None,
):
    """Highlight all retained sample/surface pairs without selecting one active contact."""

    if ax is None:
        _, ax = plt.subplots(figsize=(11, 5))
    plot_leg_geometry_with_terrain_2d(geometry, terrain, ax=ax)
    if result.candidates:
        points = np.array([candidate.point_world_xz_m for candidate in result.candidates])
        terrain_points = np.array(
            [candidate.terrain_point_world_xz_m for candidate in result.candidates]
        )
        for point, terrain_point in zip(points, terrain_points):
            ax.plot(
                [point[0], terrain_point[0]],
                [point[1], terrain_point[1]],
                color="#dc2626",
                lw=0.8,
                alpha=0.55,
                zorder=7,
            )
        ax.scatter(
            points[:, 0],
            points[:, 1],
            marker="o",
            s=32,
            facecolors="none",
            edgecolors="#dc2626",
            linewidths=1.2,
            label="contact candidates",
            zorder=8,
        )
    ax.set_title(
        f"Step 4: {len(result.candidates)} sampled contact candidates "
        f"(tolerance={result.contact_tolerance_m * 1e3:.2f} mm)"
    )
    ax.legend(fontsize=8, loc="best")
    return ax


def plot_contact_query_2d(
    geometry: SampledLegGeometry2D,
    terrain: TerrainProfile,
    result: ContactQueryResult2D,
    *,
    ax=None,
    annotate: bool = True,
):
    """Visualize geometry, valid candidates, and invalid collision geometry.

    Candidate points are green circles with a segment to the queried terrain
    projection.  Invalid rim penetrations are purple ``x`` markers and link
    collisions are red highlighted segments.  A planned right-rim face touch
    therefore appears as a valid candidate, not a collision.
    """

    if not isinstance(result, ContactQueryResult2D):
        raise TypeError("result must be ContactQueryResult2D.")
    if ax is None:
        _, ax = plt.subplots(figsize=(12, 7))
    plot_leg_geometry_with_terrain_2d(geometry, terrain, ax=ax)

    # Repeat the hip marker at query-plot scale so it remains readable in a
    # notebook figure that also contains candidate/collision annotations.
    hip = geometry.hip_pose.position_world_xz_m
    ax.scatter(
        hip[0], hip[1], marker="X", s=150, color="black",
        edgecolors="white", linewidths=1.0, zorder=14,
        label="hip origin H",
    )
    ax.annotate(
        "hip origin H", hip, xytext=(8, 10), textcoords="offset points",
        fontsize=10, fontweight="bold", color="black",
        bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "0.7", "pad": 2},
        zorder=15,
    )

    for candidate in result.candidates:
        point = candidate.point_world_xz_m
        terrain_point = candidate.terrain_point_world_xz_m
        ax.plot(
            [point[0], terrain_point[0]],
            [point[1], terrain_point[1]],
            color="#dc2626",
            lw=0.9,
            alpha=0.65,
            zorder=7,
        )
        ax.scatter(
            point[0],
            point[1],
            marker="o",
            s=42,
            facecolors="none",
            edgecolors="#16a34a",
            linewidths=1.3,
            label="valid contact" if candidate is result.candidates[0] else None,
            zorder=8,
        )
        if annotate:
            ax.annotate(
                f"{candidate_status_2d(candidate, terrain).value}\n"
                f"{candidate.rim.value} α={np.rad2deg(candidate.alpha_rad):.1f}°\n"
                f"gap={candidate.terrain_gap_m * 1e3:.2f} mm, "
                f"edge={np.rad2deg(candidate.edge_margin_rad):.1f}°\n"
                f"{candidate.terrain_surface_id}",
                point,
                xytext=(4, 5),
                textcoords="offset points",
                fontsize=6.5,
                color="#991b1b",
                zorder=11,
            )

    for collision in result.collisions:
        point = collision.point_world_xz_m
        ax.scatter(
            point[0],
            point[1],
            marker="x",
            s=70,
            color="#7c3aed",
            linewidths=2.0,
            label="vertical-face collision"
            if collision is result.collisions[0]
            else None,
            zorder=10,
        )
        if annotate:
            ax.annotate(
                f"{collision.rim.value} α={np.rad2deg(collision.alpha_rad):.1f}°\n"
                f"pen={collision.penetration_depth_m * 1e3:.2f} mm\n"
                f"{collision.terrain_surface_id}",
                point,
                xytext=(4, -22),
                textcoords="offset points",
                fontsize=6.5,
                color="#5b21b6",
                zorder=11,
            )

    for penetration in result.geometry_penetrations:
        point = penetration.point_world_xz_m
        ax.scatter(
            point[0],
            point[1],
            marker="x",
            s=78,
            color="#7c3aed",
            linewidths=2.0,
            label="invalid geometry penetration"
            if penetration is result.geometry_penetrations[0]
            else None,
            zorder=10,
        )
        if annotate:
            ax.annotate(
                f"{ContactStatus2D.INVALID_GEOMETRY_PENETRATION.value}\n"
                f"{penetration.rim.value}, pen={penetration.penetration_depth_m * 1e3:.2f} mm\n"
                f"{penetration.terrain_surface_id}",
                point,
                xytext=(4, -22),
                textcoords="offset points",
                fontsize=6.5,
                color="#5b21b6",
                zorder=11,
            )

    for link_collision in result.link_collisions:
        segment = link_collision.segment_world_xz_m
        point = link_collision.point_world_xz_m
        ax.plot(
            segment[:, 0],
            segment[:, 1],
            color="#dc2626",
            linewidth=4.0,
            alpha=0.85,
            label="invalid link collision"
            if link_collision is result.link_collisions[0]
            else None,
            zorder=9,
        )
        ax.scatter(
            point[0],
            point[1],
            marker="X",
            s=70,
            color="#dc2626",
            edgecolor="white",
            linewidths=0.7,
            zorder=11,
        )
        if annotate:
            ax.annotate(
                f"{ContactStatus2D.INVALID_LINK_COLLISION.value}\n"
                f"{link_collision.geometry_id}, pen={link_collision.penetration_depth_m * 1e3:.2f} mm\n"
                f"{link_collision.terrain_surface_id}",
                point,
                xytext=(4, 6),
                textcoords="offset points",
                fontsize=6.5,
                color="#991b1b",
                zorder=11,
            )

    ax.set_title(
        "Contact query: "
        f"{len(result.candidates)} valid candidates / "
        f"{len(result.collisions) + len(result.geometry_penetrations) + len(result.link_collisions)} collisions"
    )
    ax.text(
        0.01,
        0.99,
        "status: " + ", ".join(status.value for status in result.statuses)
        + "\nvalid surfaces: "
        + (", ".join(result.candidate_surface_ids) or "none")
        + "\ncollision surfaces: "
        + (", ".join(result.collision_surface_ids) or "none"),
        transform=ax.transAxes,
        va="top",
        fontsize=8,
        bbox={"facecolor": "white", "alpha": 0.78, "edgecolor": "0.75"},
        zorder=12,
    )
    ax.legend(fontsize=8, loc="best")
    return ax


__all__ = [
    "CONTACT_REGION_TO_RIM",
    "ContactStatus2D",
    "ContactCandidate2D",
    "ContactDetectionResult2D",
    "ContactQueryResult2D",
    "GeometryPenetration2D",
    "LinkCollision2D",
    "VerticalFaceCollision2D",
    "VerticalFaceCollisionResult2D",
    "contact_candidate_rows",
    "candidate_status_2d",
    "detect_contact_candidates_2d",
    "detect_geometry_penetrations_2d",
    "detect_link_collisions_2d",
    "detect_rectangle_vertical_face_collisions_2d",
    "geometry_penetration_rows",
    "link_collision_rows",
    "plot_contact_candidates_2d",
    "plot_contact_query_2d",
    "query_contact",
    "vertical_face_collision_rows",
]
