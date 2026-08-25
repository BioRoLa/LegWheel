"""Sampling-based F/L/R contact-candidate detection for the 2D prototype.

This Step-4 detector applies a geometric contact tolerance to existing physical
tyre samples. It deliberately does not select an active contact, evaluate
whole-leg collision, compute rim-edge margin, or decide ROLL/SWING.
"""

from __future__ import annotations

from dataclasses import dataclass

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
    """One rim sample touching or penetrating a rectangle vertical face.

    Vertical faces are blocking geometry, not support surfaces.  They are
    therefore reported separately from :class:`ContactCandidate2D`.
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

    def __post_init__(self) -> None:
        candidates = tuple(self.candidates)
        collisions = tuple(self.collisions)
        if not all(isinstance(item, ContactCandidate2D) for item in candidates):
            raise TypeError("candidates must contain ContactCandidate2D values.")
        if not all(isinstance(item, VerticalFaceCollision2D) for item in collisions):
            raise TypeError("collisions must contain VerticalFaceCollision2D values.")
        for name in ("contact_tolerance_m", "collision_tolerance_m"):
            value = getattr(self, name)
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative.")
        if self.evaluated_sample_count < 0 or self.excluded_non_contact_sample_count < 0:
            raise ValueError("sample counts must be non-negative.")
        object.__setattr__(self, "candidates", candidates)
        object.__setattr__(self, "collisions", collisions)

    @property
    def has_contact_candidates(self) -> bool:
        return bool(self.candidates)

    @property
    def has_collisions(self) -> bool:
        return bool(self.collisions)

    @property
    def candidate_surface_ids(self) -> tuple[str, ...]:
        return tuple(sorted({item.terrain_surface_id for item in self.candidates}))

    @property
    def collision_surface_ids(self) -> tuple[str, ...]:
        return tuple(sorted({item.terrain_surface_id for item in self.collisions}))


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


def detect_rectangle_vertical_face_collisions_2d(
    geometry: SampledLegGeometry2D,
    terrain: TerrainProfile,
    *,
    collision_tolerance_m: float = 1e-3,
) -> VerticalFaceCollisionResult2D:
    """Detect F/L/R samples touching or penetrating rectangle vertical faces.

    A point is a hit when its nearest point lies on a front/back face span and
    its Euclidean distance to that finite face segment is no larger than the
    tolerance.  This includes a shallow approach to the face and shallow
    penetration; points above/below the finite face span are not hits.
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
            if surface_gap.projection_within_span and surface_gap.euclidean_distance_m <= collision_tolerance_m:
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
    """Visualize geometry, support candidates, and blocking face collisions.

    Candidate points are red circles with a red segment to the queried terrain
    projection.  Vertical-face collisions are purple ``x`` markers.  When
    ``annotate`` is true, each marker includes rim, alpha, gap/margin, and
    surface information so one figure is sufficient for geometry debugging.
    """

    if not isinstance(result, ContactQueryResult2D):
        raise TypeError("result must be ContactQueryResult2D.")
    if ax is None:
        _, ax = plt.subplots(figsize=(12, 5))
    plot_leg_geometry_with_terrain_2d(geometry, terrain, ax=ax)

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
            edgecolors="#dc2626",
            linewidths=1.3,
            label="contact candidate" if candidate is result.candidates[0] else None,
            zorder=8,
        )
        if annotate:
            ax.annotate(
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

    ax.set_title(
        "Contact query: "
        f"{len(result.candidates)} candidates / {len(result.collisions)} collisions"
    )
    ax.text(
        0.01,
        0.99,
        "candidate surfaces: " + (", ".join(result.candidate_surface_ids) or "none")
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
    "ContactCandidate2D",
    "ContactDetectionResult2D",
    "ContactQueryResult2D",
    "VerticalFaceCollision2D",
    "VerticalFaceCollisionResult2D",
    "contact_candidate_rows",
    "detect_contact_candidates_2d",
    "detect_rectangle_vertical_face_collisions_2d",
    "plot_contact_candidates_2d",
    "plot_contact_query_2d",
    "query_contact",
    "vertical_face_collision_rows",
]
