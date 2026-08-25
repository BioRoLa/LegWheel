"""Point-to-terrain surface geometry queries for the Day 3--5 2D model.

This module reports objective geometry only.  It does not apply a contact
tolerance, create ContactCandidate values, select an active rim, or decide a
gait mode.

Signed-gap convention
---------------------
Every surface uses its outward normal.  A positive signed gap is on the free
space side of that surface, zero is on its supporting line, and negative is on
the solid half-space side::

    ground / top : z_point - z_surface
    front        : x_front - x_point   (outward normal = -x)
    back         : x_point - x_back    (outward normal = +x)

A negative per-face gap alone is not penetration.  Rectangle penetration is
reported only when the point is inside the complete rectangle solid.
"""

from __future__ import annotations

from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray

from .terrain_2d import (
    SurfaceOrientation,
    TerrainProfile,
    TerrainSurface2D,
    TerrainSurfaceKind,
    plot_terrain_profile_2d,
)


def _point_xz(value, field_name: str = "point_world_xz_m") -> NDArray[np.float64]:
    point = np.asarray(value, dtype=float)
    if point.shape != (2,):
        raise ValueError(f"{field_name} must have shape (2,); got {point.shape}.")
    if not np.all(np.isfinite(point)):
        raise ValueError(f"{field_name} must contain only finite values.")
    point = point.copy()
    point.setflags(write=False)
    return point


@dataclass(frozen=True)
class SurfaceGapResult2D:
    """Geometric relation between one world-frame point and one terrain surface."""

    surface_id: str
    surface_kind: TerrainSurfaceKind
    point_world_xz_m: NDArray[np.float64]
    nearest_point_world_xz_m: NDArray[np.float64]
    outward_normal_world_xz: NDArray[np.float64]
    signed_normal_gap_m: float
    euclidean_distance_m: float
    projection_within_span: bool
    is_occluded: bool
    is_relevant: bool
    point_inside_surface_solid: bool
    penetration_depth_m: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "surface_kind", TerrainSurfaceKind(self.surface_kind))
        object.__setattr__(self, "point_world_xz_m", _point_xz(self.point_world_xz_m))
        object.__setattr__(
            self,
            "nearest_point_world_xz_m",
            _point_xz(self.nearest_point_world_xz_m, "nearest_point_world_xz_m"),
        )
        normal = _point_xz(self.outward_normal_world_xz, "outward_normal_world_xz")
        if not np.isclose(np.linalg.norm(normal), 1.0):
            raise ValueError("outward_normal_world_xz must be a unit vector.")
        object.__setattr__(self, "outward_normal_world_xz", normal)
        for name in ("signed_normal_gap_m", "euclidean_distance_m", "penetration_depth_m"):
            if not np.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite.")
        if self.euclidean_distance_m < 0.0 or self.penetration_depth_m < 0.0:
            raise ValueError("distance and penetration depth must be non-negative.")
        if self.is_relevant != (self.projection_within_span and not self.is_occluded):
            raise ValueError("is_relevant must equal projection_within_span and not is_occluded.")
        if not self.point_inside_surface_solid and self.penetration_depth_m != 0.0:
            raise ValueError("penetration requires the point to be inside the terrain solid.")


@dataclass(frozen=True)
class PointTerrainQueryResult2D:
    """All surface-gap results for one point, retaining rather than classifying them."""

    point_world_xz_m: NDArray[np.float64]
    surface_gaps: tuple[SurfaceGapResult2D, ...]

    def __post_init__(self) -> None:
        point = _point_xz(self.point_world_xz_m)
        gaps = tuple(self.surface_gaps)
        if not gaps:
            raise ValueError("surface_gaps must not be empty.")
        if not all(isinstance(gap, SurfaceGapResult2D) for gap in gaps):
            raise TypeError("surface_gaps must contain SurfaceGapResult2D values.")
        if any(not np.array_equal(gap.point_world_xz_m, point) for gap in gaps):
            raise ValueError("all surface gaps must refer to the query point.")
        ids = [gap.surface_id for gap in gaps]
        if len(ids) != len(set(ids)):
            raise ValueError("surface gap IDs must be unique.")
        object.__setattr__(self, "point_world_xz_m", point)
        object.__setattr__(self, "surface_gaps", gaps)

    @property
    def relevant_surface_gaps(self) -> tuple[SurfaceGapResult2D, ...]:
        return tuple(gap for gap in self.surface_gaps if gap.is_relevant)

    @property
    def nearest_relevant_surface(self) -> SurfaceGapResult2D:
        return min(
            self.relevant_surface_gaps,
            key=lambda gap: (gap.euclidean_distance_m, gap.surface_id),
        )

    @property
    def penetrating_surface_gaps(self) -> tuple[SurfaceGapResult2D, ...]:
        return tuple(gap for gap in self.surface_gaps if gap.penetration_depth_m > 0.0)

    @property
    def point_inside_terrain(self) -> bool:
        return bool(self.penetrating_surface_gaps)

    def surface_gap_by_id(self, surface_id: str) -> SurfaceGapResult2D:
        for gap in self.surface_gaps:
            if gap.surface_id == surface_id:
                return gap
        raise KeyError(f"unknown terrain surface: {surface_id!r}")


def _obstacle_contains_point(point: NDArray[np.float64], terrain: TerrainProfile) -> bool:
    obstacle = terrain.obstacle
    if obstacle is None:
        return False
    x_m, z_m = point
    top_m = terrain.ground_height_m + obstacle.height_m
    return bool(
        obstacle.x_min_m < x_m < obstacle.x_max_m
        and terrain.ground_height_m < z_m < top_m
    )


def _ground_is_occluded(point: NDArray[np.float64], terrain: TerrainProfile) -> bool:
    obstacle = terrain.obstacle
    return bool(
        obstacle is not None
        and obstacle.x_min_m <= point[0] <= obstacle.x_max_m
        and point[1] >= terrain.ground_height_m
    )


def _query_surface(
    point: NDArray[np.float64],
    surface: TerrainSurface2D,
    terrain: TerrainProfile,
    span_tolerance_m: float,
) -> SurfaceGapResult2D:
    x_m, z_m = point
    if surface.orientation is SurfaceOrientation.HORIZONTAL:
        span_coordinate = x_m
        nearest = np.array(
            [np.clip(x_m, surface.span_min_m, surface.span_max_m), surface.position_m],
            dtype=float,
        )
        normal = np.array([0.0, 1.0])
        signed_gap = z_m - surface.position_m
    else:
        span_coordinate = z_m
        nearest = np.array(
            [surface.position_m, np.clip(z_m, surface.span_min_m, surface.span_max_m)],
            dtype=float,
        )
        if surface.kind is TerrainSurfaceKind.OBSTACLE_FRONT:
            normal = np.array([-1.0, 0.0])
            signed_gap = surface.position_m - x_m
        elif surface.kind is TerrainSurfaceKind.OBSTACLE_BACK:
            normal = np.array([1.0, 0.0])
            signed_gap = x_m - surface.position_m
        else:
            raise ValueError(f"unsupported vertical surface kind: {surface.kind.value}")

    within_span = bool(
        surface.span_min_m - span_tolerance_m
        <= span_coordinate
        <= surface.span_max_m + span_tolerance_m
    )
    is_occluded = surface.kind is TerrainSurfaceKind.GROUND and _ground_is_occluded(point, terrain)
    inside_ground = surface.kind is TerrainSurfaceKind.GROUND and z_m < terrain.ground_height_m
    inside_obstacle = (
        surface.kind is not TerrainSurfaceKind.GROUND
        and _obstacle_contains_point(point, terrain)
    )
    inside_solid = bool(inside_ground or inside_obstacle)
    penetration = max(0.0, -float(signed_gap)) if inside_solid else 0.0
    return SurfaceGapResult2D(
        surface_id=surface.surface_id,
        surface_kind=surface.kind,
        point_world_xz_m=point,
        nearest_point_world_xz_m=nearest,
        outward_normal_world_xz=normal,
        signed_normal_gap_m=float(signed_gap),
        euclidean_distance_m=float(np.linalg.norm(point - nearest)),
        projection_within_span=within_span,
        is_occluded=is_occluded,
        is_relevant=within_span and not is_occluded,
        point_inside_surface_solid=inside_solid,
        penetration_depth_m=penetration,
    )


def query_point_to_terrain_surfaces_2d(
    point_world_xz_m,
    terrain: TerrainProfile,
    *,
    span_tolerance_m: float = 0.0,
) -> PointTerrainQueryResult2D:
    """Query one point against every terrain surface without contact classification."""

    point = _point_xz(point_world_xz_m)
    if not isinstance(terrain, TerrainProfile):
        raise TypeError("terrain must be a 2D TerrainProfile.")
    if not np.isfinite(span_tolerance_m) or span_tolerance_m < 0.0:
        raise ValueError("span_tolerance_m must be finite and non-negative.")
    gaps = tuple(
        _query_surface(point, surface, terrain, span_tolerance_m)
        for surface in terrain.surfaces
    )
    return PointTerrainQueryResult2D(point, gaps)


def point_terrain_gap_rows(query: PointTerrainQueryResult2D) -> list[dict]:
    """Return all surface results in a table-friendly representation."""

    return [
        {
            "surface_id": gap.surface_id,
            "kind": gap.surface_kind.value,
            "signed_normal_gap_m": gap.signed_normal_gap_m,
            "euclidean_distance_m": gap.euclidean_distance_m,
            "projection_within_span": gap.projection_within_span,
            "is_occluded": gap.is_occluded,
            "is_relevant": gap.is_relevant,
            "inside_solid": gap.point_inside_surface_solid,
            "penetration_depth_m": gap.penetration_depth_m,
            "nearest_x_m": gap.nearest_point_world_xz_m[0],
            "nearest_z_m": gap.nearest_point_world_xz_m[1],
        }
        for gap in query.surface_gaps
    ]


def plot_point_terrain_query_2d(
    query: PointTerrainQueryResult2D,
    terrain: TerrainProfile,
    *,
    ax=None,
):
    """Visualize one point and its nearest relevant terrain-surface projection."""

    if ax is None:
        _, ax = plt.subplots(figsize=(7, 4))
    point = query.point_world_xz_m
    x_values = [point[0]]
    if terrain.obstacle is not None:
        x_values.extend([terrain.obstacle.x_min_m, terrain.obstacle.x_max_m])
    padding = max(0.1, 0.25 * max(max(x_values) - min(x_values), 0.1))
    plot_terrain_profile_2d(
        terrain,
        ax=ax,
        x_limits_m=(min(x_values) - padding, max(x_values) + padding),
    )
    nearest = query.nearest_relevant_surface
    projection = nearest.nearest_point_world_xz_m
    ax.plot(
        [point[0], projection[0]],
        [point[1], projection[1]],
        "--",
        color="#7c3aed",
        lw=1.5,
        label=f"nearest: {nearest.surface_id}",
        zorder=6,
    )
    ax.scatter(*projection, marker="x", s=65, color="#7c3aed", zorder=7)
    ax.scatter(
        *point,
        s=70,
        color="#dc2626" if query.point_inside_terrain else "#111827",
        zorder=8,
        label="query point",
    )
    ax.set_title(
        f"point={point.tolist()}, nearest={nearest.surface_id}, "
        f"distance={nearest.euclidean_distance_m:.3f} m"
    )
    ax.legend(fontsize=8, loc="best")
    return ax


__all__ = [
    "PointTerrainQueryResult2D",
    "SurfaceGapResult2D",
    "plot_point_terrain_query_2d",
    "point_terrain_gap_rows",
    "query_point_to_terrain_surfaces_2d",
]
