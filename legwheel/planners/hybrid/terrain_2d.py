"""Two-dimensional terrain primitives for the Day 3--5 contact prototype.

The coordinate convention is ``+x`` forward and ``+z`` upward.  This module
only represents geometry and stable surface identities; it deliberately does
not contain rim, contact, collision, or gait-mode decisions.

The unqualified class names in this module are ``RectangleObstacle`` and
``TerrainProfile``.  The hybrid package root exports them as
``RectangleObstacle2D`` and ``TerrainProfile2D`` so they cannot be confused
with the Day 1--2 three-dimensional planning contracts.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Iterable

import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray


class TerrainSurfaceKind(str, Enum):
    """Geometric role of one axis-aligned terrain surface."""

    GROUND = "ground"
    OBSTACLE_TOP = "obstacle_top"
    OBSTACLE_FRONT = "obstacle_front"
    OBSTACLE_BACK = "obstacle_back"


class SurfaceOrientation(str, Enum):
    """Orientation used to interpret a surface position and span."""

    HORIZONTAL = "horizontal"
    VERTICAL = "vertical"


@dataclass(frozen=True)
class TerrainSurface2D:
    """One horizontal or vertical terrain surface in the world ``x-z`` plane.

    For a horizontal surface, ``position_m`` is its z coordinate and the span
    is an x interval.  For a vertical surface, ``position_m`` is its x
    coordinate and the span is a z interval.  Infinite spans are allowed for
    the ground surface.
    """

    surface_id: str
    kind: TerrainSurfaceKind
    orientation: SurfaceOrientation
    position_m: float
    span_min_m: float
    span_max_m: float
    obstacle_id: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "kind", TerrainSurfaceKind(self.kind))
        object.__setattr__(self, "orientation", SurfaceOrientation(self.orientation))
        if not self.surface_id:
            raise ValueError("surface_id must not be empty.")
        if not np.isfinite(self.position_m):
            raise ValueError("position_m must be finite.")
        if np.isnan(self.span_min_m) or np.isnan(self.span_max_m):
            raise ValueError("surface span must not contain NaN.")
        if self.span_max_m <= self.span_min_m:
            raise ValueError("span_max_m must be greater than span_min_m.")
        if self.kind is TerrainSurfaceKind.GROUND:
            if self.orientation is not SurfaceOrientation.HORIZONTAL:
                raise ValueError("ground must be horizontal.")
            if self.obstacle_id is not None:
                raise ValueError("ground must not have an obstacle_id.")
        elif not self.obstacle_id:
            raise ValueError("obstacle surfaces require an obstacle_id.")

    @property
    def endpoints_xz_m(self) -> NDArray[np.float64]:
        """Return the two endpoints as ``[[x0, z0], [x1, z1]]``.

        Ground endpoints are infinite by design.  A visualization or finite
        query window should clip them without changing the terrain model.
        """

        if self.orientation is SurfaceOrientation.HORIZONTAL:
            endpoints = np.array(
                [[self.span_min_m, self.position_m], [self.span_max_m, self.position_m]],
                dtype=float,
            )
        else:
            endpoints = np.array(
                [[self.position_m, self.span_min_m], [self.position_m, self.span_max_m]],
                dtype=float,
            )
        endpoints.setflags(write=False)
        return endpoints


@dataclass(frozen=True)
class RectangleObstacle:
    """Axis-aligned rectangle resting on the terrain ground surface."""

    obstacle_id: str
    x_min_m: float
    x_max_m: float
    height_m: float

    def __post_init__(self) -> None:
        if not self.obstacle_id:
            raise ValueError("obstacle_id must not be empty.")
        for name in ("x_min_m", "x_max_m", "height_m"):
            if not np.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite.")
        if self.x_max_m <= self.x_min_m:
            raise ValueError("x_max_m must be greater than x_min_m.")
        if self.height_m <= 0.0:
            raise ValueError("height_m must be positive.")

    @property
    def width_m(self) -> float:
        return float(self.x_max_m - self.x_min_m)

    @property
    def surface_ids(self) -> tuple[str, str, str]:
        return (
            f"{self.obstacle_id}_top",
            f"{self.obstacle_id}_front",
            f"{self.obstacle_id}_back",
        )

    def surfaces(self, ground_height_m: float = 0.0) -> tuple[TerrainSurface2D, ...]:
        """Return top/front/back surfaces; no hidden bottom face is created."""

        if not np.isfinite(ground_height_m):
            raise ValueError("ground_height_m must be finite.")
        top_id, front_id, back_id = self.surface_ids
        top_height = ground_height_m + self.height_m
        return (
            TerrainSurface2D(
                top_id,
                TerrainSurfaceKind.OBSTACLE_TOP,
                SurfaceOrientation.HORIZONTAL,
                top_height,
                self.x_min_m,
                self.x_max_m,
                self.obstacle_id,
            ),
            TerrainSurface2D(
                front_id,
                TerrainSurfaceKind.OBSTACLE_FRONT,
                SurfaceOrientation.VERTICAL,
                self.x_min_m,
                ground_height_m,
                top_height,
                self.obstacle_id,
            ),
            TerrainSurface2D(
                back_id,
                TerrainSurfaceKind.OBSTACLE_BACK,
                SurfaceOrientation.VERTICAL,
                self.x_max_m,
                ground_height_m,
                top_height,
                self.obstacle_id,
            ),
        )


@dataclass(frozen=True)
class TerrainProfile:
    """Flat ground plus at most one rectangular obstacle in the world x-z plane."""

    ground_height_m: float = 0.0
    obstacles: tuple[RectangleObstacle, ...] = field(default_factory=tuple)
    ground_surface_id: str = "ground"

    def __post_init__(self) -> None:
        if not np.isfinite(self.ground_height_m):
            raise ValueError("ground_height_m must be finite.")
        if not self.ground_surface_id:
            raise ValueError("ground_surface_id must not be empty.")
        obstacles = tuple(self.obstacles)
        if not all(isinstance(item, RectangleObstacle) for item in obstacles):
            raise TypeError("obstacles must contain only 2D RectangleObstacle values.")
        if len(obstacles) > 1:
            raise ValueError("Day 3--5 TerrainProfile supports at most one rectangle.")
        surface_ids = [self.ground_surface_id]
        for obstacle in obstacles:
            surface_ids.extend(obstacle.surface_ids)
        if len(surface_ids) != len(set(surface_ids)):
            raise ValueError("terrain surface IDs must be unique.")
        object.__setattr__(self, "obstacles", obstacles)

    @property
    def obstacle(self) -> RectangleObstacle | None:
        return self.obstacles[0] if self.obstacles else None

    @property
    def surfaces(self) -> tuple[TerrainSurface2D, ...]:
        ground = TerrainSurface2D(
            self.ground_surface_id,
            TerrainSurfaceKind.GROUND,
            SurfaceOrientation.HORIZONTAL,
            self.ground_height_m,
            -np.inf,
            np.inf,
        )
        obstacle_surfaces = () if self.obstacle is None else self.obstacle.surfaces(self.ground_height_m)
        return (ground, *obstacle_surfaces)

    @property
    def surface_ids(self) -> tuple[str, ...]:
        return tuple(surface.surface_id for surface in self.surfaces)

    def surface_by_id(self, surface_id: str) -> TerrainSurface2D:
        """Return a surface by stable identity, or raise ``KeyError``."""

        for surface in self.surfaces:
            if surface.surface_id == surface_id:
                return surface
        raise KeyError(f"unknown terrain surface: {surface_id!r}")


def terrain_surface_rows(terrain: TerrainProfile) -> list[dict]:
    """Return a table-friendly description without performing distance queries."""

    return [
        {
            "surface_id": surface.surface_id,
            "kind": surface.kind.value,
            "orientation": surface.orientation.value,
            "position_m": surface.position_m,
            "span_min_m": surface.span_min_m,
            "span_max_m": surface.span_max_m,
        }
        for surface in terrain.surfaces
    ]


def plot_terrain_profile_2d(
    terrain: TerrainProfile,
    *,
    ax=None,
    x_limits_m: Iterable[float] | None = None,
):
    """Draw the Step-1 terrain geometry and label every surface identity."""

    if ax is None:
        _, ax = plt.subplots(figsize=(10, 3.5))

    if x_limits_m is None:
        if terrain.obstacle is None:
            x_min, x_max = -0.5, 0.5
        else:
            padding = max(0.15, terrain.obstacle.width_m)
            x_min = terrain.obstacle.x_min_m - padding
            x_max = terrain.obstacle.x_max_m + padding
    else:
        limits = np.asarray(tuple(x_limits_m), dtype=float)
        if limits.shape != (2,) or not np.all(np.isfinite(limits)) or limits[1] <= limits[0]:
            raise ValueError("x_limits_m must contain two increasing finite values.")
        x_min, x_max = limits

    ground = terrain.surface_by_id(terrain.ground_surface_id)
    ax.plot([x_min, x_max], [ground.position_m, ground.position_m], color="black", lw=2)
    ax.text(x_min, ground.position_m, f" {ground.surface_id}", va="bottom", ha="left")

    obstacle = terrain.obstacle
    if obstacle is not None:
        bottom = terrain.ground_height_m
        ax.add_patch(
            plt.Rectangle(
                (obstacle.x_min_m, bottom),
                obstacle.width_m,
                obstacle.height_m,
                facecolor="#94a3b8",
                edgecolor="none",
                alpha=0.35,
            )
        )
        surface_colors = {
            TerrainSurfaceKind.OBSTACLE_TOP: "#16a34a",
            TerrainSurfaceKind.OBSTACLE_FRONT: "#dc2626",
            TerrainSurfaceKind.OBSTACLE_BACK: "#2563eb",
        }
        for surface in obstacle.surfaces(bottom):
            points = surface.endpoints_xz_m
            ax.plot(points[:, 0], points[:, 1], color=surface_colors[surface.kind], lw=3)
            midpoint = points.mean(axis=0)
            ax.annotate(
                surface.surface_id,
                midpoint,
                xytext=(4, 5),
                textcoords="offset points",
                fontsize=9,
                color=surface_colors[surface.kind],
            )

    vertical_extent = obstacle.height_m if obstacle is not None else 0.1
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(
        terrain.ground_height_m - max(0.02, 0.15 * vertical_extent),
        terrain.ground_height_m + max(0.12, 1.35 * vertical_extent),
    )
    ax.set(
        xlabel="world x [m] (+forward)",
        ylabel="world z [m] (+upward)",
        title="Day 3--5 Step 1: 2D ground + single rectangle",
    )
    ax.grid(True, alpha=0.25)
    return ax


__all__ = [
    "RectangleObstacle",
    "SurfaceOrientation",
    "TerrainProfile",
    "TerrainSurface2D",
    "TerrainSurfaceKind",
    "plot_terrain_profile_2d",
    "terrain_surface_rows",
]
