"""Step 3 terrain contract for straight Walk over one known rectangle.

This module is intentionally a query layer. It classifies a nominal world-X
touchdown and never selects a gait action, generates a swing, calls IK, or
modifies a trajectory segment.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np


BOUNDARY_ATOL_M = 1e-12


class TouchdownStatus(str, Enum):
    """Outcome of a nominal point-touchdown query."""

    GROUND = "ground"
    OBSTACLE_TOP = "obstacle_top"
    NO_LEGAL_TOUCHDOWN = "NO_LEGAL_TOUCHDOWN"


class TouchdownRejectReason(str, Enum):
    """Stable reasons that a query can reject a nominal touchdown."""

    WITHIN_EDGE_MARGIN = "WITHIN_EDGE_MARGIN"


@dataclass(frozen=True)
class RectangleObstacle1D:
    """One rectangular obstacle projected onto the straight world-X path.

    ``height_m`` is measured upward from the terrain ground plane. The legal
    top interval is closed and has positive width:
    ``[x_start + edge_margin, x_end - edge_margin]``.
    """

    x_start_m: float
    length_m: float
    height_m: float
    edge_margin_m: float
    top_surface_id: str = "obstacle_top"

    def __post_init__(self) -> None:
        for name in ("x_start_m", "length_m", "height_m", "edge_margin_m"):
            if not np.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite.")
        if self.length_m <= 0.0:
            raise ValueError("length_m must be positive.")
        if self.height_m <= 0.0:
            raise ValueError("height_m must be positive.")
        if self.edge_margin_m < 0.0:
            raise ValueError("edge_margin_m must be non-negative.")
        if self.length_m <= 2.0 * self.edge_margin_m:
            raise ValueError(
                "obstacle top is too short: length_m must be greater than "
                "2 * edge_margin_m."
            )
        if not isinstance(self.top_surface_id, str) or not self.top_surface_id:
            raise ValueError("top_surface_id must be a non-empty string.")

    @property
    def x_end_m(self) -> float:
        return self.x_start_m + self.length_m

    @property
    def legal_top_x_min_m(self) -> float:
        return self.x_start_m + self.edge_margin_m

    @property
    def legal_top_x_max_m(self) -> float:
        return self.x_end_m - self.edge_margin_m


@dataclass(frozen=True)
class WalkTerrain1D:
    """Flat ground plus exactly one rectangle for the first Walk baseline."""

    obstacle: RectangleObstacle1D
    ground_height_m: float = 0.0
    ground_surface_id: str = "ground"

    def __post_init__(self) -> None:
        if not isinstance(self.obstacle, RectangleObstacle1D):
            raise TypeError("obstacle must be RectangleObstacle1D.")
        if not np.isfinite(self.ground_height_m):
            raise ValueError("ground_height_m must be finite.")
        if not isinstance(self.ground_surface_id, str) or not self.ground_surface_id:
            raise ValueError("ground_surface_id must be a non-empty string.")
        if self.ground_surface_id == self.obstacle.top_surface_id:
            raise ValueError("ground and obstacle-top surface IDs must be unique.")


@dataclass(frozen=True)
class TouchdownQueryResult:
    """Terrain-only classification of one nominal world-X touchdown."""

    nominal_x_world_m: float
    status: TouchdownStatus
    surface_id: str | None
    surface_height_world_m: float | None
    distance_to_nearest_obstacle_edge_m: float
    required_edge_margin_m: float
    rejection_reason: TouchdownRejectReason | None = None

    def __post_init__(self) -> None:
        if not np.isfinite(self.nominal_x_world_m):
            raise ValueError("nominal_x_world_m must be finite.")
        object.__setattr__(self, "status", TouchdownStatus(self.status))
        if not np.isfinite(self.distance_to_nearest_obstacle_edge_m):
            raise ValueError("distance_to_nearest_obstacle_edge_m must be finite.")
        if self.distance_to_nearest_obstacle_edge_m < 0.0:
            raise ValueError("distance_to_nearest_obstacle_edge_m must be non-negative.")
        if not np.isfinite(self.required_edge_margin_m) or self.required_edge_margin_m < 0.0:
            raise ValueError("required_edge_margin_m must be finite and non-negative.")
        if self.rejection_reason is not None:
            object.__setattr__(
                self,
                "rejection_reason",
                TouchdownRejectReason(self.rejection_reason),
            )

        rejected = self.status is TouchdownStatus.NO_LEGAL_TOUCHDOWN
        if rejected:
            if self.surface_id is not None or self.surface_height_world_m is not None:
                raise ValueError("a rejected touchdown must not select a surface or height.")
            if self.rejection_reason is None:
                raise ValueError("a rejected touchdown must provide rejection_reason.")
        else:
            if not isinstance(self.surface_id, str) or not self.surface_id:
                raise ValueError("a legal touchdown must provide a non-empty surface_id.")
            if self.surface_height_world_m is None or not np.isfinite(
                self.surface_height_world_m
            ):
                raise ValueError("a legal touchdown must provide a finite surface height.")
            if self.rejection_reason is not None:
                raise ValueError("a legal touchdown must not provide rejection_reason.")

    @property
    def is_legal(self) -> bool:
        return self.status is not TouchdownStatus.NO_LEGAL_TOUCHDOWN


def query_touchdown_surface(
    terrain: WalkTerrain1D,
    nominal_x_world_m: float,
) -> TouchdownQueryResult:
    """Classify a point touchdown on ground, legal top, or top edge margin.

    Ground immediately before/after the obstacle remains point-wise legal in
    Step 3. Full wheel/leg overlap with a vertical face belongs to the later
    trajectory collision checker, not this surface-height query.
    """

    if not isinstance(terrain, WalkTerrain1D):
        raise TypeError("terrain must be WalkTerrain1D.")
    x = float(nominal_x_world_m)
    if not np.isfinite(x):
        raise ValueError("nominal_x_world_m must be finite.")

    obstacle = terrain.obstacle
    if x < obstacle.x_start_m:
        return TouchdownQueryResult(
            nominal_x_world_m=x,
            status=TouchdownStatus.GROUND,
            surface_id=terrain.ground_surface_id,
            surface_height_world_m=terrain.ground_height_m,
            distance_to_nearest_obstacle_edge_m=obstacle.x_start_m - x,
            required_edge_margin_m=obstacle.edge_margin_m,
        )
    if x > obstacle.x_end_m:
        return TouchdownQueryResult(
            nominal_x_world_m=x,
            status=TouchdownStatus.GROUND,
            surface_id=terrain.ground_surface_id,
            surface_height_world_m=terrain.ground_height_m,
            distance_to_nearest_obstacle_edge_m=x - obstacle.x_end_m,
            required_edge_margin_m=obstacle.edge_margin_m,
        )

    edge_distance = min(x - obstacle.x_start_m, obstacle.x_end_m - x)
    inside_safe_top = (
        x >= obstacle.legal_top_x_min_m - BOUNDARY_ATOL_M
        and x <= obstacle.legal_top_x_max_m + BOUNDARY_ATOL_M
    )
    if not inside_safe_top:
        return TouchdownQueryResult(
            nominal_x_world_m=x,
            status=TouchdownStatus.NO_LEGAL_TOUCHDOWN,
            surface_id=None,
            surface_height_world_m=None,
            distance_to_nearest_obstacle_edge_m=edge_distance,
            required_edge_margin_m=obstacle.edge_margin_m,
            rejection_reason=TouchdownRejectReason.WITHIN_EDGE_MARGIN,
        )

    return TouchdownQueryResult(
        nominal_x_world_m=x,
        status=TouchdownStatus.OBSTACLE_TOP,
        surface_id=obstacle.top_surface_id,
        surface_height_world_m=terrain.ground_height_m + obstacle.height_m,
        distance_to_nearest_obstacle_edge_m=edge_distance,
        required_edge_margin_m=obstacle.edge_margin_m,
    )
