"""Hip/world-frame geometry bridge for the Day 3--5 2D prototype.

Coordinate contract
-------------------
Both frames use ordered coordinates ``[x, z]`` in metres.  ``+x`` is forward
and ``+z`` is upward.  Positive hip pitch is counter-clockwise in the x-z
plane, so the rigid transform is::

    p_W = R(pitch_W_H) @ p_H + position_W_H

Legacy ``PlotLeg`` records store these same planar components as ``x_m`` and
``y_m``.  The adapter in this module renames legacy ``y`` to explicit ``z``;
it does not reinterpret gamma or perform any terrain/contact query.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray

from .terrain_2d import TerrainProfile, plot_terrain_profile_2d


CONTACT_REGIONS = ("foot_rim", "left_rim", "right_rim", "non_contact_region")
CONTACT_REGION_COLORS = {
    "foot_rim": "#16a34a",
    "left_rim": "#2563eb",
    "right_rim": "#f97316",
    "non_contact_region": "#9ca3af",
}

LEGACY_SURFACE_ALPHA_LIMITS_DEG = {
    "foot_rim": (-40.0, 40.0),
    "upper_tyre_l": (-180.0, -40.0),
    "upper_tyre_r": (40.0, 180.0),
}


def _finite_xz_array(value, field_name: str) -> NDArray[np.float64]:
    points = np.asarray(value, dtype=float)
    if points.ndim < 1 or points.shape[-1] != 2:
        raise ValueError(f"{field_name} must have trailing shape (..., 2); got {points.shape}.")
    if not np.all(np.isfinite(points)):
        raise ValueError(f"{field_name} must contain only finite values.")
    points = points.copy()
    points.setflags(write=False)
    return points


@dataclass(frozen=True)
class HipPose2D:
    """Pose of hip frame ``{H}`` expressed in the world x-z frame ``{W}``."""

    position_world_xz_m: NDArray[np.float64]
    pitch_world_hip_rad: float = 0.0

    def __post_init__(self) -> None:
        position = _finite_xz_array(self.position_world_xz_m, "position_world_xz_m")
        if position.shape != (2,):
            raise ValueError("position_world_xz_m must have shape (2,).")
        if not np.isfinite(self.pitch_world_hip_rad):
            raise ValueError("pitch_world_hip_rad must be finite.")
        object.__setattr__(self, "position_world_xz_m", position)

    @property
    def rotation_world_hip(self) -> NDArray[np.float64]:
        """Return ``R_W_H``, mapping a hip-frame vector into world coordinates."""

        c = np.cos(self.pitch_world_hip_rad)
        s = np.sin(self.pitch_world_hip_rad)
        rotation = np.array([[c, -s], [s, c]], dtype=float)
        rotation.setflags(write=False)
        return rotation


def transform_points_hip_to_world(
    points_hip_xz_m,
    hip_pose: HipPose2D,
) -> NDArray[np.float64]:
    """Apply ``p_W = R_W_H p_H + p_W_H`` to one point or a point batch."""

    points = _finite_xz_array(points_hip_xz_m, "points_hip_xz_m")
    transformed = points @ hip_pose.rotation_world_hip.T + hip_pose.position_world_xz_m
    transformed.setflags(write=False)
    return transformed


def transform_points_world_to_hip(
    points_world_xz_m,
    hip_pose: HipPose2D,
) -> NDArray[np.float64]:
    """Apply the inverse rigid transform ``p_H = R_W_H.T (p_W - p_W_H)``."""

    points = _finite_xz_array(points_world_xz_m, "points_world_xz_m")
    transformed = (points - hip_pose.position_world_xz_m) @ hip_pose.rotation_world_hip
    transformed.setflags(write=False)
    return transformed


@dataclass(frozen=True)
class SampledLegGeometry2D:
    """Numeric rim samples in hip frame plus their semantic F/L/R/N labels."""

    points_hip_xz_m: NDArray[np.float64]
    surface_names: Sequence[str]
    contact_regions: Sequence[str]
    arc_angles_deg: NDArray[np.float64]
    alpha_rad: NDArray[np.float64]
    hip_pose: HipPose2D

    def __post_init__(self) -> None:
        points = _finite_xz_array(self.points_hip_xz_m, "points_hip_xz_m")
        if points.ndim != 2:
            raise ValueError("points_hip_xz_m must have shape (N, 2).")
        count = len(points)
        surface_names = tuple(self.surface_names)
        contact_regions = tuple(self.contact_regions)
        angles = np.asarray(self.arc_angles_deg, dtype=float)
        alpha = np.asarray(self.alpha_rad, dtype=float)
        if (
            len(surface_names) != count
            or len(contact_regions) != count
            or angles.shape != (count,)
            or alpha.shape != (count,)
        ):
            raise ValueError("sample metadata must have the same length as points_hip_xz_m.")
        if not all(surface_names):
            raise ValueError("surface_names must not contain empty values.")
        invalid_regions = sorted(set(contact_regions) - set(CONTACT_REGIONS))
        if invalid_regions:
            raise ValueError(f"unknown contact regions: {invalid_regions}.")
        if not np.all(np.isfinite(angles)):
            raise ValueError("arc_angles_deg must contain only finite values.")
        if not np.all(np.isfinite(alpha)):
            raise ValueError("alpha_rad must contain only finite values.")
        if not isinstance(self.hip_pose, HipPose2D):
            raise TypeError("hip_pose must be a HipPose2D.")
        angles = angles.copy()
        angles.setflags(write=False)
        alpha = alpha.copy()
        alpha.setflags(write=False)
        object.__setattr__(self, "points_hip_xz_m", points)
        object.__setattr__(self, "surface_names", surface_names)
        object.__setattr__(self, "contact_regions", contact_regions)
        object.__setattr__(self, "arc_angles_deg", angles)
        object.__setattr__(self, "alpha_rad", alpha)

    @property
    def points_world_xz_m(self) -> NDArray[np.float64]:
        return transform_points_hip_to_world(self.points_hip_xz_m, self.hip_pose)


def sampled_leg_geometry_from_legacy_records(
    records: Sequence[Mapping[str, object]],
    hip_pose: HipPose2D,
) -> SampledLegGeometry2D:
    """Bridge legacy rim-arc records into the explicit hip/world contract.

    Reference points are intentionally excluded. Step 2 visualizes only the
    three physically contactable tyre arcs; their gap-facing endpoints encode
    the open non-contact sector between the left and right upper tyres.
    """

    rim_records = [record for record in records if record.get("geometry_type") == "rim_arc"]
    if not rim_records:
        raise ValueError("records must contain at least one rim_arc sample.")
    try:
        points = np.array([[record["x_m"], record["y_m"]] for record in rim_records], dtype=float)
        surface_names = [str(record["surface_name"]) for record in rim_records]
        contact_regions = [str(record["contact_state"]) for record in rim_records]
        arc_angles = np.array([record["arc_angle_deg"] for record in rim_records], dtype=float)
        surface_sample_counts = {
            surface_name: max(
                int(record["arc_sample_index"])
                for record in rim_records
                if str(record["surface_name"]) == surface_name
            )
            + 1
            for surface_name in set(surface_names)
        }
        alpha_rad = np.array(
            [
                legacy_rim_sample_alpha_rad(
                    str(record["surface_name"]),
                    int(record["arc_sample_index"]),
                    surface_sample_counts[str(record["surface_name"])],
                )
                for record in rim_records
            ],
            dtype=float,
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("legacy rim records are missing valid geometry fields.") from error
    return SampledLegGeometry2D(
        points,
        surface_names,
        contact_regions,
        arc_angles,
        alpha_rad,
        hip_pose,
    )


def legacy_rim_sample_alpha_rad(
    surface_name: str,
    arc_sample_index: int,
    sample_count: int,
) -> float:
    """Map one physical tyre-arc sample to the global rim alpha parameter."""

    if surface_name not in LEGACY_SURFACE_ALPHA_LIMITS_DEG:
        raise ValueError(f"unsupported contactable tyre surface: {surface_name!r}.")
    if sample_count < 1 or not 0 <= arc_sample_index < sample_count:
        raise ValueError("arc_sample_index must lie within sample_count.")
    fraction = 0.5 if sample_count == 1 else arc_sample_index / float(sample_count - 1)
    alpha_min_deg, alpha_max_deg = LEGACY_SURFACE_ALPHA_LIMITS_DEG[surface_name]
    return float(np.deg2rad(alpha_min_deg + fraction * (alpha_max_deg - alpha_min_deg)))


def legacy_rim_edge_margin_rad(surface_name: str, alpha_rad: float) -> float:
    """Return global-alpha distance to the nearest boundary of one rim arc."""

    if surface_name not in LEGACY_SURFACE_ALPHA_LIMITS_DEG:
        raise ValueError(f"unsupported contactable tyre surface: {surface_name!r}.")
    if not np.isfinite(alpha_rad):
        raise ValueError("alpha_rad must be finite.")
    alpha_min_deg, alpha_max_deg = LEGACY_SURFACE_ALPHA_LIMITS_DEG[surface_name]
    alpha_min_rad = float(np.deg2rad(alpha_min_deg))
    alpha_max_rad = float(np.deg2rad(alpha_max_deg))
    if alpha_rad < alpha_min_rad - 1e-12 or alpha_rad > alpha_max_rad + 1e-12:
        raise ValueError("alpha_rad must lie within the selected rim arc.")
    return float(min(alpha_rad - alpha_min_rad, alpha_max_rad - alpha_rad))


def plot_leg_geometry_with_terrain_2d(
    geometry: SampledLegGeometry2D,
    terrain: TerrainProfile,
    *,
    ax=None,
):
    """Overlay transformed F/L/R/N leg samples and rectangle geometry in ``{W}``."""

    if ax is None:
        _, ax = plt.subplots(figsize=(11, 5))
    points_world = geometry.points_world_xz_m
    x_values = list(points_world[:, 0])
    if terrain.obstacle is not None:
        x_values.extend([terrain.obstacle.x_min_m, terrain.obstacle.x_max_m])
    x_min, x_max = min(x_values), max(x_values)
    padding = max(0.08, 0.15 * max(x_max - x_min, 0.1))
    plot_terrain_profile_2d(terrain, ax=ax, x_limits_m=(x_min - padding, x_max + padding))

    regions = np.asarray(geometry.contact_regions)
    for region in CONTACT_REGIONS:
        mask = regions == region
        if np.any(mask):
            ax.scatter(
                points_world[mask, 0],
                points_world[mask, 1],
                s=7 if region != "non_contact_region" else 5,
                color=CONTACT_REGION_COLORS[region],
                alpha=0.80 if region != "non_contact_region" else 0.42,
                label=region,
                zorder=4,
            )

    hip = geometry.hip_pose.position_world_xz_m
    rotation = geometry.hip_pose.rotation_world_hip
    axis_length = 0.06
    ax.scatter(*hip, marker="x", s=70, color="black", label="hip origin {H}", zorder=6)
    ax.quiver(
        hip[0],
        hip[1],
        rotation[0, 0] * axis_length,
        rotation[1, 0] * axis_length,
        angles="xy",
        scale_units="xy",
        scale=1,
        color="#7c3aed",
        width=0.006,
        zorder=6,
    )
    ax.text(hip[0], hip[1] + 0.012, "{H}: +x_H arrow", ha="center", fontsize=9)
    ax.set_title("Step 2: hip-frame leg geometry transformed into terrain world x-z frame")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(fontsize=8, loc="best")
    return ax


__all__ = [
    "CONTACT_REGIONS",
    "HipPose2D",
    "LEGACY_SURFACE_ALPHA_LIMITS_DEG",
    "SampledLegGeometry2D",
    "legacy_rim_sample_alpha_rad",
    "legacy_rim_edge_margin_rad",
    "plot_leg_geometry_with_terrain_2d",
    "sampled_leg_geometry_from_legacy_records",
    "transform_points_hip_to_world",
    "transform_points_world_to_hip",
]
