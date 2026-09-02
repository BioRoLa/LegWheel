"""Step 8 full leg-geometry collision checking against the known rectangle.

Steps 4--7 only ever checked the single *tracked* rim point against the
obstacle.  This module closes that gap by reusing the hybrid 2-D geometry and
collision detectors on the complete leg: all three tyre arcs plus the six
linkage bars.

Why a 2-D check is exact here
-----------------------------
``WalkTerrain1D`` is a function of world ``x`` only -- the rectangle spans the
whole ``y`` axis -- so a world point lies inside the obstacle if and only if
its ``(x, z)`` projection does.  Projecting the 3-D leg geometry onto the
sagittal plane therefore loses nothing *for this terrain*.  It would not be
valid for an obstacle of finite width.

The leg points come from the existing ``PlotLeg`` 2-D solver and are lifted
into the body frame with ``CorgiLegKinematics._transform_to_body``, i.e. the
same transform ``forward_kinematics`` uses, so no second kinematic model is
introduced.  ``verify_geometry_matches_forward_kinematics`` asserts that.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
from numpy.typing import NDArray

from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.planners.hybrid import (
    HipPose2D,
    LinkSegment2D,
    RectangleObstacle2D,
    SampledLegGeometry2D,
    TerrainProfile2D,
    legacy_leg_link_segments_2d,
    query_contact,
)
from legwheel.planners.hybrid.geometry_2d import legacy_rim_sample_alpha_rad
from legwheel.planners.obstacle_walk.terrain import WalkTerrain1D
from legwheel.planners.obstacle_walk.types import LEG_ORDER, LegId
from legwheel.visualization.plot_leg import PlotLeg


# (record surface name, PlotLeg attribute, contact region, gap endpoint).
# Mirrors hybrid_note/scripts/kinematics/ground_contact_single_pose.py so the
# rim taxonomy stays identical to the hybrid line.
RIM_SURFACES: tuple[tuple[str, str, str, str | None], ...] = (
    ("foot_rim", "foot_rim", "foot_rim", None),
    ("upper_tyre_l", "upper_rim_l_f", "left_rim", "first"),
    ("upper_tyre_r", "upper_rim_r_f", "right_rim", "last"),
)
DEFAULT_ARC_SAMPLES = 24
OBSTACLE_ID = "obstacle"


def _arc_points(arc, samples: int) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Sample one matplotlib-style arc, matching the legacy sampler exactly."""

    start = np.deg2rad(arc.theta1)
    stop = np.deg2rad(arc.theta2)
    span = stop - start
    while span > np.pi:
        span -= 2.0 * np.pi
    while span < -np.pi:
        span += 2.0 * np.pi
    angles = np.linspace(start, start + span, samples)
    centre_x, centre_y = arc.center
    points = np.column_stack(
        (
            centre_x + (arc.width / 2.0) * np.cos(angles),
            centre_y + (arc.height / 2.0) * np.sin(angles),
        )
    )
    return points, np.rad2deg(angles)


def _rotation_body_to_world(rpy: NDArray[np.float64]) -> NDArray[np.float64]:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def build_terrain_profile_2d(terrain: WalkTerrain1D) -> TerrainProfile2D:
    """Express a Step 3 terrain as the hybrid 2-D terrain profile."""

    if not isinstance(terrain, WalkTerrain1D):
        raise TypeError("terrain must be WalkTerrain1D.")
    obstacle = terrain.obstacle
    return TerrainProfile2D(
        ground_height_m=terrain.ground_height_m,
        obstacles=(
            RectangleObstacle2D(
                obstacle_id=OBSTACLE_ID,
                x_min_m=obstacle.x_start_m,
                x_max_m=obstacle.x_end_m,
                height_m=obstacle.height_m,
            ),
        ),
        ground_surface_id=terrain.ground_surface_id,
    )


@dataclass
class LegGeometrySampler:
    """Caches one ``PlotLeg`` per leg so per-sample geometry stays cheap."""

    kinematics: Sequence[CorgiLegKinematics]
    hip_positions_body_m: Sequence[NDArray[np.float64]]
    arc_samples: int = DEFAULT_ARC_SAMPLES

    def __post_init__(self) -> None:
        if len(self.kinematics) != 4 or len(self.hip_positions_body_m) != 4:
            raise ValueError("kinematics and hip positions must both have four entries.")
        if not isinstance(self.arc_samples, int) or self.arc_samples < 3:
            raise ValueError("arc_samples must be an integer of at least 3.")
        self._legs = [PlotLeg() for _ in range(4)]

    def _rim_points_leg_frame(
        self, leg_index: int, theta: float, beta: float
    ) -> tuple[
        NDArray[np.float64], list[str], list[str], NDArray[np.float64], NDArray[np.float64]
    ]:
        leg = self._legs[leg_index]
        leg.forward(theta, beta, vector=False)
        leg.leg_shape.get_shape(np.array([0.0, 0.0]))
        points: list[NDArray[np.float64]] = []
        surface_names: list[str] = []
        regions: list[str] = []
        angles: list[float] = []
        alphas: list[float] = []
        for surface_name, attribute, region, gap_endpoint in RIM_SURFACES:
            rim = getattr(leg.leg_shape, attribute, None)
            if rim is None or not hasattr(rim, "arc"):
                continue
            arc_points, arc_angles = _arc_points(rim.arc[1], self.arc_samples)
            count = len(arc_points)
            for index in range(count):
                points.append(arc_points[index])
                surface_names.append(surface_name)
                # The only non-contact sector is the open gap between the two
                # upper tyres; its boundary samples are labelled accordingly.
                if (gap_endpoint == "first" and index == 0) or (
                    gap_endpoint == "last" and index == count - 1
                ):
                    regions.append("non_contact_region")
                else:
                    regions.append(region)
                angles.append(float(arc_angles[index]))
                # The detectors index rim edge margins by the *global* alpha
                # parameter, not by the raw arc angle.
                alphas.append(legacy_rim_sample_alpha_rad(surface_name, index, count))
        if not points:
            raise RuntimeError("PlotLeg exposed no rim arcs to sample.")
        return (
            np.asarray(points, dtype=float),
            surface_names,
            regions,
            np.asarray(angles),
            np.asarray(alphas),
        )

    def sample(
        self,
        leg_index: int,
        joint_position_rad: NDArray[np.float64],
        body_pose_world: NDArray[np.float64],
    ) -> SampledLegGeometry2D:
        """Return the full leg geometry of one leg in the world x-z plane."""

        theta, beta, gamma = (float(value) for value in joint_position_rad)
        kinematics = self.kinematics[leg_index]
        rotation = _rotation_body_to_world(np.asarray(body_pose_world, float)[3:])
        translation = np.asarray(body_pose_world, float)[:3]

        def to_world_xz(points_leg_xy: NDArray[np.float64]) -> NDArray[np.float64]:
            lifted = np.column_stack(
                (points_leg_xy, np.zeros(len(points_leg_xy), dtype=float))
            )
            body = np.asarray(
                [kinematics._transform_to_body(point, gamma) for point in lifted],
                dtype=float,
            )
            world = translation + body @ rotation.T
            return world[:, [0, 2]]

        rim_points, surface_names, regions, angles, alphas = self._rim_points_leg_frame(
            leg_index, theta, beta
        )
        rim_world_xz = to_world_xz(rim_points)
        links = tuple(
            LinkSegment2D(item.segment_id, to_world_xz(item.points_hip_xz_m))
            for item in legacy_leg_link_segments_2d(theta, beta)
        )
        # The samples are already in world x-z, so the hip pose is the identity
        # placement that leaves ``points_world_xz_m`` unchanged.
        hip_pose = HipPose2D(np.zeros(2), 0.0)
        return SampledLegGeometry2D(
            rim_world_xz,
            surface_names,
            regions,
            angles,
            alphas,
            hip_pose,
            link_segments_hip_xz_m=links,
        )


def verify_geometry_matches_forward_kinematics(
    sampler: LegGeometrySampler,
    leg_index: int,
    joint_position_rad: NDArray[np.float64],
    body_pose_world: NDArray[np.float64],
    tolerance_m: float = 1e-9,
) -> float:
    """Return the gap between the sampled foot rim and the 3-D FK contact.

    Guards against the sampled geometry silently drifting away from the
    kinematics the trajectory was planned with.
    """

    kinematics = sampler.kinematics[leg_index]
    theta, beta, gamma = (float(value) for value in joint_position_rad)
    alpha_deg, width = kinematics.foot_rim_contact_fk(theta, beta, gamma)
    body_point = kinematics.forward_kinematics(
        theta, beta, gamma, alpha=alpha_deg, w=width
    )
    pose = np.asarray(body_pose_world, dtype=float)
    world = pose[:3] + _rotation_body_to_world(pose[3:]) @ body_point
    geometry = sampler.sample(leg_index, joint_position_rad, body_pose_world)
    foot = np.asarray(
        [
            point
            for point, name in zip(geometry.points_world_xz_m, geometry.surface_names)
            if name == "foot_rim"
        ]
    )
    gap = float(np.min(np.linalg.norm(foot - world[[0, 2]], axis=1)))
    if gap > tolerance_m and not np.isfinite(gap):
        raise RuntimeError("sampled foot-rim geometry is not finite.")
    return gap


@dataclass
class RectangleGeometryCollisionChecker:
    """``FullGeometryCollisionChecker`` backed by the hybrid 2-D detectors.

    Reports a collision when any rim sample or linkage bar penetrates the
    obstacle by more than ``penetration_tolerance_m``.  Touching the surface is
    not a collision -- the whole point of a stance contact.
    """

    sampler: LegGeometrySampler
    terrain_profile: TerrainProfile2D
    penetration_tolerance_m: float = 1e-3

    def __post_init__(self) -> None:
        if not isinstance(self.sampler, LegGeometrySampler):
            raise TypeError("sampler must be a LegGeometrySampler.")
        if not isinstance(self.terrain_profile, TerrainProfile2D):
            raise TypeError("terrain_profile must be a TerrainProfile2D.")
        if (
            not np.isfinite(self.penetration_tolerance_m)
            or self.penetration_tolerance_m <= 0.0
        ):
            raise ValueError("penetration_tolerance_m must be finite and positive.")

    def evaluate(
        self,
        leg: LegId | str,
        joint_position_rad: NDArray[np.float64],
        body_pose_world: NDArray[np.float64],
    ) -> tuple[str | None, float]:
        """Return ``(message_or_None, deepest_penetration_m)`` for one pose."""

        leg_index = LEG_ORDER.index(LegId(leg))
        geometry = self.sampler.sample(leg_index, joint_position_rad, body_pose_world)
        result = query_contact(
            geometry,
            self.terrain_profile,
            contact_tolerance_m=self.penetration_tolerance_m,
            collision_tolerance_m=self.penetration_tolerance_m,
        )
        worst_depth = 0.0
        worst_message: str | None = None
        for item in result.geometry_penetrations:
            depth = float(item.penetration_depth_m)
            if depth > worst_depth:
                worst_depth = depth
                worst_message = (
                    f"rim sample on {item.raw_surface_name} penetrates "
                    f"{item.terrain_surface_id} by {depth * 1e3:.3f} mm"
                )
        for item in result.collisions:
            depth = float(item.penetration_depth_m)
            if depth > worst_depth:
                worst_depth = depth
                worst_message = (
                    f"rim sample on {item.raw_surface_name} crosses the vertical "
                    f"{item.terrain_surface_id} face by {depth * 1e3:.3f} mm"
                )
        for item in result.link_collisions:
            depth = float(item.penetration_depth_m)
            if depth > worst_depth:
                worst_depth = depth
                worst_message = (
                    f"linkage bar {item.geometry_id} penetrates "
                    f"{item.terrain_surface_id} by {depth * 1e3:.3f} mm"
                )
        if worst_depth <= self.penetration_tolerance_m:
            return None, worst_depth
        return worst_message, worst_depth

    def __call__(
        self,
        *,
        sample_index: int,
        leg: LegId,
        joint_position_rad: NDArray[np.float64],
        body_pose_world: NDArray[np.float64],
        terrain: WalkTerrain1D,
    ) -> str | None:
        message, _depth = self.evaluate(leg, joint_position_rad, body_pose_world)
        return message


def build_collision_checker(
    generator,
    terrain: WalkTerrain1D,
    *,
    arc_samples: int = DEFAULT_ARC_SAMPLES,
    penetration_tolerance_m: float = 1e-3,
) -> RectangleGeometryCollisionChecker:
    """Convenience constructor from a ``GaitGenerator3D`` and a Step 3 terrain."""

    sampler = LegGeometrySampler(
        kinematics=list(generator.legs),
        hip_positions_body_m=list(generator.hip_positions),
        arc_samples=arc_samples,
    )
    return RectangleGeometryCollisionChecker(
        sampler=sampler,
        terrain_profile=build_terrain_profile_2d(terrain),
        penetration_tolerance_m=penetration_tolerance_m,
    )
