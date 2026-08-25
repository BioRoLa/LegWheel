import matplotlib.pyplot as plt
import numpy as np
import pytest

from legwheel.planners.hybrid import (
    HipPose2D,
    RectangleObstacle2D,
    TerrainProfile2D,
    plot_leg_geometry_with_terrain_2d,
    sampled_leg_geometry_from_legacy_records,
    transform_points_hip_to_world,
    transform_points_world_to_hip,
)


def _records():
    return [
        {
            "geometry_type": "rim_arc",
            "x_m": 0.0,
            "y_m": -0.1,
            "surface_name": "foot_rim",
            "contact_state": "foot_rim",
            "arc_sample_index": 0,
            "arc_angle_deg": -90.0,
        },
        {
            "geometry_type": "rim_arc",
            "x_m": 0.1,
            "y_m": 0.0,
            "surface_name": "upper_tyre_r",
            "contact_state": "right_rim",
            "arc_sample_index": 0,
            "arc_angle_deg": 0.0,
        },
        {
            "geometry_type": "reference_point",
            "x_m": 99.0,
            "y_m": 99.0,
            "surface_name": "O",
            "contact_state": "non_contact_region",
            "arc_angle_deg": "",
        },
    ]


def test_hip_to_world_transform_uses_xz_translation_and_positive_pitch():
    pose = HipPose2D([1.0, 2.0], np.pi / 2)
    points_world = transform_points_hip_to_world([[1.0, 0.0], [0.0, 1.0]], pose)

    np.testing.assert_allclose(points_world, [[1.0, 3.0], [0.0, 2.0]], atol=1e-12)
    assert not points_world.flags.writeable


def test_world_hip_transform_round_trip_for_point_batch():
    pose = HipPose2D([0.35, 0.28], np.deg2rad(12.0))
    points_hip = np.array([[-0.2, -0.1], [0.0, 0.0], [0.15, 0.04]])

    points_world = transform_points_hip_to_world(points_hip, pose)
    recovered = transform_points_world_to_hip(points_world, pose)
    np.testing.assert_allclose(recovered, points_hip, atol=1e-12)


@pytest.mark.parametrize(
    "factory",
    [
        lambda: HipPose2D([0.0, 0.0, 0.0]),
        lambda: HipPose2D([0.0, np.nan]),
        lambda: HipPose2D([0.0, 0.0], np.inf),
        lambda: transform_points_hip_to_world([0.0, 0.0, 0.0], HipPose2D([0.0, 0.0])),
    ],
)
def test_invalid_transform_inputs_are_rejected(factory):
    with pytest.raises(ValueError):
        factory()


def test_legacy_adapter_renames_y_height_to_z_and_excludes_references():
    pose = HipPose2D([0.2, 0.3])
    geometry = sampled_leg_geometry_from_legacy_records(_records(), pose)

    np.testing.assert_allclose(geometry.points_hip_xz_m, [[0.0, -0.1], [0.1, 0.0]])
    np.testing.assert_allclose(geometry.points_world_xz_m, [[0.2, 0.2], [0.3, 0.3]])
    assert geometry.surface_names == ("foot_rim", "upper_tyre_r")
    assert geometry.contact_regions == ("foot_rim", "right_rim")


def test_overlay_plot_uses_world_coordinates_for_leg_hip_and_terrain():
    pose = HipPose2D([0.2, 0.3], np.deg2rad(5.0))
    geometry = sampled_leg_geometry_from_legacy_records(_records(), pose)
    terrain = TerrainProfile2D(obstacles=[RectangleObstacle2D("box", 0.4, 0.6, 0.05)])
    fig, ax = plt.subplots()

    returned = plot_leg_geometry_with_terrain_2d(geometry, terrain, ax=ax)

    assert returned is ax
    assert "world x" in ax.get_xlabel()
    assert len(ax.collections) >= 3  # semantic samples, hip marker, hip-axis arrow
    plt.close(fig)
