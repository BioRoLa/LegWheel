import matplotlib.pyplot as plt
import numpy as np
import pytest

from legwheel.planners.hybrid import (
    RectangleObstacle2D,
    TerrainProfile2D,
    plot_point_terrain_query_2d,
    query_point_to_terrain_surfaces_2d,
)


@pytest.fixture
def terrain():
    return TerrainProfile2D(
        ground_height_m=0.0,
        obstacles=[RectangleObstacle2D("obstacle_0", 0.4, 0.6, 0.05)],
    )


def test_flat_ground_signed_gap_and_penetration():
    flat = TerrainProfile2D(ground_height_m=0.01)

    above = query_point_to_terrain_surfaces_2d([0.2, 0.04], flat).surface_gap_by_id("ground")
    below_query = query_point_to_terrain_surfaces_2d([0.2, 0.0], flat)
    below = below_query.surface_gap_by_id("ground")

    assert above.signed_normal_gap_m == pytest.approx(0.03)
    assert above.euclidean_distance_m == pytest.approx(0.03)
    assert above.is_relevant
    assert above.penetration_depth_m == 0.0
    assert below.signed_normal_gap_m == pytest.approx(-0.01)
    assert below.penetration_depth_m == pytest.approx(0.01)
    assert below_query.point_inside_terrain


def test_top_surface_is_relevant_only_over_rectangle_footprint(terrain):
    over = query_point_to_terrain_surfaces_2d([0.5, 0.08], terrain)
    outside = query_point_to_terrain_surfaces_2d([0.7, 0.08], terrain)

    top_over = over.surface_gap_by_id("obstacle_0_top")
    top_outside = outside.surface_gap_by_id("obstacle_0_top")
    assert top_over.signed_normal_gap_m == pytest.approx(0.03)
    assert top_over.is_relevant
    assert top_over.euclidean_distance_m == pytest.approx(0.03)
    assert not top_outside.projection_within_span
    assert not top_outside.is_relevant
    np.testing.assert_allclose(top_outside.nearest_point_world_xz_m, [0.6, 0.05])


def test_front_and_back_use_outward_normal_signed_gap(terrain):
    point = [0.39, 0.025]
    query = query_point_to_terrain_surfaces_2d(point, terrain)
    front = query.surface_gap_by_id("obstacle_0_front")
    back = query.surface_gap_by_id("obstacle_0_back")

    assert front.signed_normal_gap_m == pytest.approx(0.01)
    assert back.signed_normal_gap_m == pytest.approx(-0.21)
    assert front.projection_within_span and back.projection_within_span
    assert front.penetration_depth_m == 0.0
    assert back.penetration_depth_m == 0.0
    assert not query.point_inside_terrain
    assert query.nearest_relevant_surface.surface_id == "obstacle_0_front"


def test_inside_rectangle_reports_face_penetrations_not_false_ground_contact(terrain):
    query = query_point_to_terrain_surfaces_2d([0.41, 0.04], terrain)

    assert query.point_inside_terrain
    assert query.surface_gap_by_id("ground").is_occluded
    assert not query.surface_gap_by_id("ground").is_relevant
    assert query.surface_gap_by_id("obstacle_0_top").penetration_depth_m == pytest.approx(0.01)
    assert query.surface_gap_by_id("obstacle_0_front").penetration_depth_m == pytest.approx(0.01)
    assert query.surface_gap_by_id("obstacle_0_back").penetration_depth_m == pytest.approx(0.19)


def test_ground_is_occluded_above_obstacle_and_top_becomes_nearest(terrain):
    query = query_point_to_terrain_surfaces_2d([0.5, 0.07], terrain)

    ground = query.surface_gap_by_id("ground")
    assert ground.projection_within_span
    assert ground.is_occluded
    assert not ground.is_relevant
    assert query.nearest_relevant_surface.surface_id == "obstacle_0_top"
    assert query.nearest_relevant_surface.euclidean_distance_m == pytest.approx(0.02)


def test_span_tolerance_changes_relevance_without_contact_classification(terrain):
    point = [0.3995, 0.06]
    exact = query_point_to_terrain_surfaces_2d(point, terrain)
    tolerant = query_point_to_terrain_surfaces_2d(point, terrain, span_tolerance_m=1e-3)

    assert not exact.surface_gap_by_id("obstacle_0_top").is_relevant
    assert tolerant.surface_gap_by_id("obstacle_0_top").is_relevant


@pytest.mark.parametrize(
    "point,tolerance",
    [([0.0, 0.0, 0.0], 0.0), ([np.nan, 0.0], 0.0), ([0.0, 0.0], -1e-3)],
)
def test_invalid_query_input_is_rejected(terrain, point, tolerance):
    with pytest.raises(ValueError):
        query_point_to_terrain_surfaces_2d(point, terrain, span_tolerance_m=tolerance)


def test_query_plot_marks_point_and_nearest_projection(terrain):
    query = query_point_to_terrain_surfaces_2d([0.39, 0.025], terrain)
    fig, ax = plt.subplots()

    returned = plot_point_terrain_query_2d(query, terrain, ax=ax)

    assert returned is ax
    assert "nearest=obstacle_0_front" in ax.get_title()
    assert len(ax.collections) >= 2
    plt.close(fig)
