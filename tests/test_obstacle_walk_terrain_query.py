"""Step 3 tests for the one-rectangle Walk touchdown query."""

import math

import pytest

from legwheel.planners.obstacle_walk import (
    RectangleObstacle1D,
    TouchdownRejectReason,
    TouchdownStatus,
    WalkTerrain1D,
    query_touchdown_surface,
)


@pytest.fixture
def terrain() -> WalkTerrain1D:
    return WalkTerrain1D(
        RectangleObstacle1D(
            x_start_m=0.40,
            length_m=0.30,
            height_m=0.10,
            edge_margin_m=0.04,
        )
    )


@pytest.mark.parametrize(
    ("x", "edge_distance"),
    [
        (0.10, 0.30),
        (0.399, 0.001),
        (0.701, 0.001),
        (1.00, 0.30),
    ],
)
def test_touchdowns_before_and_after_obstacle_are_ground(terrain, x, edge_distance):
    result = query_touchdown_surface(terrain, x)

    assert result.status is TouchdownStatus.GROUND
    assert result.is_legal
    assert result.surface_id == "ground"
    assert result.surface_height_world_m == pytest.approx(0.0)
    assert result.distance_to_nearest_obstacle_edge_m == pytest.approx(edge_distance)
    assert result.rejection_reason is None


@pytest.mark.parametrize("x", [0.44, 0.50, 0.66])
def test_closed_safe_top_interval_returns_obstacle_top(terrain, x):
    result = query_touchdown_surface(terrain, x)

    assert result.status is TouchdownStatus.OBSTACLE_TOP
    assert result.is_legal
    assert result.surface_id == "obstacle_top"
    assert result.surface_height_world_m == pytest.approx(0.10)
    assert result.distance_to_nearest_obstacle_edge_m >= 0.04 - 1e-12
    assert result.rejection_reason is None


@pytest.mark.parametrize(
    ("x", "edge_distance"),
    [
        (0.40, 0.0),
        (0.42, 0.02),
        (0.68, 0.02),
        (0.70, 0.0),
    ],
)
def test_touchdowns_inside_top_edge_margin_are_rejected(terrain, x, edge_distance):
    result = query_touchdown_surface(terrain, x)

    assert result.status is TouchdownStatus.NO_LEGAL_TOUCHDOWN
    assert not result.is_legal
    assert result.surface_id is None
    assert result.surface_height_world_m is None
    assert result.distance_to_nearest_obstacle_edge_m == pytest.approx(edge_distance)
    assert result.required_edge_margin_m == pytest.approx(0.04)
    assert result.rejection_reason is TouchdownRejectReason.WITHIN_EDGE_MARGIN


def test_nonzero_ground_height_offsets_both_surfaces():
    terrain = WalkTerrain1D(
        RectangleObstacle1D(0.4, 0.3, 0.1, 0.04),
        ground_height_m=0.02,
    )

    assert query_touchdown_surface(terrain, 0.2).surface_height_world_m == pytest.approx(0.02)
    assert query_touchdown_surface(terrain, 0.5).surface_height_world_m == pytest.approx(0.12)


def test_zero_margin_makes_physical_top_edges_legal():
    terrain = WalkTerrain1D(RectangleObstacle1D(0.4, 0.3, 0.1, 0.0))

    assert query_touchdown_surface(terrain, 0.4).status is TouchdownStatus.OBSTACLE_TOP
    assert query_touchdown_surface(terrain, 0.7).status is TouchdownStatus.OBSTACLE_TOP


@pytest.mark.parametrize(
    "kwargs",
    [
        {"x_start_m": math.nan, "length_m": 0.3, "height_m": 0.1, "edge_margin_m": 0.04},
        {"x_start_m": 0.4, "length_m": 0.0, "height_m": 0.1, "edge_margin_m": 0.04},
        {"x_start_m": 0.4, "length_m": 0.3, "height_m": 0.0, "edge_margin_m": 0.04},
        {"x_start_m": 0.4, "length_m": 0.3, "height_m": -0.1, "edge_margin_m": 0.04},
        {"x_start_m": 0.4, "length_m": 0.3, "height_m": 0.1, "edge_margin_m": -0.01},
    ],
)
def test_invalid_obstacle_dimensions_are_rejected(kwargs):
    with pytest.raises(ValueError):
        RectangleObstacle1D(**kwargs)


@pytest.mark.parametrize(
    ("length", "margin"),
    [
        (0.08, 0.04),
        (0.07, 0.04),
    ],
)
def test_top_without_positive_width_safe_interval_is_rejected(length, margin):
    with pytest.raises(ValueError, match="too short"):
        RectangleObstacle1D(0.4, length, 0.1, margin)


def test_surface_ids_must_be_nonempty_and_unique():
    with pytest.raises(ValueError, match="top_surface_id"):
        RectangleObstacle1D(0.4, 0.3, 0.1, 0.04, top_surface_id="")

    obstacle = RectangleObstacle1D(
        0.4,
        0.3,
        0.1,
        0.04,
        top_surface_id="same_surface",
    )
    with pytest.raises(ValueError, match="unique"):
        WalkTerrain1D(obstacle, ground_surface_id="same_surface")


@pytest.mark.parametrize("x", [math.nan, math.inf, -math.inf])
def test_nonfinite_touchdown_query_is_rejected(terrain, x):
    with pytest.raises(ValueError, match="finite"):
        query_touchdown_surface(terrain, x)
