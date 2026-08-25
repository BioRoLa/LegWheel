import matplotlib.pyplot as plt
import numpy as np
import pytest

from legwheel.planners.hybrid import RectangleObstacle2D, TerrainProfile2D
from legwheel.planners.hybrid.terrain_2d import (
    RectangleObstacle,
    SurfaceOrientation,
    TerrainProfile,
    TerrainSurfaceKind,
    plot_terrain_profile_2d,
)


def test_public_aliases_keep_2d_and_3d_terrain_contracts_unambiguous():
    assert RectangleObstacle2D is RectangleObstacle
    assert TerrainProfile2D is TerrainProfile


def test_flat_ground_has_one_infinite_horizontal_surface():
    terrain = TerrainProfile(ground_height_m=0.03)

    assert terrain.obstacle is None
    assert terrain.surface_ids == ("ground",)
    ground = terrain.surface_by_id("ground")
    assert ground.kind is TerrainSurfaceKind.GROUND
    assert ground.orientation is SurfaceOrientation.HORIZONTAL
    assert ground.position_m == pytest.approx(0.03)
    assert np.all(np.isinf(ground.endpoints_xz_m[:, 0]))


def test_rectangle_exposes_top_front_and_back_surface_geometry():
    obstacle = RectangleObstacle("obstacle_0", x_min_m=0.4, x_max_m=0.6, height_m=0.05)
    terrain = TerrainProfile(ground_height_m=0.01, obstacles=[obstacle])

    assert terrain.surface_ids == (
        "ground",
        "obstacle_0_top",
        "obstacle_0_front",
        "obstacle_0_back",
    )

    top = terrain.surface_by_id("obstacle_0_top")
    front = terrain.surface_by_id("obstacle_0_front")
    back = terrain.surface_by_id("obstacle_0_back")
    np.testing.assert_allclose(top.endpoints_xz_m, [[0.4, 0.06], [0.6, 0.06]])
    np.testing.assert_allclose(front.endpoints_xz_m, [[0.4, 0.01], [0.4, 0.06]])
    np.testing.assert_allclose(back.endpoints_xz_m, [[0.6, 0.01], [0.6, 0.06]])
    assert top.kind is TerrainSurfaceKind.OBSTACLE_TOP
    assert front.kind is TerrainSurfaceKind.OBSTACLE_FRONT
    assert back.kind is TerrainSurfaceKind.OBSTACLE_BACK


@pytest.mark.parametrize(
    "factory",
    [
        lambda: RectangleObstacle("", 0.0, 1.0, 0.1),
        lambda: RectangleObstacle("box", 1.0, 0.0, 0.1),
        lambda: RectangleObstacle("box", 0.0, 1.0, 0.0),
        lambda: TerrainProfile(obstacles=[RectangleObstacle("a", 0.0, 1.0, 0.1), RectangleObstacle("b", 2.0, 3.0, 0.1)]),
        lambda: TerrainProfile(obstacles=[object()]),
    ],
)
def test_invalid_day3_5_terrain_is_rejected(factory):
    with pytest.raises((TypeError, ValueError)):
        factory()


def test_surface_lookup_rejects_unknown_identity():
    with pytest.raises(KeyError, match="unknown terrain surface"):
        TerrainProfile().surface_by_id("missing")


def test_plot_contains_ground_and_three_obstacle_surface_lines():
    terrain = TerrainProfile(obstacles=[RectangleObstacle("box", 0.4, 0.6, 0.05)])
    fig, ax = plt.subplots()
    returned = plot_terrain_profile_2d(terrain, ax=ax)

    assert returned is ax
    assert len(ax.lines) == 4
    assert ax.get_xlabel() == "world x [m] (+forward)"
    plt.close(fig)
