"""Day 3--5 end-to-end regression cases for the 2D contact query."""

import matplotlib.pyplot as plt
import numpy as np
import pytest

from legwheel.planners.hybrid import (
    HipPose2D,
    RectangleObstacle2D,
    RimId,
    SampledLegGeometry2D,
    TerrainProfile2D,
    plot_contact_query_2d,
    query_contact,
)


def _geometry(points, regions, surfaces=None, alpha=None):
    count = len(points)
    return SampledLegGeometry2D(
        points_hip_xz_m=points,
        surface_names=surfaces or ["foot_rim"] * count,
        contact_regions=regions,
        arc_angles_deg=np.zeros(count),
        alpha_rad=np.zeros(count) if alpha is None else alpha,
        hip_pose=HipPose2D([0.0, 0.0]),
    )


def _box_terrain():
    return TerrainProfile2D(obstacles=[RectangleObstacle2D("box", 0.4, 0.6, 0.05)])


def test_case_a_flat_ground_normal_contact():
    result = query_contact(_geometry([[0.0, 0.0]], ["foot_rim"]), TerrainProfile2D())

    assert result.has_contact_candidates
    assert result.candidate_surface_ids == ("ground",)
    assert not result.has_collisions


def test_case_b_approaching_obstacle_without_touching():
    geometry = _geometry(
        [[0.0, 0.0], [0.39, 0.025]],
        ["foot_rim", "right_rim"],
        ["foot_rim", "upper_tyre_r"],
        [0.0, 1.0],
    )
    result = query_contact(geometry, _box_terrain(), collision_tolerance_m=1e-3)

    assert any(item.terrain_surface_id == "ground" for item in result.candidates)
    assert not result.has_collisions


def test_case_c_vertical_face_interference_with_ground_contact():
    geometry = _geometry(
        [[0.0, 0.0002], [0.4005, 0.025]],
        ["foot_rim", "right_rim"],
        ["foot_rim", "upper_tyre_r"],
        [0.0, 1.0],
    )
    result = query_contact(geometry, _box_terrain())

    assert any(item.terrain_surface_id == "ground" for item in result.candidates)
    assert result.has_collisions
    assert result.collision_surface_ids == ("box_front",)


def test_case_d_obstacle_top_contact_without_vertical_collision():
    geometry = _geometry([[0.5, 0.05]], ["left_rim"], ["upper_tyre_l"], [-1.2])
    result = query_contact(geometry, _box_terrain())

    assert result.candidate_surface_ids == ("box_top",)
    assert not result.has_collisions


def test_case_e_no_contact():
    result = query_contact(_geometry([[0.0, 0.02]], ["foot_rim"]), TerrainProfile2D())

    assert result.candidates == ()
    assert result.collisions == ()


def test_case_f_near_rim_transition_has_small_margin():
    geometry = _geometry(
        [[0.0, 0.0]],
        ["foot_rim"],
        ["foot_rim"],
        [np.deg2rad(39.5)],
    )
    result = query_contact(geometry, TerrainProfile2D())

    assert len(result.candidates) == 1
    assert result.candidates[0].rim is RimId.FOOT
    assert np.rad2deg(result.candidates[0].edge_margin_rad) == pytest.approx(0.5)


def test_case_g_multiple_contacts_are_preserved():
    geometry = _geometry(
        [[0.0, 0.0], [0.2, 0.0], [0.1, 0.02]],
        ["foot_rim", "right_rim", "left_rim"],
        ["foot_rim", "upper_tyre_r", "upper_tyre_l"],
        [0.0, np.deg2rad(90.0), np.deg2rad(-90.0)],
    )
    result = query_contact(geometry, TerrainProfile2D())

    assert len(result.candidates) == 2
    assert {item.rim for item in result.candidates} == {RimId.FOOT, RimId.RIGHT}


def test_complete_query_visualization_marks_candidates_and_collisions():
    geometry = _geometry(
        [[0.0, 0.0002], [0.4005, 0.025]],
        ["foot_rim", "right_rim"],
        ["foot_rim", "upper_tyre_r"],
        [0.0, 1.0],
    )
    terrain = _box_terrain()
    result = query_contact(geometry, terrain)
    fig, ax = plt.subplots()

    returned = plot_contact_query_2d(geometry, terrain, result, ax=ax, annotate=False)

    assert returned is ax
    assert "candidates" in ax.get_title()
    assert "collisions" in ax.get_title()
    assert len(ax.collections) >= 2
    plt.close(fig)
