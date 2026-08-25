import matplotlib.pyplot as plt
import numpy as np
import pytest

from legwheel.planners.hybrid import (
    HipPose2D,
    RectangleObstacle2D,
    RimId,
    SampledLegGeometry2D,
    TerrainProfile2D,
    detect_contact_candidates_2d,
    detect_rectangle_vertical_face_collisions_2d,
    plot_contact_candidates_2d,
    query_contact,
)
from legwheel.planners.hybrid.geometry_2d import (
    legacy_rim_edge_margin_rad,
    legacy_rim_sample_alpha_rad,
)


def _geometry(points, regions, surfaces=None, alpha=None):
    count = len(points)
    return SampledLegGeometry2D(
        points_hip_xz_m=points,
        surface_names=surfaces or ["foot_rim"] * count,
        contact_regions=regions,
        arc_angles_deg=np.linspace(-10.0, 10.0, count),
        alpha_rad=np.zeros(count) if alpha is None else alpha,
        hip_pose=HipPose2D([0.0, 0.0]),
    )


def test_global_alpha_mapping_for_three_physical_tyre_arcs():
    assert np.rad2deg(legacy_rim_sample_alpha_rad("foot_rim", 0, 3)) == pytest.approx(-40.0)
    assert np.rad2deg(legacy_rim_sample_alpha_rad("foot_rim", 2, 3)) == pytest.approx(40.0)
    assert np.rad2deg(legacy_rim_sample_alpha_rad("upper_tyre_l", 0, 3)) == pytest.approx(-180.0)
    assert np.rad2deg(legacy_rim_sample_alpha_rad("upper_tyre_l", 2, 3)) == pytest.approx(-40.0)
    assert np.rad2deg(legacy_rim_sample_alpha_rad("upper_tyre_r", 0, 3)) == pytest.approx(40.0)
    assert np.rad2deg(legacy_rim_sample_alpha_rad("upper_tyre_r", 2, 3)) == pytest.approx(180.0)


def test_rim_edge_margin_uses_nearest_global_alpha_boundary():
    assert np.rad2deg(legacy_rim_edge_margin_rad("foot_rim", 0.0)) == pytest.approx(40.0)
    assert np.rad2deg(legacy_rim_edge_margin_rad("foot_rim", np.deg2rad(40.0))) == pytest.approx(0.0)
    assert np.rad2deg(legacy_rim_edge_margin_rad("upper_tyre_l", np.deg2rad(-90.0))) == pytest.approx(50.0)
    with pytest.raises(ValueError):
        legacy_rim_edge_margin_rad("upper_tyre_r", np.deg2rad(0.0))


def test_flat_ground_detects_only_samples_within_contact_tolerance():
    geometry = _geometry(
        [[0.0, 0.0004], [0.1, 0.002], [0.2, 0.0008]],
        ["foot_rim", "left_rim", "right_rim"],
        ["foot_rim", "upper_tyre_l", "upper_tyre_r"],
        [0.0, -1.0, 1.0],
    )
    result = detect_contact_candidates_2d(
        geometry,
        TerrainProfile2D(),
        contact_tolerance_m=1e-3,
    )

    assert len(result.candidates) == 2
    assert {candidate.rim for candidate in result.candidates} == {RimId.FOOT, RimId.RIGHT}
    assert result.candidate_surface_ids == ("ground",)
    assert result.evaluated_sample_count == 3
    assert [np.rad2deg(item.edge_margin_rad) for item in result.candidates] == pytest.approx(
        [40.0, 17.2957795]
    )


def test_non_contact_region_is_excluded_even_when_exactly_on_ground():
    geometry = _geometry(
        [[0.0, 0.0], [0.1, 0.0]],
        ["non_contact_region", "foot_rim"],
        ["upper_tyre_l", "foot_rim"],
    )
    result = detect_contact_candidates_2d(geometry, TerrainProfile2D())

    assert len(result.candidates) == 1
    assert result.candidates[0].rim is RimId.FOOT
    assert result.excluded_non_contact_sample_count == 1
    assert result.evaluated_sample_count == 1


def test_obstacle_top_candidate_does_not_pass_through_to_occluded_ground():
    terrain = TerrainProfile2D(obstacles=[RectangleObstacle2D("box", 0.4, 0.6, 0.05)])
    geometry = _geometry([[0.5, 0.0505]], ["left_rim"], ["upper_tyre_l"], [-1.2])
    result = detect_contact_candidates_2d(geometry, terrain, contact_tolerance_m=1e-3)

    assert len(result.candidates) == 1
    assert result.candidates[0].terrain_surface_id == "box_top"
    assert result.candidates[0].rim is RimId.LEFT


def test_airborne_geometry_has_no_candidates():
    geometry = _geometry([[0.1, 0.02], [0.2, 0.03]], ["foot_rim", "right_rim"])
    result = detect_contact_candidates_2d(geometry, TerrainProfile2D())

    assert not result.has_contact_candidates
    assert result.candidates == ()


def test_corner_sample_retains_multiple_surface_candidates():
    terrain = TerrainProfile2D(obstacles=[RectangleObstacle2D("box", 0.4, 0.6, 0.05)])
    geometry = _geometry([[0.4, 0.05]], ["foot_rim"])
    result = detect_contact_candidates_2d(geometry, terrain, contact_tolerance_m=0.0)

    assert {candidate.terrain_surface_id for candidate in result.candidates} == {
        "box_front",
        "box_top",
    }


def test_candidate_detector_rejects_invalid_tolerance():
    with pytest.raises(ValueError):
        detect_contact_candidates_2d(
            _geometry([[0.0, 0.0]], ["foot_rim"]),
            TerrainProfile2D(),
            contact_tolerance_m=-1e-3,
        )


def test_candidate_plot_highlights_retained_points():
    terrain = TerrainProfile2D()
    geometry = _geometry([[0.0, 0.0005]], ["foot_rim"])
    result = detect_contact_candidates_2d(geometry, terrain)
    fig, ax = plt.subplots()

    returned = plot_contact_candidates_2d(geometry, terrain, result, ax=ax)

    assert returned is ax
    assert "1 sampled contact candidates" in ax.get_title()
    plt.close(fig)


def test_ground_contact_can_coexist_with_front_face_collision():
    """A supporting foot point must not hide a blocking front-face hit."""

    terrain = TerrainProfile2D(obstacles=[RectangleObstacle2D("box", 0.4, 0.6, 0.05)])
    geometry = _geometry(
        [[0.0, 0.0002], [0.4005, 0.025]],
        ["foot_rim", "right_rim"],
        ["foot_rim", "upper_tyre_r"],
        [0.0, 1.0],
    )

    contacts = detect_contact_candidates_2d(geometry, terrain, contact_tolerance_m=1e-3)
    collisions = detect_rectangle_vertical_face_collisions_2d(
        geometry,
        terrain,
        collision_tolerance_m=1e-3,
    )

    assert any(candidate.terrain_surface_id == "ground" for candidate in contacts.candidates)
    assert collisions.has_collision
    assert collisions.collision_surface_ids == ("box_front",)
    assert collisions.collisions[0].rim is RimId.RIGHT
    assert collisions.collisions[0].penetration_depth_m == pytest.approx(0.0005)


def test_vertical_face_collision_ignores_samples_outside_face_span():
    terrain = TerrainProfile2D(obstacles=[RectangleObstacle2D("box", 0.4, 0.6, 0.05)])
    geometry = _geometry(
        [[0.4, 0.051], [0.4, -0.001]],
        ["foot_rim", "foot_rim"],
        ["foot_rim", "foot_rim"],
    )

    result = detect_rectangle_vertical_face_collisions_2d(
        geometry,
        terrain,
        collision_tolerance_m=1e-3,
    )

    assert not result.has_collision


def test_vertical_face_collision_excludes_non_contact_region():
    terrain = TerrainProfile2D(obstacles=[RectangleObstacle2D("box", 0.4, 0.6, 0.05)])
    geometry = _geometry(
        [[0.4, 0.025], [0.0, 0.0]],
        ["non_contact_region", "foot_rim"],
        ["upper_tyre_l", "foot_rim"],
    )

    result = detect_rectangle_vertical_face_collisions_2d(geometry, terrain)

    assert not result.has_collision
    assert result.evaluated_sample_count == 1
    assert result.excluded_non_contact_sample_count == 1


def test_query_contact_returns_candidates_and_collisions_together():
    terrain = TerrainProfile2D(obstacles=[RectangleObstacle2D("box", 0.4, 0.6, 0.05)])
    geometry = _geometry(
        [[0.0, 0.0002], [0.4005, 0.025]],
        ["foot_rim", "right_rim"],
        ["foot_rim", "upper_tyre_r"],
        [0.0, 1.0],
    )

    result = query_contact(
        geometry,
        terrain,
        contact_tolerance_m=1e-3,
        collision_tolerance_m=1e-3,
    )

    assert result.has_contact_candidates
    assert result.has_collisions
    assert "ground" in result.candidate_surface_ids
    assert result.collision_surface_ids == ("box_front",)
    assert result.evaluated_sample_count == 2
    assert result.excluded_non_contact_sample_count == 0


def test_query_contact_preserves_multiple_surface_candidates():
    terrain = TerrainProfile2D(obstacles=[RectangleObstacle2D("box", 0.4, 0.6, 0.05)])
    geometry = _geometry([[0.4, 0.05]], ["foot_rim"], ["foot_rim"])

    result = query_contact(geometry, terrain, contact_tolerance_m=0.0)

    assert {item.terrain_surface_id for item in result.candidates} == {
        "box_front",
        "box_top",
    }


def test_query_contact_preserves_multiple_rim_contacts():
    geometry = _geometry(
        [[0.0, 0.0], [0.2, 0.0], [0.1, 0.02]],
        ["foot_rim", "right_rim", "left_rim"],
        ["foot_rim", "upper_tyre_r", "upper_tyre_l"],
        [0.0, np.deg2rad(90.0), np.deg2rad(-90.0)],
    )

    result = query_contact(geometry, TerrainProfile2D(), contact_tolerance_m=1e-3)

    assert len(result.candidates) == 2
    assert {item.rim for item in result.candidates} == {RimId.FOOT, RimId.RIGHT}
    assert all(item.terrain_surface_id == "ground" for item in result.candidates)
    assert not result.has_collisions


def test_query_contact_rejects_invalid_collision_tolerance():
    with pytest.raises(ValueError):
        query_contact(
            _geometry([[0.0, 0.0]], ["foot_rim"]),
            TerrainProfile2D(),
            collision_tolerance_m=-1e-3,
        )
