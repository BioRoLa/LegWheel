"""``query_point_to_terrain_surfaces_2d`` still answers exactly what it did.

The function was rewritten to do its arithmetic and its validation with
``math`` on two floats instead of with ``numpy`` on two-element arrays.  The
maths is unchanged -- a clip, a subtraction, a hypotenuse -- so every field of
every result must be **bit-identical**, and that is what this holds.

The golden file was generated from the implementation as it stood before the
rewrite, over three terrains (flat, a 40 mm platform, a 100 mm platform on
sunken ground), 516 points each, at two span tolerances: 9288 rows covering all
four surface kinds and every branch (inside the solid, occluded, outside the
span).  Regenerating it from the current code would make the test vacuous, so
it is committed data, not a fixture.
"""

import json
from pathlib import Path

import numpy as np
import pytest

from legwheel.planners.hybrid.terrain_2d import RectangleObstacle
from legwheel.planners.hybrid.terrain_query_2d import (
    TerrainProfile,
    query_point_to_terrain_surfaces_2d,
)

GOLDEN = Path(__file__).parent / "data" / "terrain_query_golden.json"

#: The three terrains the golden rows were taken over, in the order their
#: ``scene`` index refers to.
TERRAINS = (
    TerrainProfile(ground_height_m=0.0, obstacles=()),
    TerrainProfile(ground_height_m=0.0, obstacles=(
        RectangleObstacle(obstacle_id="platform", x_min_m=1.0, x_max_m=1.4,
                          height_m=0.04),)),
    TerrainProfile(ground_height_m=-0.02, obstacles=(
        RectangleObstacle(obstacle_id="p2", x_min_m=0.2, x_max_m=0.6,
                          height_m=0.10),)),
)


@pytest.fixture(scope="module")
def golden():
    rows = json.loads(GOLDEN.read_text())
    assert rows, "the golden file is empty; the test would pass vacuously"
    return rows


def test_the_golden_file_covers_every_surface_kind_and_branch(golden):
    """A regression file that misses the branches is a false green."""

    kinds = {row["surface_kind"] for row in golden}
    assert kinds == {"ground", "obstacle_front", "obstacle_back", "obstacle_top"}
    assert sum(1 for r in golden if r["inside"]) > 100, "penetration branch"
    assert sum(1 for r in golden if r["occl"]) > 100, "occlusion branch"
    assert sum(1 for r in golden if not r["within"]) > 100, "out-of-span branch"
    assert len({r["tol"] for r in golden}) == 2, "both span tolerances"


def test_every_field_matches_the_pre_rewrite_implementation(golden):
    """Bit-identical, not approximately equal: the arithmetic did not change."""

    by_query: dict = {}
    for row in golden:
        by_query.setdefault((row["scene"], row["tol"], row["x"], row["z"]),
                            []).append(row)

    checked = 0
    for (scene, tol, x, z), rows in by_query.items():
        result = query_point_to_terrain_surfaces_2d(
            np.array([x, z]), TERRAINS[scene], span_tolerance_m=tol)
        gaps = {gap.surface_id: gap for gap in result.surface_gaps}
        assert len(gaps) == len(rows)
        for row in rows:
            gap = gaps[row["surface_id"]]
            assert gap.surface_kind.value == row["surface_kind"]
            assert float(gap.nearest_point_world_xz_m[0]) == row["nx"]
            assert float(gap.nearest_point_world_xz_m[1]) == row["nz"]
            assert float(gap.outward_normal_world_xz[0]) == row["onx"]
            assert float(gap.outward_normal_world_xz[1]) == row["onz"]
            assert float(gap.signed_normal_gap_m) == row["gap"]
            assert float(gap.euclidean_distance_m) == row["dist"]
            assert bool(gap.projection_within_span) == row["within"]
            assert bool(gap.is_occluded) == row["occl"]
            assert bool(gap.is_relevant) == row["rel"]
            assert bool(gap.point_inside_surface_solid) == row["inside"]
            assert float(gap.penetration_depth_m) == row["pen"]
            checked += 1
    assert checked == len(golden)


# --------------------------------------------------------------------------
# The validation the rewrite had to keep
# --------------------------------------------------------------------------


def test_a_non_finite_point_is_still_refused():
    for bad in ([float("nan"), 0.0], [0.0, float("inf")]):
        with pytest.raises(ValueError, match="finite"):
            query_point_to_terrain_surfaces_2d(np.array(bad), TERRAINS[0])


def test_a_wrongly_shaped_point_is_still_refused():
    for bad in (np.array([0.0, 0.0, 0.0]), np.array([[0.0, 0.0]])):
        with pytest.raises(ValueError, match="shape"):
            query_point_to_terrain_surfaces_2d(bad, TERRAINS[0])


def test_a_negative_or_non_finite_tolerance_is_still_refused():
    for bad in (-1e-9, float("nan")):
        with pytest.raises(ValueError, match="span_tolerance_m"):
            query_point_to_terrain_surfaces_2d(np.array([0.0, 0.1]),
                                               TERRAINS[0], span_tolerance_m=bad)


def test_a_non_unit_normal_is_still_refused():
    """The unit-vector check was the single most expensive validation; it has
    to still fire."""

    from legwheel.planners.hybrid.terrain_query_2d import (
        SurfaceGapResult2D,
        TerrainSurfaceKind,
    )

    with pytest.raises(ValueError, match="unit vector"):
        SurfaceGapResult2D(
            surface_id="ground", surface_kind=TerrainSurfaceKind.GROUND,
            point_world_xz_m=np.array([0.0, 0.1]),
            nearest_point_world_xz_m=np.array([0.0, 0.0]),
            outward_normal_world_xz=np.array([0.0, 0.9]),
            signed_normal_gap_m=0.1, euclidean_distance_m=0.1,
            projection_within_span=True, is_occluded=False, is_relevant=True,
            point_inside_surface_solid=False, penetration_depth_m=0.0)


def test_a_list_point_still_works_like_an_array():
    """Callers pass lists and tuples too; the fast path must not require one."""

    a = query_point_to_terrain_surfaces_2d([0.3, 0.12], TERRAINS[1])
    b = query_point_to_terrain_surfaces_2d(np.array([0.3, 0.12]), TERRAINS[1])
    assert len(a.surface_gaps) == len(b.surface_gaps)
    for x, y in zip(a.surface_gaps, b.surface_gaps):
        assert x.signed_normal_gap_m == y.signed_normal_gap_m
        assert x.euclidean_distance_m == y.euclidean_distance_m
