import numpy as np

from hybrid_note.scripts.kinematics.leg_geometry_inventory import (
    build_leg_geometry_inventory,
    geometry_capability_rows,
    rim_classification_rows,
)


def test_inventory_exposes_all_named_rims_and_lowest_region():
    inventory = build_leg_geometry_inventory(
        np.deg2rad(80.0),
        np.deg2rad(-20.0),
        np.deg2rad(10.0),
        arc_samples=11,
        lateral_samples=3,
    )

    rows = inventory["surface_summary"]
    assert len(rows) == 3
    assert {row["surface_name"] for row in rows} == {
        "foot_rim",
        "upper_tyre_l",
        "upper_tyre_r",
    }
    assert all(row["sample_count"] == 11 for row in rows)
    assert inventory["contact_2d"]["surface_points"].shape == (3 * 11 + 3, 2)
    assert inventory["contact_3d"]["surface_points"].shape == (3 * 11 * 3, 3)
    assert inventory["contact_2d"]["number_of_contact_points"] > 0


def test_inventory_calls_out_world_frame_and_alpha_adapter_gaps():
    rows = geometry_capability_rows()
    world = next(row for row in rows if row["geometry/data"] == "world-frame geometry")
    alpha = next(row for row in rows if row["geometry/data"] == "global rim parameter alpha")

    assert world["available now"] == "no"
    assert "Step 2" in world["Step 1+ action"]
    assert "degree" in alpha["frame / unit"]


def test_semantic_rim_taxonomy_matches_contact_map_contract():
    rows = rim_classification_rows()

    assert [row["state_id"] for row in rows] == ["F", "L", "R", "N"]
    assert [row["semantic_class"] for row in rows] == [
        "foot_rim",
        "left_rim",
        "right_rim",
        "non_contact_region",
    ]
    assert all(row["valid_contact_rim"] for row in rows[:3])
    assert not rows[3]["valid_contact_rim"]
