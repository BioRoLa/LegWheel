import numpy as np

from hybrid_note.scripts.analysis.rim_contact_parameter_scan import (
    contact_states_for_surface_samples,
)
from hybrid_note.scripts.kinematics.ground_contact_single_pose import (
    RIM_SURFACES,
    surface_contact_state,
)


def _states(surface_name: str, sample_count: int = 11) -> list[str]:
    return [
        surface_contact_state(surface_name, index, sample_count)
        for index in range(sample_count)
    ]


def test_contact_taxonomy_uses_semantic_rim_names():
    assert tuple(RIM_SURFACES) == ("foot_rim", "upper_tyre_l", "upper_tyre_r")
    assert set(_states("foot_rim")) == {"foot_rim"}
    left_states = _states("upper_tyre_l")
    assert left_states[0] == "non_contact_region"
    assert set(left_states[1:]) == {"left_rim"}
    right_states = _states("upper_tyre_r")
    assert set(right_states[:-1]) == {"right_rim"}
    assert right_states[-1] == "non_contact_region"

    for structural_rim in ("upper_rim_l", "upper_rim_r", "lower_rim_l", "lower_rim_r"):
        assert set(_states(structural_rim)) == {"non_contact_region"}


def test_parameter_scan_reuses_single_pose_taxonomy():
    for surface_name in (
        "foot_rim",
        "upper_tyre_l",
        "upper_tyre_r",
    ):
        assert np.array_equal(
            contact_states_for_surface_samples(surface_name, 17),
            np.asarray(_states(surface_name, 17), dtype=object),
        )
