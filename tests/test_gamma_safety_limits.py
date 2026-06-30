import importlib.util
from pathlib import Path

import pytest

from legwheel.config import RobotParams

MODULE_PATH = Path(__file__).resolve().parents[1] / "examples"
MODULE_PATH = MODULE_PATH / "gait" / "generate_transform_csv.py"
SPEC = importlib.util.spec_from_file_location("gtc", MODULE_PATH)
generate_transform_csv = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(generate_transform_csv)


def test_gamma_safety_limit_allows_plus_minus_70_degrees():
    assert RobotParams.GAMMA_MAX_DEG == 70.0
    assert RobotParams.GAMMA_GUARD_DEG == 70.0

    generate_transform_csv.validate_pose_limits(
        theta=[17.0, 17.0, 17.0, 17.0],
        beta=[0.0, 0.0, 0.0, 0.0],
        gamma=[-70.0, 70.0, 40.0, -40.0],
        unit="deg",
    )


@pytest.mark.parametrize("gamma_value", [-70.1, 70.1])
def test_gamma_safety_limit_rejects_outside_plus_minus_70_degrees(
    gamma_value,
):
    with pytest.raises(ValueError, match="outside"):
        generate_transform_csv.validate_pose_limits(
            theta=[17.0, 17.0, 17.0, 17.0],
            beta=[0.0, 0.0, 0.0, 0.0],
            gamma=[gamma_value, 0.0, 0.0, 0.0],
            unit="deg",
        )
