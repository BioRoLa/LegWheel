import importlib.util
import inspect
from pathlib import Path

from legwheel.planners.pose_planner import PosePlanner

MODULE_PATH = Path(__file__).resolve().parents[1] / "examples" / "gait" / "generate_lean_csv.py"
SPEC = importlib.util.spec_from_file_location("generate_lean_csv", MODULE_PATH)
generate_lean_csv = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(generate_lean_csv)


def test_lean_ramp_defaults_to_one_second_at_one_kilohertz():
    planner_default = inspect.signature(PosePlanner.plan_lean).parameters["n_steps"].default
    generator_default = (
        inspect.signature(generate_lean_csv.generate_lean_csv).parameters["n_steps"].default
    )

    assert planner_default == 1000
    assert generator_default == 1000
