"""Step 0 regression lock for the existing flat-ground Walk CSV generator.

This test deliberately exercises the public hardware CSV path.  Obstacle,
terrain, segment, and world-state behavior do not belong in this baseline.
"""

from __future__ import annotations

import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
GENERATOR_PATH = REPO_ROOT / "examples" / "gait" / "generate_hardware_csv.py"
FIXTURE_PATH = Path(__file__).parent / "fixtures" / "flat_walk_baseline_v1.json"
LEG_NAMES = ("FL", "FR", "RR", "RL")


def _load_generator_module():
    spec = importlib.util.spec_from_file_location("flat_walk_hardware_csv", GENERATOR_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load hardware CSV generator at {GENERATOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _phase_transitions(phase: np.ndarray) -> list[dict[str, int | str]]:
    transitions: list[dict[str, int | str]] = []
    for row in range(1, len(phase)):
        for leg_index, leg_name in enumerate(LEG_NAMES):
            previous = int(phase[row - 1, leg_index])
            current = int(phase[row, leg_index])
            if current != previous:
                transitions.append(
                    {"row": row, "leg": leg_name, "from": previous, "to": current}
                )
    return transitions


def test_flat_walk_hardware_csv_matches_step0_fixture(tmp_path):
    fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
    parameters = fixture["parameters"]
    expected = fixture["expected"]
    generator = _load_generator_module()

    # Keep the regression output quiet while still executing the same function
    # used by the CLI and CSV UI.
    with contextlib.redirect_stdout(io.StringIO()):
        csv_path = Path(
            generator.generate_hardware_csv(
                twist=parameters["twist"],
                gait_type=parameters["gait_type"],
                stand_height=parameters["stand_height"],
                step_height=parameters["step_height"],
                period=parameters["period"],
                dt=parameters["dt"],
                n_cycles=parameters["n_cycles"],
                output_dir=str(tmp_path),
                with_launch=parameters["with_launch"],
                stability_margin=parameters["stability_margin"],
                stance_duty=parameters["stance_duty"],
            )
        )

    phase_path = csv_path.with_name(f"{csv_path.stem}_phase.csv")
    commands = np.loadtxt(csv_path, delimiter=",")
    phase = np.loadtxt(phase_path, delimiter=",", skiprows=1)

    assert csv_path.name == fixture["expected_filename"]
    assert list(commands.shape) == expected["command_shape"]
    assert list(phase.shape) == expected["phase_shape"]
    assert phase_path.read_text(encoding="utf-8").splitlines()[0] == (
        "FL_Phase,FR_Phase,RR_Phase,RL_Phase"
    )
    assert set(np.unique(phase)) <= {0.0, 1.0}

    np.testing.assert_allclose(commands[0], expected["first_command"], atol=5e-7, rtol=0.0)
    np.testing.assert_allclose(
        commands[fixture["prep"]["rows"] - 1],
        expected["prep_last_command"],
        atol=5e-7,
        rtol=0.0,
    )
    np.testing.assert_allclose(
        commands[fixture["prep"]["rows"]],
        expected["steady_first_command"],
        atol=5e-7,
        rtol=0.0,
    )
    np.testing.assert_allclose(commands[-1], expected["last_command"], atol=5e-7, rtol=0.0)

    assert _phase_transitions(phase) == expected["phase_transitions"]
    for leg_index, leg_name in enumerate(LEG_NAMES):
        counts = expected["phase_counts_by_leg"][leg_name]
        assert int(np.sum(phase[:, leg_index] == 0)) == counts["stance"]
        assert int(np.sum(phase[:, leg_index] == 1)) == counts["swing"]

    # Hashes lock every six-decimal output sample, while the selected-row and
    # phase assertions above make a future mismatch easier to diagnose.
    assert _sha256(csv_path) == expected["csv_sha256"]
    assert _sha256(phase_path) == expected["phase_csv_sha256"]


def test_step0_fixture_documents_the_external_csv_contract():
    fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))

    assert fixture["planner"].endswith("gait_generator_3d.py:GaitGenerator3D")
    assert fixture["hardware_column_order"] == [
        "FL_theta",
        "FL_beta",
        "FR_theta",
        "FR_beta",
        "RR_theta",
        "RR_beta",
        "RL_theta",
        "RL_beta",
        "FL_gamma",
        "FR_gamma",
        "RR_gamma",
        "RL_gamma",
    ]
    assert fixture["phase_column_order"] == list(LEG_NAMES)
    assert fixture["prep"]["rows"] == int(
        fixture["prep"]["duration_s"] / fixture["parameters"]["dt"]
    )
