import importlib.util
from pathlib import Path

import numpy as np
import pandas as pd

MODULE_PATH = Path(__file__).resolve().parents[1] / "examples" / "gait" / "generate_lean_csv.py"
SPEC = importlib.util.spec_from_file_location("generate_lean_csv_body_command", MODULE_PATH)
generate_lean_csv = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(generate_lean_csv)


def test_generate_lean_csv_writes_aligned_body_command_sidecar(tmp_path):
    joint_path = Path(
        generate_lean_csv.generate_lean_csv(
            pitch_deg=10.0,
            stand_height=0.25,
            n_steps=4,
            dt=0.001,
            prep_time=0.002,
            output_dir=str(tmp_path),
        )
    )
    body_path = joint_path.with_name(f"{joint_path.stem}_body_command.csv")

    joint_commands = np.loadtxt(joint_path, delimiter=",")
    body_commands = pd.read_csv(body_path)

    assert len(body_commands) == len(joint_commands)
    assert list(body_commands.columns) == [
        "source_row",
        "time_s",
        "phase",
        "height_m",
        "roll_rad",
        "pitch_rad",
        "yaw_rad",
        "x_m",
        "y_m",
    ]
    assert body_commands["source_row"].tolist() == list(range(len(joint_commands)))
    assert np.allclose(body_commands["time_s"], np.arange(len(joint_commands)) * 0.001)

    prep = body_commands.iloc[:2]
    assert prep["phase"].tolist() == ["prep", "prep"]
    assert prep[["height_m", "roll_rad", "pitch_rad", "yaw_rad", "x_m", "y_m"]].isna().all().all()

    lean = body_commands.iloc[2:]
    assert (lean["phase"] == "lean").all()
    assert np.isclose(lean["pitch_rad"].max(), np.deg2rad(10.0))
    assert np.allclose(lean["height_m"], 0.25)
