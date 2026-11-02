import importlib.util
from pathlib import Path

EXAMPLES_GAIT = Path(__file__).resolve().parents[1] / "examples" / "gait"
MODULE_PATH = EXAMPLES_GAIT / "generate_csv_tui.py"
SPEC = importlib.util.spec_from_file_location("generate_csv_tui", MODULE_PATH)
generate_csv_tui = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(generate_csv_tui)

TRANSFORM_PATH = EXAMPLES_GAIT / "generate_transform_csv.py"
TRANSFORM_SPEC = importlib.util.spec_from_file_location("generate_transform_csv", TRANSFORM_PATH)
generate_transform_csv = importlib.util.module_from_spec(TRANSFORM_SPEC)
TRANSFORM_SPEC.loader.exec_module(generate_transform_csv)


class DummyApp:
    def __init__(self):
        self.invalidated = False

    def invalidate(self):
        self.invalidated = True


class DummyEvent:
    def __init__(self):
        self.app = DummyApp()


def test_each_tui_mode_exposes_generate_button_field():
    for fields_fn in (
        generate_csv_tui._gait_fields,
        generate_csv_tui._lean_fields,
        generate_csv_tui._transform_fields,
    ):
        fields = fields_fn()

        assert fields[-1].key == generate_csv_tui.GENERATE_ACTION_KEY
        assert fields[-1].type == "action"
        assert fields[-1].display() == "[ Generate CSV ]"


def test_focused_generate_button_starts_generation(monkeypatch):
    tui = generate_csv_tui.CSVGeneratorTUI()
    tui.focus_idx = len(tui._fields()) - 1
    calls = []

    monkeypatch.setattr(
        tui,
        "_start_generation",
        lambda app: calls.append(app),
    )

    event = DummyEvent()
    tui._activate_focused(event)

    assert calls == [event.app]
    assert event.app.invalidated


def test_generation_progress_bar_renders_percentage():
    tui = generate_csv_tui.CSVGeneratorTUI()
    tui.generation_progress = 0.5

    rendered = "".join(text for _, text in tui._render_progress_bar(width=10))

    assert "50%" in rendered
    assert "█████" in rendered
    assert "░░░░░" in rendered


def test_generation_output_advances_progress_without_regressing():
    tui = generate_csv_tui.CSVGeneratorTUI()
    tui.generation_progress = 0.8

    assert not tui._update_progress_from_output("Generating 2 steady cycles of Trot...")
    assert tui.generation_progress == 0.8

    assert not tui._update_progress_from_output("[SUCCESS]")
    assert tui.generation_progress == 0.9

    assert not tui._update_progress_from_output("Saved to  : outputs/csv/test.csv")
    assert tui.generation_progress == 1.0


def test_tui_consumes_machine_readable_progress_without_logging():
    tui = generate_csv_tui.CSVGeneratorTUI()

    tui._handle_output_line("::progress::37")

    assert tui.generation_progress == 0.37
    assert all("::progress::" not in line for line in tui.log_lines)


def test_transform_generator_emits_machine_readable_progress(capsys, tmp_path):
    output_path = tmp_path / "transform.csv"

    generate_transform_csv.generate_transform_csv(
        target_theta=[17.0, 17.0, 17.0, 17.0],
        target_beta=[0.0, 0.0, 0.0, 0.0],
        target_gamma=[0.0, 0.0, 0.0, 0.0],
        duration=0.002,
        dt=0.001,
        output_path=str(output_path),
    )

    stdout = capsys.readouterr().out
    progress_lines = [line for line in stdout.splitlines() if line.startswith("::progress::")]

    assert progress_lines[0] == "::progress::5"
    assert progress_lines[-1] == "::progress::100"
    assert output_path.exists()
