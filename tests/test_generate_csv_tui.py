import importlib.util
from pathlib import Path

MODULE_PATH = Path(__file__).resolve().parents[1] / "examples" / "gait" / "generate_csv_tui.py"
SPEC = importlib.util.spec_from_file_location("generate_csv_tui", MODULE_PATH)
generate_csv_tui = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(generate_csv_tui)


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
