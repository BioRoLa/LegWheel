# 2026-07-02 — Add TUI generation progress bar

**Type:** feat

**Scope:** legwheel

## Changes

- Added a running progress bar to the LegWheel CSV TUI status line during Gait, Lean, and Transform CSV generation.
- Progress initially advanced from process start through generator output milestones (`Generating`/`Planning`, `[SUCCESS]`, `Saved to`) and did not regress if later output reported an earlier phase.
- Added machine-readable generator progress output (`::progress::<percent>`) to the Gait, Lean, and Transform CSV generators for more accurate TUI progress.
- Updated the TUI to consume `::progress::` lines without printing them in the log, while keeping legacy text-based progress fallback.
- Added focused regression tests for progress-bar rendering, output-driven progress updates, hidden machine-readable progress lines, and Transform generator progress emission.

## Decisions

- Kept progress handling inside the existing prompt_toolkit TUI without adding dependencies.
- Chose a simple stdout protocol (`::progress::<percent>`) so standalone generators remain script-compatible and TUI can track exact generator stages.

## Tracking impact

- DEVLOG / ROADMAP / research-log impact is recorded as this branch-local change fragment to avoid editing central append-heavy indexes directly.

## Verification

- `cd LegWheel && uv run pytest tests/test_generate_csv_tui.py -q` passed.
- `cd LegWheel && uv run black --line-length 100 examples/gait/generate_csv_tui.py examples/gait/generate_hardware_csv.py examples/gait/generate_lean_csv.py examples/gait/generate_transform_csv.py tests/test_generate_csv_tui.py` passed.
- `cd LegWheel && uv run flake8 --ignore=E501 examples/gait/generate_csv_tui.py examples/gait/generate_hardware_csv.py examples/gait/generate_lean_csv.py examples/gait/generate_transform_csv.py tests/test_generate_csv_tui.py` passed.
- `cd LegWheel && uv run pytest -q` was attempted after the generator progress update; 26 tests passed and 2 pre-existing checker expectation tests failed (`tests/test_toroidal_contact_and_checker.py` expects `Velocity Guard (Y)` output that current checker text does not emit).
- `cd LegWheel && uv run black --line-length 100 legwheel/` was attempted but created unrelated formatting diffs in 21 package files; those diffs were reverted after user confirmation.
- `cd LegWheel && uv run flake8 legwheel/` was attempted before reverting the formatting diffs and fails on pre-existing style issues in `legwheel/models/`.
