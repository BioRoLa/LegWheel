# Plotly Migration Plan for LegWheel

**Goal:** Add Plotly as an interactive visualization backend for LegWheel without breaking the existing Matplotlib workflow.

**Current state:** LegWheel currently uses Matplotlib for 2D linkage plots, 3D robot rendering, workspace diagnostics, CSV trajectory viewing, and animation scripts. Matplotlib remains valuable for static publication-oriented figures and legacy examples, while Plotly is attractive for interactive 3D inspection and HTML sharing.

**Recommended direction:** Keep Matplotlib as the default/static backend and introduce Plotly as an optional interactive backend first. On the `feature/plotly-version-policy` branch, the LegWheel Python baseline is upgraded to Python `>=3.10`, which makes Plotly 6.x a viable future dependency while still avoiding a Plotly-only migration until the interactive workflow is proven useful.

---

## Dependency decision

### Option 1: Keep Python >=3.7 and pin Plotly 5.x

Historical low-risk option before the Python baseline upgrade.

```toml
[project.optional-dependencies]
plotly = [
    "plotly>=5.24,<6",
    "kaleido>=0.2.1",
]
```

Pros:

- Preserves the current `requires-python = ">=3.7"` baseline.
- Avoids forcing Plotly on users who only need kinematics or CSV generation.

Cons:

- Uses older Plotly 5.x instead of the latest 6.x line.
- Static export compatibility with Kaleido must be verified.

### Option 2: Raise the baseline to Python >=3.8

```toml
requires-python = ">=3.8"
plotly = ["plotly>=6,<7", "kaleido"]
```

Pros:

- Aligns with current Plotly releases.
- Cleaner long-term dependency maintenance.

Cons:

- Changes the supported runtime baseline.
- May affect older Docker, ROS, or lab machines.

**Current branch decision:** Upgrade LegWheel to Python `>=3.10` and add Plotly as an optional extra. Plotly remains outside mandatory runtime dependencies.

---

## Files likely to change

### Dependency and documentation

- `LegWheel/pyproject.toml`: add optional Plotly dependency group (`plotly[kaleido]>=6,<7`).
- `LegWheel/uv.lock`: update with `uv` after dependency changes.
- `LegWheel/README.md`: document installation and usage.
- `LegWheel/examples/README.md`: document Plotly examples.

### Renderer layer

- Proposed new: `LegWheel/legwheel/visualization/primitives.py`
  - Renderer-neutral data structures for lines, points, meshes, frames, and annotations.
- Proposed new: `LegWheel/legwheel/visualization/plotly_renderer.py`
  - Converts primitives into Plotly traces and figures.
- Optional later: `LegWheel/legwheel/visualization/matplotlib_renderer.py`
  - Only needed if the Matplotlib side is also refactored into the same primitive pipeline.

### Plotly demos and tools

- Proposed new: `LegWheel/render/plotly_corgi_robot.py`
  - Static interactive Corgi robot HTML viewer.
- Proposed new: `LegWheel/examples/gait/csv_viewer_plotly.py`
  - Plotly-based CSV trajectory viewer with HTML export.
- Optional later:
  - `LegWheel/render/plotly_workspace_cloud.py`
  - `LegWheel/render/plotly_leg_envelope.py`

### CLI and skills

- `LegWheel/legwheel/cli.py`: add backend selection or a new render command after the Plotly viewer exists.
- `.agents/skills/legwheel-cli/SKILL.md`: update only after CLI behavior is implemented and verified.

---

## Optional dependency split candidates

This section records which packages could reasonably become optional extras after the Python `>=3.10` baseline upgrade. The current implementation only adds the `plotly` extra; the other splits should be handled separately because they can affect imports and CLI behavior.

### Keep in core dependencies

| Package | Recommendation | Reason |
|---|---|---|
| `numpy` | Keep core | Used across almost every model, planner, utility, and example. It is fundamental to kinematics and trajectory computation. |
| `scipy` | Keep core for now | `legwheel.models.leg_model` uses `scipy.optimize.fsolve`, and `legwheel.planners.com_stability` uses `ConvexHull` / `gaussian_filter1d`. Moving SciPy would require isolating optimization and stability features first. |
| `nlopt` | Keep core for now, candidate for `planning` later | `legwheel.bezier.swing` imports `nlopt` directly and swing planning is part of gait generation. It can become optional only if swing planning imports are made lazy or if a fallback planner is added. |

### Strong candidates for extras

| Package | Proposed extra | Reason | Required refactor |
|---|---|---|---|
| `matplotlib` | `viz-mpl` or `matplotlib` | Used primarily by `legwheel.visualization`, `render/`, and many example/demo scripts. Core FK/IK can run without it if plotting imports are isolated. | Move top-level plotting imports out of core modules such as `trajectory_planning.py`, `gait_generator.py`, `bezier/bezier.py`, and `com_stability.py`, or make plotting methods lazy-import Matplotlib. |
| `pandas` | `csv` or `export` | Used for CSV/dataframe export in gait utilities and legacy planners. Core kinematics does not require it. | Replace simple CSV writes with stdlib `csv` where practical, or lazy-import pandas only inside export methods. |
| `plotly[kaleido]` | `plotly` | Interactive visualization and static export support. Not required for core kinematics or trajectory generation. | Already added as an optional extra with `plotly[kaleido]>=6,<7`. Future Plotly renderer should handle missing extra with a clear error. |
| `jupyter` | `notebook` | Used for notebooks and exploratory analysis, not runtime. | Implemented: `jupyter>=1.0` is split from `dev` into the `notebook` extra. |

### Possible future extra layout

```toml
[project.optional-dependencies]
dev = [
    "pytest>=6.0",
    "black>=21.0",
    "flake8>=3.9",
]
notebook = [
    "jupyter>=1.0",
]
viz-mpl = [
    "matplotlib>=3.3.0",
]
plotly = [
    "plotly[kaleido]>=6,<7",
]
export = [
    "pandas>=1.1.0",
]
planning = [
    "nlopt>=2.6.0",
]
all = [
    "legwheel[dev,notebook,viz-mpl,plotly,export,planning]",
]
```

### Recommended staged split

1. Keep `numpy`, `scipy`, and `nlopt` in core for now.
2. Add only `plotly` extra first, because it is new and does not break existing imports. **Implemented.**
3. Split `jupyter` out of `dev` into `notebook`, because it is low risk and reduces dev install weight. **Implemented.**
4. Next candidate: split `pandas` into `export`, after CSV/export code paths use lazy imports or stdlib alternatives.
5. Hardest candidate: split `matplotlib` into `viz-mpl`, because current plotting imports are spread across library modules, examples, and `render/` scripts.
6. Defer `nlopt` split until planning APIs can handle missing optimization backend cleanly.

---

## Task 1: Add optional Plotly dependency

**Status:** Implemented on `feature/plotly-version-policy` after the Python `>=3.10` baseline commit.

**Files:**

- Modify: `LegWheel/pyproject.toml`
- Modify: `LegWheel/uv.lock`
- Modify: `LegWheel/README.md`

**Steps:**

1. Add a `plotly` optional dependency group with `plotly[kaleido]>=6,<7`.
2. Update the lock file with `uv sync --extra plotly`.
3. Document installation and a planned example command.

**Verification:**

```bash
cd LegWheel && uv run python -c "import plotly; print(plotly.__version__)"
cd LegWheel && uv run python -c "import kaleido"
```

**Risk:** Kaleido compatibility may differ by Plotly version.

**Mitigation:** Support HTML export first; add PNG/PDF export only after static export is verified.

---

## Task 2: Add renderer-neutral primitives

**Files:**

- Add: `LegWheel/legwheel/visualization/primitives.py`
- Add: `LegWheel/tests/test_visualization_primitives.py`

**Steps:**

1. Define minimal primitive classes such as `Line3D`, `Point3D`, `Mesh3D`, and `FrameAxes`.
2. Keep the API Python 3.7 compatible.
3. Validate array shapes and raise `ValueError` for invalid geometry.

**Verification:**

```bash
cd LegWheel && uv run pytest tests/test_visualization_primitives.py -q
```

**Risk:** Over-engineering the abstraction.

**Mitigation:** Only model primitives required by the first Corgi robot viewer.

---

## Task 3: Add Plotly renderer

**Files:**

- Add: `LegWheel/legwheel/visualization/plotly_renderer.py`
- Add: `LegWheel/tests/test_plotly_renderer.py`

**Steps:**

1. Convert primitives into Plotly `Scatter3d`, `Mesh3d`, or `Surface` traces.
2. Provide `make_figure(primitives, title=None)`.
3. Provide `save_html(fig, path, auto_open=False)`.
4. If Plotly is missing, raise a clear error explaining to install the Plotly extra.

**Verification:**

```bash
cd LegWheel && uv run pytest tests/test_plotly_renderer.py -q
cd LegWheel && uv run python -c "from legwheel.visualization.plotly_renderer import make_figure; print('ok')"
```

**Risk:** Plotly 3D aspect ratio may not match Matplotlib exactly.

**Mitigation:** Add visual comparison against the existing Matplotlib renderer during the spike.

---

## Task 4: Add static Plotly Corgi robot viewer

**Files:**

- Add: `LegWheel/render/plotly_corgi_robot.py`
- Reuse: `LegWheel/render/plot_corgi_robot.py`
- Reuse: `LegWheel/legwheel/models/corgi_leg.py`

**Steps:**

1. Build chassis wireframe primitives.
2. Use existing kinematics to generate four-leg linkage segments.
3. Add optional collision/contact markers after the base robot view works.
4. Support CLI arguments:
   - `--theta`
   - `--beta`
   - `--gamma`
   - `--html`
   - `--show` / `--no-show`
   - `--bounds`

**Verification:**

```bash
cd LegWheel && uv run python render/plotly_corgi_robot.py \
  --theta 75 --beta 0 --gamma 0 \
  --html outputs/plotly/corgi_robot.html \
  --no-show

cd LegWheel && test -s outputs/plotly/corgi_robot.html
```

Backward compatibility check:

```bash
cd LegWheel && uv run python render/plot_corgi_robot.py --theta 75 --no-show
```

**Risk:** Existing `plot_leg_3d(ax, ...)` is Matplotlib-bound.

**Mitigation:** Prefer existing geometry-returning helpers such as `get_detailed_linkage`; add a geometry helper only if necessary.

---

## Task 5: Add CLI backend selection

**Files:**

- Modify: `LegWheel/legwheel/cli.py`
- Add: `LegWheel/tests/test_cli_backend_selection.py`
- Modify: `LegWheel/README.md`
- Later with approval: `.agents/skills/legwheel-cli/SKILL.md`

**Steps:**

1. Add backend selection only after the Plotly script works.
2. Prefer a non-breaking command such as:

```bash
uv run legwheel render --backend plotly --theta 75 --html outputs/plotly/corgi_robot.html
```

3. Keep existing Matplotlib commands backward compatible.
4. Fix the existing `cmd_view` path if touching the viewer command, because the CSV viewer currently lives under `examples/gait/`.

**Verification:**

```bash
cd LegWheel && uv run pytest tests/test_cli_backend_selection.py -q
cd LegWheel && uv run legwheel --help
cd LegWheel && uv run legwheel render --help
```

**Risk:** CLI naming changes may disrupt existing workflows.

**Mitigation:** Add new commands or options first; avoid changing existing behavior until verified.

---

## Task 6: Add Plotly CSV trajectory viewer

**Files:**

- Add: `LegWheel/examples/gait/csv_viewer_plotly.py`
- Reuse: `LegWheel/examples/gait/csv_viewer.py`
- Later modify: `LegWheel/legwheel/cli.py`

**Steps:**

1. Reuse CSV parsing and hardware-to-kinematics order conversion.
2. Precompute foot traces as in the Matplotlib viewer.
3. Generate Plotly frames and slider controls.
4. Support:
   - `--html`
   - `--frame-step`
   - `--max-frames`
   - `--no-browser`

**Verification:**

Generate or select a hardware CSV, then run:

```bash
cd LegWheel && uv run python examples/gait/csv_viewer_plotly.py \
  outputs/csv/<generated>.csv \
  --html outputs/plotly/gait_viewer.html \
  --no-browser

cd LegWheel && test -s outputs/plotly/gait_viewer.html
```

**Risk:** Large Plotly HTML files for long trajectories.

**Mitigation:** Downsample frames by default and expose `--frame-step` / `--max-frames`.

---

## Task 7: Migrate selected workspace plots

**Files:**

- Optional add: `LegWheel/render/plotly_workspace_cloud.py`
- Optional add: `LegWheel/render/plotly_leg_envelope.py`

**Steps:**

1. Start with `plot_workspace_cloud.py`, because point clouds benefit most from Plotly interaction.
2. Keep existing Matplotlib scripts unchanged.
3. Add HTML export first.
4. Add static export only after Kaleido is verified.

**Verification:**

```bash
cd LegWheel && uv run python render/plotly_workspace_cloud.py \
  --html outputs/plotly/workspace_cloud.html \
  --no-show

cd LegWheel && test -s outputs/plotly/workspace_cloud.html
```

**Risk:** Browser lag with dense point clouds.

**Mitigation:** Add `--max-points` and sensible default sampling.

---

## Task 8: Update documentation and skill workflow

**Files:**

- Modify: `LegWheel/README.md`
- Modify: `LegWheel/examples/README.md`
- Modify only after behavior exists: `.agents/skills/legwheel-cli/SKILL.md`
- Add: `.changes/YYYY-MM-DD-plotly-backend.md`

**Steps:**

1. Document Plotly installation and backend usage.
2. Document Matplotlib vs Plotly responsibilities:
   - Matplotlib: static figures, publication-style plots, legacy compatibility.
   - Plotly: interactive 3D inspection, HTML sharing, trajectory viewer.
3. Update the `legwheel-cli` skill only after CLI behavior is implemented.
4. Add a change fragment documenting dependency and backend decisions.

**Verification:**

```bash
cd LegWheel && uv run pytest -q
cd LegWheel && black --line-length 100 legwheel/ tests/
cd LegWheel && flake8 legwheel/ tests/
```

---

## Final verification suite

```bash
cd LegWheel && uv run pytest -q
cd LegWheel && black --line-length 100 legwheel/ tests/
cd LegWheel && flake8 legwheel/ tests/
```

Plotly smoke test:

```bash
cd LegWheel && uv run python render/plotly_corgi_robot.py \
  --theta 75 --beta 0 --gamma 0 \
  --html outputs/plotly/corgi_robot.html \
  --no-show

cd LegWheel && test -s outputs/plotly/corgi_robot.html
```

Backward compatibility smoke tests:

```bash
cd LegWheel && uv run python render/plot_corgi_robot.py --theta 75 --no-show
cd LegWheel && uv run legwheel check --height 0.3 --vx 0.15 --gait Trot
cd LegWheel && uv run legwheel ik --leg 0 --x 0.2 --y 0.1 --z -0.25
```

---

## Recommended order

1. Add optional Plotly dependency.
2. Add renderer-neutral primitives.
3. Add Plotly renderer.
4. Add static Plotly Corgi robot HTML viewer.
5. Review generated HTML manually.
6. Add CLI backend integration.
7. Add Plotly CSV trajectory viewer.
8. Update docs and `legwheel-cli` skill.
9. Optionally migrate workspace/envelope plots.

---

## Non-goals for the first implementation

- Do not remove Matplotlib.
- Do not migrate all `render/` scripts at once.
- Do not use Plotly as the only thesis/paper figure backend.
- Do not add Dash server requirements initially.
- Do not support MP4/GIF export initially.
- Do not add Plotly as a mandatory runtime dependency; keep it optional even after the Python baseline upgrade.
