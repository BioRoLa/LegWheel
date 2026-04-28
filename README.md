# LegWheel

A Python library for leg-wheel robot kinematics, trajectory planning, 3D visualization, and self-righting analysis, specifically designed for the Corgi quadruped wheel-leg robot platform.

> **Toolchain**: Uses [`uv`](https://github.com/astral-sh/uv) for fast, reproducible package management.
> All example commands should be run as `uv run python <script>`.

---

## Features

| Category | Description |
|---|---|
| **3D Kinematics** | FK/IK for the Corgi leg-wheel module with ABAD joints, coordinate frame transforms `{B}` → `{Mi}` → `{Li}` |
| **Trajectory Planning** | Stance/swing phase planning with automatic workspace-guard scaling |
| **Gait Generation** | Walk, Trot, Pace, Bound, Pronk — 12-DOF CSV export for hardware |
| **Collision Model** | 24-point bounding volume: chassis octagon (16 pts) + M6 studs (4) + wheel rims (4) |
| **Self-Righting Analysis** | Stability window sweep, potential energy landscape, minimum angular impulse estimation |
| **Cone Geometry** | M6-stud + wheel disc-cone assembly visualization and contact analysis |
| **Rendering** | Detailed 2D/3D mechanism plots, multi-view screenshots, animation export |
| **CLI** | Built-in commands for rapid IK testing and gait safety validation |

---

## Installation

```bash
# Clone the repository
git clone https://github.com/BioRoLa/LegWheel.git
cd LegWheel

# Sync environment with uv (creates .venv automatically)
uv sync

# Run any script
uv run python examples/basic_usage.py
```

### Requirements
- Python >= 3.7
- NumPy, SciPy, Matplotlib, Pandas, Nlopt (pinned in `uv.lock`)
- **Optional**: [FFmpeg](https://ffmpeg.org/) — required for MP4 animation export

---

## Command-Line Interface (CLI)

```bash
# Verify gait parameters against physical constraints
uv run legwheel check --height 0.3 --vx 0.15 --vy 0.0 --gait Trot

# Inverse Kinematics (IK) for a single leg
uv run legwheel ik --leg 0 --x 0.2 --y 0.1 --z -0.25

# Generate hardware-ready trajectory CSV
uv run legwheel generate --gait Walk --vx 0.1 --cycles 5 --outdir output/csv
```

---

## Python API

### Forward / Inverse Kinematics

```python
import numpy as np
from legwheel.models.corgi_leg import CorgiLegKinematics

leg = CorgiLegKinematics(0)   # 0=FL, 1=FR, 2=RR, 3=RL

# Forward kinematics → [x, y, z] in Body Frame {B}
p_B = leg.forward_kinematics(np.deg2rad(75), np.deg2rad(90), np.deg2rad(0))

# Inverse kinematics
q = leg.inverse_kinematics(p_B)
print(f"θ={np.deg2rad(q[0]):.1f}°, β={np.deg2rad(q[1]):.1f}°, γ={np.deg2rad(q[2]):.1f}°")
```

### Collision / Bounding-Volume Model

```python
from legwheel.models.corgi_robot import CorgiRobot
from legwheel.models.collision_model import CorgiCollisionModel

robot = CorgiRobot()
robot.base_ori = [np.deg2rad(180), 0, 0]   # Upside-down
col   = CorgiCollisionModel(robot)

q_nom = [np.deg2rad(60), np.deg2rad(90), 0.0]
pts   = col.get_all_collision_points([q_nom]*4)   # chassis / m6_studs / wheels

min_z, pivots = col.find_contact_pivots([q_nom]*4)
print("Ground contact features:", [p["label"] for p in pivots])
```

### 3D Robot Visualization

```python
from render.plot_corgi_robot import plot_corgi_robot
plot_corgi_robot(theta=np.deg2rad(75), beta=np.deg2rad(90), gamma=np.deg2rad(0))
```

---

## Project Structure

```
LegWheel/
│
├── legwheel/                  # Core library (importable package)
│   ├── models/
│   │   ├── leg_model.py       # 2D five-bar linkage geometry (LegModel)
│   │   ├── corgi_leg.py       # 3D kinematics with ABAD (CorgiLegKinematics)
│   │   ├── corgi_robot.py     # Full 4-leg robot model (CorgiRobot)
│   │   └── collision_model.py # 24-point bounding volume (CorgiCollisionModel)
│   ├── planners/              # Trajectory & gait generators
│   ├── visualization/         # 2D PlotLeg engine
│   ├── config/                # RobotParams, TrajectoryParams, GaitParams
│   └── cli.py                 # CLI entry point
│
├── render/                    # Standalone rendering scripts (not a package)
│   ├── plot_corgi_robot.py    # Full robot 3D plot (with collision bounds)
│   ├── plot_single_leg.py     # Single-leg debug plot
│   ├── plot_cone_assembly.py  # M6-stud + wheel cone geometry analysis
│   ├── animation_corgi_robot.py
│   ├── animation_multi_view.py
│   └── debug_views.py
│
├── examples/                  # Runnable scripts grouped by topic
│   │
│   ├── — Kinematics & Workspace —
│   ├── basic_usage.py
│   ├── test_ik.py
│   ├── test_robot.py
│   ├── validate_corgi_leg_transforms.py
│   ├── detailed_leg_plot.py
│   ├── check_parameters.py
│   │
│   ├── — Gait & Trajectory —
│   ├── test_4leg_gait.py
│   ├── test_full_gait_cycle.py
│   ├── test_gait_generator_3d.py
│   ├── swing_trajectory_analysis.py
│   ├── test_swing_trajectory.py
│   ├── test_stance_trajectory.py
│   ├── test_stance_rt_solver.py
│   ├── test_lateral_stance.py
│   ├── generate_hardware_csv.py
│   ├── gait_command_export.py
│   ├── generate_csv_ui.py
│   ├── csv_viewer.py
│   │
│   ├── — Self-Righting Analysis —
│   ├── stability_analysis.py       ← stability windows + energy barrier
│   ├── plot_window_states.py       ← CLI: multi-view screenshots per window
│   ├── plot_collision_model.py     ← 3D fallen-state visualization
│   ├── test_self_righting_collision.py
│   │
│   └── — Stability / COM —
│       ├── test_com_stability.py
│       └── test_screw.py
│
├── output/                    # Generated figures, CSVs, videos (gitignored)
│   └── window_states/         # Stability window screenshots
├── data/                      # Lookup tables (R_G_vs_theta_SW.csv)
├── docs/                      # Technical documentation & derivation notes
└── uv.lock                    # Pinned dependency lockfile
```

---


## License

MIT License

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.