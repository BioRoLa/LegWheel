# LegWheel

A Python library for leg-wheel robot kinematics, trajectory planning, and 3D visualization,
designed for the Corgi quadruped wheel-leg robot platform.

> **Toolchain**: Uses [`uv`](https://github.com/astral-sh/uv) for fast, reproducible package management.
> Run all scripts with `uv run python <script>`.

---

## Features

| Category | Description |
|---|---|
| **3D Kinematics** | FK/IK with ABAD joints; frame transforms `{B}` → `{Mi}` → `{Li}` |
| **Trajectory Planning** | Stance/swing planning with automatic workspace-guard scaling |
| **Gait Generation** | Walk / Trot / Pace / Bound / Pronk — 12-DOF CSV export for hardware |
| **Collision Model** | 24-point bounding volume: chassis octagon + M6 studs + wheel rims |
| **Rendering** | 2D/3D mechanism plots, leg envelope, multi-view screenshots, animation |
| **CLI** | Rapid IK testing and gait safety validation without writing code |

---

## Installation

```bash
git clone https://github.com/BioRoLa/LegWheel.git
cd LegWheel
uv sync                                  # creates .venv, installs pinned deps
uv run python examples/kinematics/basic_usage.py
```

### Requirements
- Python >= 3.7
- NumPy, SciPy, Matplotlib, Pandas, Nlopt (pinned in `uv.lock`)
- **Optional**: [FFmpeg](https://ffmpeg.org/) — required for MP4 animation export

---

## CLI

```bash
# Verify gait parameters against physical constraints
uv run legwheel check --height 0.3 --vx 0.15 --gait Trot

# Inverse Kinematics for one leg
uv run legwheel ik --leg 0 --x 0.2 --y 0.1 --z -0.25

# Generate hardware-ready trajectory CSV
uv run legwheel generate --gait Walk --vx 0.1 --cycles 5
```

---

## Python API

```python
import numpy as np
from legwheel.models.corgi_leg import CorgiLegKinematics

leg = CorgiLegKinematics(0)                       # 0=FL, 1=FR, 2=RR, 3=RL
p_B = leg.forward_kinematics(np.deg2rad(75),      # θ
                              np.deg2rad(90),      # β
                              np.deg2rad(0))       # γ (ABAD)
q   = leg.inverse_kinematics(p_B)
```

```python
from legwheel.models.corgi_robot import CorgiRobot
from legwheel.models.collision_model import CorgiCollisionModel

robot = CorgiRobot()
robot.base_ori = [np.deg2rad(180), 0, 0]           # upside-down
col   = CorgiCollisionModel(robot)
min_z, pivots = col.find_contact_pivots([[np.deg2rad(60), np.deg2rad(90), 0.0]] * 4)
```

---

## Project Structure

```
LegWheel/
│
├── legwheel/                   # Core library (installed package)
│   ├── models/
│   │   ├── leg_model.py        # 2D five-bar linkage geometry
│   │   ├── corgi_leg.py        # 3D kinematics with ABAD
│   │   ├── corgi_robot.py      # Full 4-leg robot
│   │   └── collision_model.py  # 24-point bounding volume
│   ├── planners/               # Trajectory & gait generators
│   ├── visualization/          # 2D PlotLeg engine
│   ├── config/                 # RobotParams, GaitParams
│   └── cli.py
│
├── render/                     # Standalone rendering & animation scripts
│   ├── plot_corgi_robot.py     # Full robot 3D (+ collision bounds overlay)
│   ├── plot_single_leg.py      # Single-leg debug view
│   ├── plot_cone_assembly.py   # M6-stud + wheel cone geometry
│   ├── plot_leg_envelope.py    # Leg ABAD-sweep envelope (triple cone)
│   └── animation_*.py
│
├── examples/
│   ├── kinematics/             # FK/IK, workspace, frame validation
│   ├── gait/                   # Gait generation, CSV export, trajectory
│   ├── self_righting/          # Stability windows, collision analysis
│   └── stability/              # CoM & screw analysis
│
├── output/                     # Generated figures / CSVs (gitignored)
├── data/                       # Lookup tables
├── docs/                       # Derivation notes
└── uv.lock
```

---

## Key Parameters (`RobotParams`)

| Parameter | Value | Description |
|---|---|---|
| `WHEEL_BASE` | 510 mm | Front–rear wheel-centre distance |
| `BODY_WIDTH` | 240 mm | Hip-to-hip (ABAD axis) |
| `WHEEL_RADIUS_OUTER` | ≈135 mm | Physical outer radius |
| θ range | 17° – 160° | Extension (17° = wheel mode) |
| β range | ±40° | Sagittal swing |
| γ range | ±30° | ABAD (hip roll) |

---

## License

MIT License
