# LegWheel

A Python library for leg-wheel robot kinematics, trajectory planning, and 3D visualization, specifically optimized for the Corgi robot platform.

## Features

- **3D Kinematics**: Robust forward and inverse kinematics for the Corgi leg-wheel module with ABAD (Abduction/Adduction) joints.
- **Trajectory Planning**: Intelligent stance and swing phase planning with automatic workspace guard scaling.
- **Gait Generation**: Coordinated multi-leg gait sequences (Walk, Trot, Pace, Bound, Pronk) with 12-DOF joint command CSV export.
- **Detailed 2D/3D Plotting**: Geometric projection API that translates detailed 2D linkage designs into consistent 3D robot visualizations.
- **Command-Line Interface (CLI)**: Built-in tools for rapid kinematics testing and gait safety validation without writing code.

## Installation

LegWheel can be installed locally in editable mode. We recommend using a dedicated `conda` environment.

```bash
# Clone the repository
git clone https://github.com/BioRoLa/LegWheel.git
cd LegWheel

# Install in editable mode
pip install -e .
```

### Requirements
- Python >= 3.7
- NumPy, SciPy, Matplotlib, Pandas, Nlopt
- **Optional**: [FFmpeg](https://ffmpeg.org/) (Required for saving animations as MP4).

## Command-Line Interface (CLI)

Installing the package automatically registers the `legwheel` command in your environment. Use `legwheel --help` to see all available options.

### 1. Gait Parameter Verification
Check if specific gait parameters (velocity, height, step clearance) violate the physical leg constraints or motor limits:
```bash
legwheel check --height 0.3 --vx 0.15 --vy 0.0 --gait Trot
```

### 2. Inverse Kinematics (IK)
Calculate the joint angles (theta, beta, gamma) required to reach a specific foot target position in the body frame `{B}`:
```bash
legwheel ik --leg 0 --x 0.2 --y 0.1 --z -0.25
```

### 3. Hardware CSV Generation
Generate a trajectory CSV for real-world hardware tracking (12-DOF motor commands):
```bash
legwheel generate --gait Walk --vx 0.1 --cycles 5 --outdir outputs/csv
```

## Python API Usage

The library can also be directly imported for custom scripting or integration into other frameworks (e.g., ROS 2 nodes).

### Inverse Kinematics
```python
import numpy as np
from legwheel.models.corgi_leg import CorgiLegKinematics

# Initialize the Front-Left leg (Index 0)
leg = CorgiLegKinematics(0)

# Calculate joint angles for target [x, y, z] in {B}
q = leg.inverse_kinematics([0.2, 0.1, -0.25])
print(f"Joint Angles (rad): {q}")
```

### 3D Robot Visualization
```python
import numpy as np
from render.plot_corgi_robot import plot_corgi_robot

plot_corgi_robot(
    theta=np.deg2rad(110), 
    beta=np.deg2rad(10), 
    gamma=np.deg2rad(15)
)
```

## Project Structure

```
LegWheel/
├── legwheel/           # Core library
│   ├── models/         # 2D/3D Kinematic models (CorgiLegKinematics)
│   ├── planners/       # Trajectory and gait generators
│   ├── visualization/  # Plotting utilities
│   ├── config/         # RobotParams and Gait parameters
│   └── cli.py          # Command-line interface entry point
├── render/             # 3D Rendering and Animation scripts
├── examples/           # Example scripts and IK tests
├── tests/              # Unit tests
├── outputs/            # Generated data (videos, CSVs)
└── docs/               # Technical documentation
```

## Documentation

For a detailed breakdown of the 3D coordinate system, transformation logic, and ABAD joint implementation, please see:
[Development Notes: ABAD 3D Kinematics](./output/Development_Notes_ABAD_3D.md)

## License

MIT License

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.