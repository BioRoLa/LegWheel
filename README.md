# LegWheel

A Python library for leg-wheel robot kinematics, trajectory planning, and 3D visualization, specifically optimized for the Corgi robot platform.

## Features

- **3D Kinematics**: Robust forward and inverse kinematics for the Corgi leg-wheel module with ABAD (Abduction/Adduction) joints.
- **Detailed 2D/3D Plotting**: Geometric projection API that translates detailed 2D linkage designs into 100% consistent 3D robot visualizations.
- **Trajectory Planning**: Intelligent stance and swing phase planning with customizable duty cycles and overlap.
- **Gait Generation**: Coordinated multi-leg gait sequences (Walk, Trot, Pace, etc.) with CSV/Pandas export.
- **Animation Suite**: Tools for generating range-of-motion demonstrations and multi-perspective videos (Front, Side, Ortho).

## Installation

### From Source (Development)

```bash
# Clone the repository
git clone https://github.com/BioRoLa/LegWheel.git
cd LegWheel

# Install in editable mode
pip install -e .
```

### Requirements
- Python >= 3.8
- NumPy, SciPy, Matplotlib, Pandas
- **Optional**: [FFmpeg](https://ffmpeg.org/) (Required for saving animations as MP4. Fallback to Pillow for GIFs).

## Quick Start

### 2D Leg Kinematics
```python
import numpy as np
from legwheel.models.leg_model import LegModel

# Create a leg model
leg = LegModel()

# Forward kinematics
theta = np.deg2rad(110)
beta = np.deg2rad(10)
leg.forward(theta, beta)
print(f"Foot position: {leg.G}")
```

### 3D Robot Visualization
```python
from render.plot_corgi_robot import plot_corgi_robot
import numpy as np

# Plot the Corgi robot in a specific pose
plot_corgi_robot(
    theta=np.deg2rad(110), 
    beta=np.deg2rad(10), 
    gamma=np.deg2rad(15)
)
```

## Project Structure

```
LegWheel/
├── legwheel/           # Main package
│   ├── models/         # 2D/3D Kinematic models (CorgiLegKinematics)
│   ├── planners/       # Trajectory and gait generation
│   ├── visualization/  # 2D Plotting utilities (PlotLeg)
│   ├── config/         # Centralized RobotParams and GaitParams
│   └── utils/          # Solver and geometric helpers
├── render/             # 3D Rendering and Animation scripts
├── examples/           # Derived ICRA examples and IK tests
├── output/             # Development notes and generated videos
├── data/               # Model coefficients and datasets
└── docs/               # Technical documentation
```

## Documentation

For a detailed breakdown of the 3D coordinate system, transformation logic, and ABAD joint implementation, please see:
[Development Notes: ABAD 3D Kinematics](./output/Development_Notes_ABAD_3D.md)

## License

MIT License

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.