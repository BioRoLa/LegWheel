# LegWheel

A Python library for leg-wheel robot control and trajectory planning.

## Installation

### From Source (Development)

```bash
# Clone the repository
git clone https://github.com/yourusername/legwheel.git
cd legwheel

# Install in editable mode
pip install -e .

# Or with development dependencies
pip install -e ".[dev]"
```

### Using pip (once published)

```bash
pip install legwheel
```

## Quick Start

```python
import legwheel
from legwheel import LegModel, TrajectoryPlanner

# Create a leg model
leg = LegModel(sim=True)

# Forward kinematics
import numpy as np
theta = np.deg2rad(90)
beta = np.deg2rad(30)
leg.forward(theta, beta)
print(f"Foot position: {leg.G}")

# Plan trajectory
traj = TrajectoryPlanner(
    stand_height=0.3,
    step_length=0.4,
    leg=leg,
)
traj.move()
```

## Features

- **Leg Kinematics**: Forward and inverse kinematics for leg-wheel mechanisms
- **Trajectory Planning**: Generate smooth trajectories for walking gaits
- **Gait Generation**: Support for walk, trot, and custom gaits
- **Visualization**: Real-time visualization of leg movements
- **Bezier Curves**: Optimized swing phase trajectories

## Documentation

See the `docs/` directory for detailed documentation and examples.

## Project Structure

```
legwheel/
├── legwheel/           # Main package
│   ├── models/         # Kinematic models
│   ├── planners/       # Trajectory and gait planning
│   ├── visualization/  # Plotting utilities
│   ├── utils/          # Helper functions
│   ├── bezier/         # Bezier curve tools
│   └── config.py       # Configuration
├── scripts/            # Executable scripts
├── examples/           # Usage examples
├── tests/              # Unit tests
├── data/               # Data files
└── docs/               # Documentation
```

## Requirements

- Python >= 3.7
- NumPy >= 1.19.0
- Matplotlib >= 3.3.0
- SciPy >= 1.5.0
- Pandas >= 1.1.0
- NLopt >= 2.6.0

## License

MIT License

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
