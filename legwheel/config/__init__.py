"""Configuration and path management for LegWheel package."""
from pathlib import Path
import os

# Project root directory
ROOT_DIR = Path(__file__).parent.parent.parent

# Package directory
PACKAGE_DIR = Path(__file__).parent.parent

# Data directories
DATA_DIR = ROOT_DIR / "data"
OUTPUT_DIR = ROOT_DIR / "output"
OUTPUT_CSV_DIR = OUTPUT_DIR / "csv"
OUTPUT_VIDEO_DIR = OUTPUT_DIR / "videos"
OUTPUT_PHASE_DIR = OUTPUT_DIR / "phase"

# Ensure output directories exist
for directory in [OUTPUT_DIR, OUTPUT_CSV_DIR, OUTPUT_VIDEO_DIR, OUTPUT_PHASE_DIR]:
    directory.mkdir(parents=True, exist_ok=True)

# Data files
R_G_VS_THETA_SW_CSV = DATA_DIR / "R_G_vs_theta_SW.csv"

# Default parameters
class RobotParams:
    """Default robot parameters."""
    # Body dimensions
    BODY_LENGTH = 0.444  # 44.4 cm
    BODY_HEIGHT = 0.2    # 20 cm
    BODY_WIDTH = 0.33    # 33 cm
    COM_BIAS = 0.0       # x bias of center of mass
    
    # Leg parameters
    WHEEL_RADIUS = 0.1   # 10 cm
    TIRE_RADIUS_SIM = 0.01   # 1 cm (simulation)
    TIRE_RADIUS_REAL = 0.019  # 1.9 cm (with tire)
    FOOT_OFFSET = 0.02225     # 22.25 mm
    
    # Angle limits
    MAX_THETA_DEG = 160.0
    MIN_THETA_DEG = 17.0
    THETA0_DEG = 17.0
    BETA0_DEG = 90.0

class TrajectoryParams:
    """Default trajectory parameters."""
    STAND_HEIGHT = 0.3
    STEP_LENGTH = 0.4
    STEP_HEIGHT = 0.04
    PERIOD = 2.4
    DT = 0.01
    DUTY = 0.25
    OVERLAP = 0.0
    VELOCITY = 0.1  # m/s

class GaitParams:
    """Default gait parameters."""
    WALK_GAIT = [4, 2, 3, 1]
    TROT_GAIT = [1, 3, 1, 3]
    SWING_TIME = 0.2
    SAMPLING_RATE = 1000  # Hz
