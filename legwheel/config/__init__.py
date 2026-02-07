"""Configuration and path management for LegWheel package."""
from pathlib import Path
import os

# Project root directory
ROOT_DIR = Path(__file__).parent.parent.parent

# Package directory
PACKAGE_DIR = Path(__file__).parent.parent

# Data directories
DATA_DIR = ROOT_DIR / "data"
DOCS_DIR = ROOT_DIR / "docs" / "Docs"
OUTPUT_DIR = ROOT_DIR / "output"
OUTPUT_CSV_DIR = OUTPUT_DIR / "csv"
OUTPUT_VIDEO_DIR = OUTPUT_DIR / "videos"
OUTPUT_PHASE_DIR = OUTPUT_DIR / "phase"

# Ensure output directories exist
for directory in [OUTPUT_DIR, OUTPUT_CSV_DIR, OUTPUT_VIDEO_DIR, OUTPUT_PHASE_DIR]:
    directory.mkdir(parents=True, exist_ok=True)

# Data files
R_G_VS_THETA_SW_CSV = DATA_DIR / "R_G_vs_theta_SW.csv"

# Documentation files
COORD_DEFINITIONS_PDF = DOCS_DIR / "Coord definitions.pdf"
FK_IK_PDF = DOCS_DIR / "FK & IK.pdf"
SYSTEM_PARAMETERS_PDF = DOCS_DIR / "System_Parameters.pdf"

# Archived Parameters (Old Design)
class OLD_Design:
    """Archived parameters for the previous robot design."""
    class RobotParams:
        # Body dimensions
        BODY_LENGTH = 0.444  # 44.4 cm
        BODY_HEIGHT = 0.2    # 20 cm
        BODY_WIDTH = 0.33    # 33 cm
        COM_BIAS = 0.0       # x bias of center of mass
        
        # Leg parameters
        WHEEL_RADIUS = 0.1   # 10 cm
        TIRE_RADIUS_REAL = 0.019  # 1.9 cm (with tire)
        FOOT_OFFSET = 0.02225     # 22.25 mm
        TYRE_THICKNESS = 0.01225  # 12.25 mm
        
        # Linkage parameters
        ARC_HF_DEG = 130.0
        ARC_BC_DEG = 101.0
        L1_RATIO = 0.8  # l1: OA = 0.8 * R
        L5_RATIO = 0.9  # l5: AD = 0.9 * R
        L6_RATIO = 0.4  # l6: DE = 0.4 * R
        
        # Calculated dimensions (standard Corgi)
        FOOT_RADIUS = 0.1345  # 134.5 mm
        
        # Angle limits
        MAX_THETA_DEG = 160.0
        MIN_THETA_DEG = 17.0
        THETA0_DEG = 17.0
        BETA0_DEG = 90.0

    class TrajectoryParams:
        STAND_HEIGHT = 0.3
        STEP_LENGTH = 0.4
        STEP_HEIGHT = 0.04
        PERIOD = 2.4
        DT = 0.01
        DUTY = 0.25
        OVERLAP = 0.0
        VELOCITY = 0.1  # m/s

    class GaitParams:
        WALK_GAIT = [4, 2, 3, 1]
        TROT_GAIT = [1, 3, 1, 3]
        PACE_GAIT = [1, 3, 3, 1]
        BOUND_GAIT = [1, 1, 3, 3]
        PRONK_GAIT = [1, 1, 1, 1]
        SWING_TIME = 0.2
        SAMPLING_RATE = 1000  # Hz

# Default parameters (Current Corgi Design)
class RobotParams:
    """Current robot parameters based on System_Parameters.pdf."""
    # Chassis & Body Dimensions
    CHASSIS_LENGTH = 0.694
    CHASSIS_WIDTH = 0.352
    CHASSIS_HEIGHT = 0.138
    
    WHEEL_BASE = 0.510  # Distance between front and rear wheel centers
    BODY_WIDTH = 0.240  # Hip-to-hip distance
    
    COM_BIAS = 0.0       # x bias of center of mass
    
    # Leg & Wheel Configuration
    ABAD_AXIS_OFFSET = 0.057166   # Offset from Hip Roll axis to Leg Pitch plane
    WHEEL_AXIAL_OFFSET = 0.091675 # Lateral offset from leg plane to wheel center
    WHEEL_RADIUS_PITCH = 0.100    # Effective radius for kinematics
    WHEEL_RADIUS_OUTER = 0.135    # Physical outer radius (collision)
    WHEEL_THICKNESS    = 0.04     # Thickness of the wheel (for collision and visualization)
    
    # Linkage parameters (Standard ratios)
    ARC_HF_DEG = 130.0
    ARC_BC_DEG = 101.0
    L1_RATIO = 0.8  # l1: OA = 0.8 * WHEEL_RADIUS_PITCH
    L5_RATIO = 0.9  # l5: AD = 0.9 * WHEEL_RADIUS_PITCH
    L6_RATIO = 0.4  # l6: DE = 0.4 * WHEEL_RADIUS_PITCH
    
    # Angle limits
    MAX_THETA_DEG = 160.0
    MIN_THETA_DEG = 17.0
    THETA0_DEG = 17.0
    BETA0_DEG = 90.0

class TrajectoryParams:
    """Current trajectory parameters."""
    STAND_HEIGHT = 0.3
    STEP_LENGTH = 0.4
    STEP_HEIGHT = 0.04
    PERIOD = 2.4
    DT = 0.01
    DUTY = 0.25
    OVERLAP = 0.0
    VELOCITY = 0.1  # m/s

class GaitParams:
    """Current gait parameters."""
    WALK_GAIT = [4, 2, 3, 1]
    TROT_GAIT = [1, 3, 1, 3]
    PACE_GAIT = [1, 3, 3, 1]
    BOUND_GAIT = [1, 1, 3, 3]
    PRONK_GAIT = [1, 1, 1, 1]
    SWING_TIME = 0.2
    SAMPLING_RATE = 1000  # Hz
