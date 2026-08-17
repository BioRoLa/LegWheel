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
    
    WHEEL_BASE = 0.510              # Distance between front and rear wheel centers
    BODY_WIDTH = 0.240              # Hip-to-hip distance
    
    # Leg & Wheel Configuration
    ABAD_AXIS_OFFSET = 0.057166     # Offset from Hip Roll axis to Leg Pitch plane
    WHEEL_AXIAL_OFFSET = 0.091675   # Lateral offset from leg plane to wheel center
    WHEEL_RADIUS_PITCH = 0.100      # Effective radius for kinematics (R: linkage joint circle)
    WHEEL_THICKNESS    = 0.04       # Thickness of the wheel (for collision and visualization)

    # Tire geometry (toroidal cross-section)
    TIRE_RIM_OFFSET    = 0.010      # Hard rim radial thickness beyond R (R → hard rim outer edge)
    TIRE_TREAD_RADIUS  = 0.130      # Torus major radius: tread arc center = R + TIRE_RIM_OFFSET + 0.020
    TIRE_CORNER_RADIUS = 0.015      # Torus minor radius (corner fillet); max contact = TIRE_TREAD_RADIUS + TIRE_CORNER_RADIUS
    WHEEL_RADIUS_OUTER = TIRE_TREAD_RADIUS + TIRE_CORNER_RADIUS  # = 0.145, physical outer radius (collision)
    
    # Center of Mass (COM) Biases
    COM_BIAS = 0.0                  # x bias of center of mass
    COM_BIAS_X = 0.0                # x bias of center of mass
    COM_BIAS_Y = 0.0                # y bias of center of mass
    COM_BIAS_Z = ABAD_AXIS_OFFSET   # z bias of center of mass

    # ------------------------------------------------------------------
    # Whole-body inertia about the CoM, body axes (+x fwd, +y left, +z up).
    #
    # PROVENANCE (recorded deliberately -- this project has been bitten twice by
    # physical constants with no source: the 35 N.m clamp and the absent joint
    # limits). Derived 2026-08-17 by examples/gslip/body_inertia.py, which sums
    # the 65 `physics Physics` blocks of corgi_sim/protos/CorgiRobotABAD.proto
    # with parallel-axis transport. The workspace URDF carries no <inertial>
    # blocks at all, so the proto is the only source.
    #
    # Corroboration: the same pass returns 31.04 kg against a 30.0 kg scale
    # reading and a 30.84 kg previous sim count, a CoM 0.34 mm off the wheelbase
    # centre, and lateral balance exact to 1e-9 m -- all matching the measured
    # mass distribution. Ixx < Iyy < Izz and Izz ~= Ixx + Iyy (1.873 vs 1.970),
    # as a flat-ish body requires.
    #
    # CAVEAT 1: the four *_LEG solids in the proto carry an inertiaMatrix that is
    # physically impossible (0.05 kg with a 4.89 m radius of gyration). Their own
    # inertia is DISCARDED here; their mass and position are kept. Uncorrected,
    # the diagonal reads [1.7218, 6.1497, 5.7435]. See the implementation log.
    #
    # CAVEAT 2: composite inertia is configuration-dependent and the proto encodes
    # the joints at their zero pose, not the theta ~ 100 deg nominal stance. The
    # dominant roll term is m*dy^2 for the four 4.7 kg modules, whose lateral
    # offsets do not move with theta, so I_ROLL is robust; treat the third
    # significant figure as soft.
    BODY_MASS_SIM = 31.0371         # kg, summed from the proto
    I_ROLL = 0.611906               # kg m^2 about +x (fore-aft) -- Stage 2b / BIP
    I_PITCH = 1.358494              # kg m^2 about +y
    I_YAW = 1.872999                # kg m^2 about +z
    # Chang 2022's dimensionless body inertia, on the CONTACT half-track
    # (0.4234/2), not the 0.240 m hip spacing.
    J_TILDE_ROLL = 0.4399
    
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

    # Workspace Guard Constants  only used for trajectory planning and velocity limiting
    BETA_MAX_DEG = 40.0         # Sagittal swing geometric limit (°)
    # 70 deg CONFIRMED SAFE (2026-08-12). The hardware can probably exceed it;
    # 70 is set deliberately below the true ABAD range, so the conservatism lives
    # in this value rather than in a gap between the two constants below. That is
    # why GUARD == MAX is correct here and not an oversight.
    #
    # Provenance, because it was not obvious: both were raised 30.0 -> 70.0 in
    # 680aefa, a commit about the swing acceleration budget whose message does
    # not mention them, and which deleted the guard's sizing rationale ("30.27,
    # sized so vy=0.6 @ h=0.30/T=1.0 sits on the boundary"). That silently broke
    # the two lateral-guard tests, which had hard-coded the resulting 66.7%
    # downscale -- they now derive it from GAMMA_GUARD_DEG instead.
    #
    # Note neither value is recoverable from the codebase: the corgi_sim proto's
    # ABAD HingeJoint carries no minPosition/maxPosition and motor_config.yaml
    # gives directions only. Changing GAMMA_MAX_DEG is a hardware claim -- say
    # where the number came from.
    GAMMA_MAX_DEG = 70.0        # ABAD lateral sweep geometric limit (°)
    GAMMA_GUARD_DEG = 70.0      # Velocity guard limit (°)
    GAMMA_FLOOR_DEG = 1.0       # Lateral one-sided sweep floor (°): liftoff ABAD tilt kept this far
                                # from upright so the loaded wheel never crosses gamma=0 (no contact
                                # edge / center-of-pressure flip mid-stance).
    STEP_DECAY_COEFF = 0.3      # Step height linear decay coefficient (was 0.8→0.5→0.3)
    STEP_FLOOR = 0.2            # Minimum step height scale lower bound
    STEP_USAGE_THRESHOLD = 0.15 # Deadband: no scaling when workspace usage < 15%

    # Touchdown velocity targets — tune to reduce body bounce at landing
    TOUCHDOWN_VEL_H_MAX = 0.3   # m/s: horizontal cap; prevents swing_delta/T_sw overshoot
    TOUCHDOWN_VEL_Z_SCALE = 0.1 # vertical = -2*step_h/T_sw * scale; 0.1 → ~gentle descent (legacy, unused by accel model)

    # Swing acceleration budget — unified a_max (m/s²) for liftoff/touchdown velocity design.
    # Feasibility constraint: SWING_ACCEL_MAX >= 8 * step_height / T_sw²
    # Example: Walk h=0.04, T_sw=0.25 → a_min = 5.12 m/s²; Trot T_sw=0.20 → 8.0 m/s²
    # At a_max=10: peak joint acc ≈ a_max / J_x = 10/0.019 ≈ 526 rad/s² (vs 5000+ in old model).
    SWING_ACCEL_MAX = 10.0      # m/s²: liftoff/touchdown Cartesian acceleration budget

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
