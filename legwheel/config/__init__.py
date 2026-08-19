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
    # CAVEAT (log section 72, 2026-08-18): this toroidal cross-section and
    # slip_rf_cambered's flat-band one (flat 5 mm + corner 15 mm) are two
    # SEMANTICALLY DIFFERENT models of the same tread, and neither matches
    # the simulator, whose contact runs on a knobby (real tread texture,
    # +-4 mm relief) auto-decimated (~5 mm facet) mesh. Measured contact
    # behaviour in the sim, wheel mode, kp 500 (n = 3, sections 65/70/72):
    # ride drop follows A*(1 - cos(lean)) with A_ROLL ~= 0.0247 m (6.9% rms)
    # and A_HOLD ~= 0.0351 m (14.4% rms) -- an EFFECTIVE transverse crown of
    # 25-35 mm, state-dependent, against the 130 mm either model implies.
    # Use the empirical law for sim-facing predictions; keep the geometric
    # models for design reasoning.
    DROP_CROWN_ROLL_SIM = 0.0247    # m, empirical A in drop = A*(1-cos(lean)), rolling
    DROP_CROWN_HOLD_SIM = 0.0351    # m, empirical A, static hold

    # Wheel-mode closure calibration (log sections 66-71, 2026-08-18).
    # The linkage closes the wheel concentric at THETA 17.00 deg BY DESIGN
    # (closed_wheel_eccentricity(17 deg) = 0), but closure error costs
    # ~0.9 mm/deg of k=1 eccentricity (envelope model; ~0.5 measured), the
    # sim's theta loop sags ~0.85 deg below command while rolling, and the
    # sim's proto closes ~1.0 deg from design. Closure angle is therefore a
    # PER-ROBOT CALIBRATION (V-curve procedure, section 71), not a constant.
    WHEEL_CLOSURE_THETA_SIM_DEG = 18.04   # achieved theta minimizing k=1, this sim
    WHEEL_CLOSURE_CMD_SIM_DEG = 18.85     # command that lands it at kp 500
    WHEEL_ECC_FLOOR_SIM = 0.00036         # m, residual e at the calibrated closure

    # Effective FORWARD rolling radius, sim wheel mode (stage15 z_leg fit on
    # the Stage 1 corpus, 180k contact samples): the wheel advances as if
    # r = 0.14482 at EVERY lean (0-40 deg) and every kp (90-1000) -- the
    # geometric rolling_radius(lambda) does NOT describe the sim's forward
    # channel under lean (knobby decimated tread again, section 73/75; the
    # design formula would be 23% low at 40 deg). ROLL state; measured at
    # the default theta command (uncalibrated closure, achieved ~16.2 deg).
    WHEEL_ROLL_RADIUS_SIM = 0.14482       # m, lambda- and kp-independent

    # Camber-thrust lateral slip, sim wheel mode (stage15 --thrust-fit;
    # two-parameter law of section 81, superseding section 78/79's
    # origin-forced fit whose constants were SLOPE 0.0038 / SAT 0.0022):
    # v_slip = OFFSET + min(SLOPE * lean, SAT), toward the lean, ROLL
    # state. The offset is the lean-independent lateral bias the
    # origin-forced slope had partially absorbed (U1: +0.549 mm/s at
    # kp 500, 43 sigma over run scatter; +1.54 mm/s at kp 90). Constants
    # are kp-TAGGED per U5's registered failure clause: with per-kp
    # offsets separated, the lambda-20 kp-ladder points sit at
    # 0.46x/1.02x/1.32x of the kp-500 line (kp 90/250/1000, n = 1 each)
    # -- there is no kp-independent slope. Fit rms 0.13 mm/s = 8.5% of
    # the lambda-40 value (U3 bar 10%); the hard-knee min() form beats
    # tanh (0.23 mm/s). yaw ~ 0 so this is slip, not turning.
    CAMBER_THRUST_OFFSET_KP500_SIM = 0.00055   # m/s, lean-independent bias
    CAMBER_THRUST_SLOPE_KP500_SIM = 0.0030     # m/s per rad of lean
    CAMBER_THRUST_SAT_KP500_SIM = 0.0016       # m/s, saturation above ~30 deg
    
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
