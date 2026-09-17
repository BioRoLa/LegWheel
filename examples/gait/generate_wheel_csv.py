"""
Wheel-Mode CSV Generator for CorgiRobot 12-DOF Rolling Trajectories.

Generates a CSV of pure rolling motion: theta is held at the wheel-mode fold
angle (17 deg) while beta integrates continuously, so the leg-wheel rolls like
a wheel. Output matches ``generate_hardware_csv.py`` exactly:
- 12 columns, NO header, dt = 0.001 s.
- Order:
  [FL_theta, FL_beta, FR_theta, FR_beta, RR_theta, RR_beta, RL_theta, RL_beta,
   FL_gamma, FR_gamma, RR_gamma, RL_gamma]

Sign convention
---------------
All four legs carry the SAME beta sign. The left/right mirroring is applied
downstream — by ``corgi_ros_bridge`` (real_motor_config.yaml) on hardware and by
``corgi_driver.py`` (motor_config.yaml) in simulation. This matches what
``GaitGenerator3D`` emits, so wheel and gait CSVs share one convention. Do NOT
copy the per-leg flip found in ``wheeled_gen.cpp``; that node predates the
bridge-level direction config and now double-flips.

Beta is deliberately NOT range-checked: continuous rolling requires beta to grow
without bound (many revolutions). Only theta and gamma are validated. The bridge
clamps theta to [17, 160] deg and passes beta through untouched, and the Webots
RotationalMotors declare no position limits, so unbounded beta is safe on both
paths.
"""

import argparse
import os

import numpy as np

from legwheel.config import RobotParams

# ─────────────────────────────────────────────────────────────────────────────
# Hardware calibration — edit these for the machine you are targeting.
# ─────────────────────────────────────────────────────────────────────────────
# Fold angle at which the leg closes into a round wheel. In simulation the
# geometry is exact and the nominal 17 deg (RobotParams.THETA0_DEG) is right; a
# physical leg may need a slightly larger angle to close the rim gap, so measure
# it on the real robot and set it here. The --theta flag overrides this per run.
#   simulation : 17.0
#   real robot : measure it (18.0 is a common value once tolerance is taken up)
WHEEL_THETA_DEG = 17.0

# The mechanical home the motors are zeroed at — where the prep segment starts
# and ramps from. This is a property of the machine's zeroing, not of the wheel
# pose, so it stays at the nominal value even when WHEEL_THETA_DEG is retuned.
HOME_THETA_DEG = RobotParams.THETA0_DEG

# Note: the rolling radius is NOT derived from these angles. The rim arc radius
# is fixed by the tyre geometry, so retuning the fold angle does not change it.
# If the real wheel rolls a different distance per revolution than the model
# predicts, calibrate that separately with the -R flag.

LEG_NAMES = ("FL", "FR", "RR", "RL")
LEFT_LEGS = (0, 3)  # FL, RL  -> modules A, D
RIGHT_LEGS = (1, 2)  # FR, RR  -> modules B, C

PROGRESS_PREFIX = "::progress::"


def _emit_progress(percent):
    """Emit a machine-readable progress percentage for TUI consumers."""
    percent = max(0, min(100, int(round(percent))))
    print("{}{}".format(PROGRESS_PREFIX, percent), flush=True)


def _phase_header() -> str:
    return "FL_Phase,FR_Phase,RR_Phase,RL_Phase"


def validate_wheel_pose(theta_deg, gamma_deg):
    """Validate the constant joints of a wheel-mode pose.

    Beta is excluded on purpose — see the module docstring.

    Args:
        theta_deg (float): Wheel-mode fold angle in degrees (all legs).
        gamma_deg (float): ABAD angle in degrees (all legs).

    Raises:
        ValueError: If theta or gamma is outside the configured safety limits.
    """
    errors = []
    if not (RobotParams.MIN_THETA_DEG <= theta_deg <= RobotParams.MAX_THETA_DEG):
        errors.append(
            "theta={:.3f} deg outside [{:.1f}, {:.1f}] deg".format(
                theta_deg, RobotParams.MIN_THETA_DEG, RobotParams.MAX_THETA_DEG
            )
        )
    if abs(gamma_deg) > RobotParams.GAMMA_MAX_DEG:
        errors.append(
            "gamma={:.3f} deg outside +/-{:.1f} deg".format(gamma_deg, RobotParams.GAMMA_MAX_DEG)
        )
    if errors:
        raise ValueError("Wheel pose outside safety limits: " + "; ".join(errors))


def velocity_profile(n_samples, dt, v_cruise, ramp_time):
    """Build a cosine-smoothed trapezoidal speed profile.

    A step change in speed would command a beta velocity discontinuity, so the
    profile eases in and out. The ramps are half-cosine, giving continuous
    velocity (and therefore a C1 beta trajectory).

    Args:
        n_samples (int): Number of samples in the rolling segment.
        dt (float): Sample time (s).
        v_cruise (float): Cruise speed (m/s).
        ramp_time (float): Ramp-in and ramp-out duration each (s).

    Returns:
        np.ndarray: Speed at each sample, shape (n_samples,).
    """
    t = np.arange(n_samples) * dt
    total = n_samples * dt
    ramp = min(ramp_time, total / 2.0)
    v = np.full(n_samples, v_cruise, dtype=float)

    if ramp > 0.0:
        rise = t < ramp
        v[rise] = v_cruise * 0.5 * (1.0 - np.cos(np.pi * t[rise] / ramp))
        fall = t > (total - ramp)
        v[fall] = v_cruise * 0.5 * (1.0 - np.cos(np.pi * (total - t[fall]) / ramp))
    return v


def _solve_rolling_samples(distance, duration, v_cruise, ramp_time, dt):
    """Resolve the rolling-segment sample count from distance or duration.

    With half-cosine ramps the mean speed over each ramp is exactly v/2, so a
    run of total time T covers ``v * (T - ramp)``. Inverting that gives the
    duration needed for a requested distance.

    Args:
        distance (float or None): Requested travel distance (m).
        duration (float or None): Requested rolling duration (s).
        v_cruise (float): Cruise speed (m/s).
        ramp_time (float): Ramp duration each side (s).
        dt (float): Sample time (s).

    Returns:
        tuple[int, float]: Sample count and the ramp time actually used (s).
    """
    if duration is None:
        min_distance = v_cruise * ramp_time
        if distance < min_distance:
            # Too short for full ramps: shrink them so the run still fits.
            ramp_time = distance / v_cruise
        duration = distance / v_cruise + ramp_time

    n_samples = max(1, int(round(duration / dt)))
    return n_samples, min(ramp_time, n_samples * dt / 2.0)


def generate_wheel_csv(
    velocity=0.1,
    distance=1.2,
    duration=None,
    yaw_rate=0.0,
    rolling_radius=None,
    theta_deg=None,
    gamma_deg=0.0,
    beta0_rad=0.0,
    ramp_time=1.0,
    hold_time=1.0,
    prep_time=5.0,
    dt=0.001,
    output_dir="outputs/csv",
    output_path=None,
):
    """Generate a wheel-mode rolling CSV.

    Args:
        velocity (float): Body forward speed at cruise (m/s). Negative rolls backward.
        distance (float): Travel distance (m). Ignored when ``duration`` is given.
        duration (float, optional): Rolling duration (s); overrides ``distance``.
        yaw_rate (float): Body yaw rate (rad/s) for differential steering.
        rolling_radius (float, optional): Effective rolling radius (m). Defaults to
            ``RobotParams.WHEEL_RADIUS_OUTER``.
        theta_deg (float, optional): Wheel-mode fold angle (deg). Defaults to the
            module-level ``WHEEL_THETA_DEG`` calibration constant.
        gamma_deg (float): ABAD angle held during the roll (deg, all legs).
        beta0_rad (float): Starting beta (rad). Use the previous segment's final
            beta when concatenating so the command stays continuous.
        ramp_time (float): Speed ramp-in / ramp-out duration each (s).
        hold_time (float): Time to hold the final pose after stopping (s).
        prep_time (float): Leading transform segment (s). The corgi_csv_control
            node consumes the first 5000 rows as its transform block, so 5.0 s at
            dt=0.001 is what the hardware expects.
        dt (float): Sample time (s).
        output_dir (str): Directory for the default filename.
        output_path (str, optional): Explicit output path; overrides ``output_dir``.

    Returns:
        str: Saved CSV filepath.
    """
    _emit_progress(5)
    print("=========================================")
    print(" CorgiRobot Wheel-Mode CSV Generator     ")
    print("=========================================")

    if dt <= 0:
        raise ValueError("dt must be positive")
    if velocity == 0.0 and yaw_rate == 0.0:
        raise ValueError("velocity and yaw_rate cannot both be zero")
    if duration is None and distance <= 0:
        raise ValueError("distance must be positive when duration is not given")

    R = RobotParams.WHEEL_RADIUS_OUTER if rolling_radius is None else rolling_radius
    theta_deg = WHEEL_THETA_DEG if theta_deg is None else theta_deg
    if R <= 0:
        raise ValueError("rolling_radius must be positive")

    validate_wheel_pose(theta_deg, gamma_deg)
    _emit_progress(15)

    theta = np.radians(theta_deg)
    gamma = np.radians(gamma_deg)
    half_track = RobotParams.BODY_WIDTH / 2.0

    # Differential steering: the body yaw rate spreads the two sides' speeds.
    v_left = velocity - yaw_rate * half_track
    v_right = velocity + yaw_rate * half_track

    # The profile is shaped on the body speed, then scaled per side, so both
    # sides ramp together and the yaw rate stays proportional throughout.
    v_ref = velocity if velocity != 0.0 else max(abs(v_left), abs(v_right))
    n_roll, ramp_time = _solve_rolling_samples(distance, duration, abs(v_ref), ramp_time, dt)
    shape = velocity_profile(n_roll, dt, 1.0, ramp_time)
    _emit_progress(30)

    # beta(t) = beta0 + integral(v(t) / R) dt, integrated per side.
    beta_left = beta0_rad + np.cumsum(shape * (v_left / R)) * dt
    beta_right = beta0_rad + np.cumsum(shape * (v_right / R)) * dt
    _emit_progress(50)

    roll_cmds = np.zeros((n_roll, 12))
    for leg in range(4):
        roll_cmds[:, leg * 2] = theta
        roll_cmds[:, leg * 2 + 1] = beta_left if leg in LEFT_LEGS else beta_right
        roll_cmds[:, 8 + leg] = gamma
    _emit_progress(65)

    # Prep: cosine ramp from the hardware home pose to the first rolling frame.
    # For a default wheel run both ends are the home pose, so this is a hold.
    n_prep = int(round(prep_time / dt))
    segments = []
    if n_prep > 0:
        home_pose = np.zeros(12)
        home_pose[[0, 2, 4, 6]] = np.radians(HOME_THETA_DEG)
        alpha = (0.5 * (1.0 - np.cos(np.pi * np.linspace(0.0, 1.0, n_prep))))[:, np.newaxis]
        segments.append((1.0 - alpha) * home_pose + alpha * roll_cmds[0])
    segments.append(roll_cmds)

    n_hold = int(round(hold_time / dt))
    if n_hold > 0:
        segments.append(np.tile(roll_cmds[-1], (n_hold, 1)))
    final_cmds = np.vstack(segments)
    _emit_progress(80)

    # Phase sidecar, row-aligned with the main CSV. Wheel mode never lifts a
    # leg, so every leg is in continuous rolling contact (stance = 0).
    final_phase = np.zeros((len(final_cmds), 4))

    travelled = float(np.sum(shape * abs(v_ref)) * dt)

    if output_path is None:
        name = "Wheel_V{:.2f}_Wz{:.2f}_R{:.3f}_D{:.2f}_dt{}.csv".format(
            velocity, yaw_rate, R, travelled, dt
        )
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, name)
    else:
        parent = os.path.dirname(output_path)
        if parent:
            os.makedirs(parent, exist_ok=True)

    np.savetxt(output_path, final_cmds, delimiter=",", fmt="%.6f")
    phase_path = output_path[:-4] + "_phase.csv"
    np.savetxt(
        phase_path, final_phase, delimiter=",", fmt="%.0f", header=_phase_header(), comments=""
    )
    _emit_progress(100)

    print("\n  Rolling radius : {:.4f} m  (RobotParams.WHEEL_RADIUS_OUTER)".format(R))
    theta_note = (
        ""
        if theta_deg == RobotParams.THETA0_DEG
        else "  <- calibrated, nominal is {:.1f}".format(RobotParams.THETA0_DEG)
    )
    print("  Wheel theta    : {:.2f} deg{}".format(theta_deg, theta_note))
    print("  Home theta     : {:.2f} deg  (prep segment starts here)".format(HOME_THETA_DEG))
    print("  Speed          : {:+.3f} m/s cruise, yaw {:+.3f} rad/s".format(velocity, yaw_rate))
    if yaw_rate != 0.0:
        print("  Side speeds    : left {:+.3f} m/s, right {:+.3f} m/s".format(v_left, v_right))
    print("\n[SUCCESS]")
    print("  Prep      : {} frames ({:.1f} s)".format(n_prep, n_prep * dt))
    print(
        "  Roll      : {} frames ({:.1f} s, {:.3f} m, ramp {:.2f} s)".format(
            n_roll, n_roll * dt, travelled, ramp_time
        )
    )
    print("  Hold      : {} frames ({:.1f} s)".format(n_hold, n_hold * dt))
    print("  Total     : {} frames ({:.1f} s)".format(len(final_cmds), len(final_cmds) * dt))
    print(
        "  Beta span : {:+.3f} -> {:+.3f} rad  ({:.2f} revolutions)".format(
            beta0_rad, float(beta_left[-1]), abs(float(beta_left[-1]) - beta0_rad) / (2 * np.pi)
        )
    )
    print("  Saved to  : {}".format(output_path))
    print("  Phase to  : {}".format(phase_path))
    return output_path


def parse_args():
    """Parse command-line arguments.

    Returns:
        argparse.Namespace: Parsed arguments.
    """
    parser = argparse.ArgumentParser(
        description="Generate CorgiRobot 12-DOF wheel-mode rolling CSV.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("-v", "--velocity", type=float, default=0.1, help="Cruise speed (m/s)")
    parser.add_argument("-d", "--distance", type=float, default=1.2, help="Travel distance (m)")
    parser.add_argument(
        "-t",
        "--duration",
        type=float,
        default=None,
        help="Rolling duration (s); overrides distance",
    )
    parser.add_argument("-wz", "--yaw-rate", type=float, default=0.0, help="Yaw rate (rad/s)")
    parser.add_argument(
        "-R",
        "--radius",
        type=float,
        default=None,
        help="Rolling radius (m); default RobotParams.WHEEL_RADIUS_OUTER",
    )
    parser.add_argument(
        "--theta",
        type=float,
        default=None,
        help="Wheel fold angle (deg); default WHEEL_THETA_DEG set at the top of this file",
    )
    parser.add_argument(
        "--gamma", type=float, default=0.0, help="ABAD angle held while rolling (deg)"
    )
    parser.add_argument("--beta0", type=float, default=0.0, help="Starting beta (rad)")
    parser.add_argument(
        "--ramp", type=float, default=1.0, help="Speed ramp in/out duration each (s)"
    )
    parser.add_argument("--hold", type=float, default=1.0, help="Final-pose hold time (s)")
    parser.add_argument(
        "--prep",
        type=float,
        default=5.0,
        help="Leading transform segment (s); 5.0 matches corgi_csv_control",
    )
    parser.add_argument("-dt", "--dt", type=float, default=0.001, help="Sample time (s)")
    parser.add_argument("-o", "--outdir", type=str, default="outputs/csv", help="Output directory")
    parser.add_argument("--output", type=str, default=None, help="Explicit output CSV path")
    return parser.parse_args()


def main():
    """Run the wheel-mode CSV generator."""
    args = parse_args()
    generate_wheel_csv(
        velocity=args.velocity,
        distance=args.distance,
        duration=args.duration,
        yaw_rate=args.yaw_rate,
        rolling_radius=args.radius,
        theta_deg=args.theta,
        gamma_deg=args.gamma,
        beta0_rad=args.beta0,
        ramp_time=args.ramp,
        hold_time=args.hold,
        prep_time=args.prep,
        dt=args.dt,
        output_dir=args.outdir,
        output_path=args.output,
    )


if __name__ == "__main__":
    main()
