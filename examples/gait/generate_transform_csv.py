"""
Generate a simple 12-DOF actuator transform CSV for CorgiRobot hardware.

This script does not use gait planning. It creates a smooth point-to-point
transition from a start actuator pose to a target actuator pose, using the same
hardware CSV column order as ``generate_hardware_csv.py``:

    [FL_theta, FL_beta, FR_theta, FR_beta, RR_theta, RR_beta, RL_theta, RL_beta,
     FL_gamma, FR_gamma, RR_gamma, RL_gamma]

The default start pose is the hardware home pose: theta=17 deg, beta=0 deg,
gamma=0 deg for all four legs.
"""

import argparse
import os
from datetime import datetime

import numpy as np

LEG_NAMES = ("FL", "FR", "RR", "RL")
PROGRESS_PREFIX = "::progress::"


def _emit_progress(percent):
    """Emit a machine-readable progress percentage for TUI consumers."""
    percent = max(0, min(100, int(round(percent))))
    print("{}{}".format(PROGRESS_PREFIX, percent), flush=True)


def _get_safety_limits_deg():
    """Get actuator safety limits in degrees.

    Returns:
        dict: Limit ranges for theta, beta, and gamma in degrees.
    """
    from legwheel.config import RobotParams

    return {
        "theta": (RobotParams.MIN_THETA_DEG, RobotParams.MAX_THETA_DEG),
        "beta": (-RobotParams.BETA_MAX_DEG, RobotParams.BETA_MAX_DEG),
        "gamma": (-RobotParams.GAMMA_MAX_DEG, RobotParams.GAMMA_MAX_DEG),
    }


def _as_degrees(values, unit):
    """Represent input angles in degrees for validation.

    Args:
        values (list[float]): Angle values.
        unit (str): Input angle unit, either ``deg`` or ``rad``.

    Returns:
        np.ndarray: Angle values in degrees.
    """
    arr = np.asarray(values, dtype=float)
    if unit == "rad":
        return np.rad2deg(arr)
    return arr


def validate_pose_limits(theta, beta, gamma, unit="deg"):
    """Validate actuator target values against conservative hardware limits.

    Args:
        theta (list[float]): Extension actuator values for FL, FR, RR, RL.
        beta (list[float]): Swing actuator values for FL, FR, RR, RL.
        gamma (list[float]): Hip-roll / ABAD actuator values for FL, FR, RR, RL.
        unit (str): Input angle unit, either ``deg`` or ``rad``.

    Raises:
        ValueError: If any actuator target is outside the configured safety limits.
    """
    limits = _get_safety_limits_deg()
    commands = {
        "theta": _as_degrees(theta, unit),
        "beta": _as_degrees(beta, unit),
        "gamma": _as_degrees(gamma, unit),
    }

    errors = []
    for joint_name, values in commands.items():
        lower, upper = limits[joint_name]
        if values.shape != (4,):
            errors.append("{} must contain exactly four values".format(joint_name))
            continue
        for leg_name, value in zip(LEG_NAMES, values):
            if value < lower or value > upper:
                errors.append(
                    "{}_{}={:.3f} deg outside [{:.3f}, {:.3f}] deg".format(
                        leg_name, joint_name, value, lower, upper
                    )
                )

    if errors:
        raise ValueError("Actuator safety check failed: " + "; ".join(errors))


def _convert_angles(values, unit):
    """Convert angle values to radians.

    Args:
        values (list[float]): Angle values for the four legs in FL, FR, RR, RL order.
        unit (str): Input unit, either ``deg`` or ``rad``.

    Returns:
        np.ndarray: Angle values in radians.
    """
    arr = np.asarray(values, dtype=float)
    if unit == "deg":
        return np.deg2rad(arr)
    return arr


def build_hw_pose(theta, beta, gamma, unit="deg"):
    """Build one hardware command row from per-leg actuator values.

    Args:
        theta (list[float]): Extension actuator values for FL, FR, RR, RL.
        beta (list[float]): Swing actuator values for FL, FR, RR, RL.
        gamma (list[float]): Hip-roll / ABAD actuator values for FL, FR, RR, RL.
        unit (str): Input angle unit, either ``deg`` or ``rad``.

    Returns:
        np.ndarray: Shape ``(12,)`` command row in hardware CSV order.
    """
    theta = _convert_angles(theta, unit)
    beta = _convert_angles(beta, unit)
    gamma = _convert_angles(gamma, unit)

    if theta.shape != (4,) or beta.shape != (4,) or gamma.shape != (4,):
        raise ValueError("theta, beta, and gamma must each contain exactly four values")

    pose = np.zeros(12)
    pose[[0, 2, 4, 6]] = theta
    pose[[1, 3, 5, 7]] = beta
    pose[[8, 9, 10, 11]] = gamma
    return pose


def generate_transform_csv(
    target_theta,
    target_beta,
    target_gamma,
    start_theta=None,
    start_beta=None,
    start_gamma=None,
    unit="deg",
    duration=5.0,
    hold_time=0.0,
    dt=0.001,
    output_path=None,
):
    """Generate a smooth actuator transform CSV.

    Args:
        target_theta (list[float]): Target theta values for FL, FR, RR, RL.
        target_beta (list[float]): Target beta values for FL, FR, RR, RL.
        target_gamma (list[float]): Target gamma values for FL, FR, RR, RL.
        start_theta (list[float], optional): Start theta values. Defaults to 17 deg all legs.
        start_beta (list[float], optional): Start beta values. Defaults to 0 deg all legs.
        start_gamma (list[float], optional): Start gamma values. Defaults to 0 deg all legs.
        unit (str): Input angle unit, either ``deg`` or ``rad``.
        duration (float): Transform duration in seconds.
        hold_time (float): Extra time to hold the final target pose in seconds.
        dt (float): CSV sample time in seconds.
        output_path (str, optional): Output CSV path. Defaults to ``outputs/csv``.

    Returns:
        str: Saved CSV filepath.
    """
    _emit_progress(5)

    if dt <= 0:
        raise ValueError("dt must be positive")
    if duration < 0:
        raise ValueError("duration must be non-negative")
    if hold_time < 0:
        raise ValueError("hold_time must be non-negative")

    start_theta = start_theta if start_theta is not None else [17.0, 17.0, 17.0, 17.0]
    start_beta = start_beta if start_beta is not None else [0.0, 0.0, 0.0, 0.0]
    start_gamma = start_gamma if start_gamma is not None else [0.0, 0.0, 0.0, 0.0]

    validate_pose_limits(start_theta, start_beta, start_gamma, unit=unit)
    validate_pose_limits(target_theta, target_beta, target_gamma, unit=unit)
    _emit_progress(20)

    start_pose = build_hw_pose(start_theta, start_beta, start_gamma, unit=unit)
    target_pose = build_hw_pose(target_theta, target_beta, target_gamma, unit=unit)

    n_transform = max(1, int(duration / dt))
    if n_transform == 1:
        transform_cmds = target_pose.reshape(1, -1)
    else:
        t_interp = np.linspace(0.0, 1.0, n_transform)
        alpha = 0.5 * (1.0 - np.cos(np.pi * t_interp))
        alpha = alpha[:, np.newaxis]
        transform_cmds = (1.0 - alpha) * start_pose + alpha * target_pose
    _emit_progress(70)

    n_hold = int(hold_time / dt)
    if n_hold > 0:
        hold_cmds = np.tile(target_pose, (n_hold, 1))
        final_cmds = np.vstack((transform_cmds, hold_cmds))
    else:
        final_cmds = transform_cmds
    _emit_progress(85)

    if output_path is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = os.path.join("outputs", "csv", "transform_pose_{}.csv".format(timestamp))

    output_dir = os.path.dirname(output_path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    np.savetxt(output_path, final_cmds, delimiter=",", fmt="%.6f")
    _emit_progress(100)
    return output_path


def parse_args():
    """Parse command-line arguments.

    Returns:
        argparse.Namespace: Parsed arguments.
    """
    parser = argparse.ArgumentParser(
        description="Generate a simple 12-DOF actuator transform CSV for CorgiRobot.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--theta",
        nargs=4,
        type=float,
        required=True,
        metavar=LEG_NAMES,
        help="Target theta values for FL FR RR RL.",
    )
    parser.add_argument(
        "--beta",
        nargs=4,
        type=float,
        required=True,
        metavar=LEG_NAMES,
        help="Target beta values for FL FR RR RL.",
    )
    parser.add_argument(
        "--gamma",
        nargs=4,
        type=float,
        required=True,
        metavar=LEG_NAMES,
        help="Target gamma values for FL FR RR RL.",
    )
    parser.add_argument(
        "--start-theta",
        nargs=4,
        type=float,
        default=[17.0, 17.0, 17.0, 17.0],
        metavar=LEG_NAMES,
        help="Start theta values for FL FR RR RL.",
    )
    parser.add_argument(
        "--start-beta",
        nargs=4,
        type=float,
        default=[0.0, 0.0, 0.0, 0.0],
        metavar=LEG_NAMES,
        help="Start beta values for FL FR RR RL.",
    )
    parser.add_argument(
        "--start-gamma",
        nargs=4,
        type=float,
        default=[0.0, 0.0, 0.0, 0.0],
        metavar=LEG_NAMES,
        help="Start gamma values for FL FR RR RL.",
    )
    parser.add_argument("--unit", choices=("deg", "rad"), default="deg", help="Input angle unit.")
    parser.add_argument(
        "--duration", type=float, default=5.0, help="Transform duration in seconds."
    )
    parser.add_argument("--hold", type=float, default=0.0, help="Final-pose hold time in seconds.")
    parser.add_argument("--dt", type=float, default=0.001, help="CSV sample time in seconds.")
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Output CSV path. Defaults to outputs/csv/transform_pose_<timestamp>.csv.",
    )
    return parser.parse_args()


def main():
    """Run the actuator transform CSV generator."""
    args = parse_args()
    filepath = generate_transform_csv(
        target_theta=args.theta,
        target_beta=args.beta,
        target_gamma=args.gamma,
        start_theta=args.start_theta,
        start_beta=args.start_beta,
        start_gamma=args.start_gamma,
        unit=args.unit,
        duration=args.duration,
        hold_time=args.hold,
        dt=args.dt,
        output_path=args.output,
    )
    print("[SUCCESS] Generated actuator transform CSV")
    print("Leg order for inputs: FL FR RR RL")
    print("Hardware column order:")
    print("  FL_theta, FL_beta, FR_theta, FR_beta, RR_theta, RR_beta, RL_theta, RL_beta,")
    print("  FL_gamma, FR_gamma, RR_gamma, RL_gamma")
    print("Saved to: {}".format(filepath))


if __name__ == "__main__":
    main()
