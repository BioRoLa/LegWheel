"""
Lean Pose CSV Generator for CorgiRobot hardware experiments.

Generates a hardware-format CSV (12 columns, no header) containing a
whole-body lean / pose trajectory produced by PosePlanner.

Hardware column order (matches generate_hardware_csv.py convention):
  [FL_theta, FL_beta, FR_theta, FR_beta, RR_theta, RR_beta, RL_theta, RL_beta,
   FL_gamma, FR_gamma, RR_gamma, RL_gamma]
"""

import os
import sys
import argparse
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

LEG_LABELS = ["FL", "FR", "RR", "RL"]


def _to_hw_order(cmds: np.ndarray) -> np.ndarray:
    """Reorder (N, 12) PosePlanner output → hardware column order."""
    # Planner order: FL_t FL_b FL_g  FR_t FR_b FR_g  RR_t RR_b RR_g  RL_t RL_b RL_g
    #                  0    1    2     3    4    5     6    7    8     9   10   11
    # HW order:      FL_t FL_b  FR_t FR_b  RR_t RR_b  RL_t RL_b  FL_g FR_g RR_g RL_g
    N = cmds.shape[0]
    hw = np.zeros((N, 12))
    hw[:, 0] = cmds[:, 0]   # FL theta
    hw[:, 1] = cmds[:, 1]   # FL beta
    hw[:, 2] = cmds[:, 3]   # FR theta
    hw[:, 3] = cmds[:, 4]   # FR beta
    hw[:, 4] = cmds[:, 6]   # RR theta
    hw[:, 5] = cmds[:, 7]   # RR beta
    hw[:, 6] = cmds[:, 9]   # RL theta
    hw[:, 7] = cmds[:, 10]  # RL beta
    hw[:, 8] = cmds[:, 2]   # FL gamma
    hw[:, 9] = cmds[:, 5]   # FR gamma
    hw[:, 10] = cmds[:, 8]  # RR gamma
    hw[:, 11] = cmds[:, 11] # RL gamma
    return hw


def generate_lean_csv(
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
    stand_height: float = 0.30,
    height_compensation: float = 0.0,
    n_steps: int = 500,
    return_to_neutral: bool = True,
    dt: float = 0.001,
    output_dir: str = "outputs/csv",
    prep_time: float = 3.0,
) -> str:
    """
    Generate a hardware lean trajectory CSV.

    Args:
        roll_deg (float): Target roll angle (degrees). Positive = left-side up.
        pitch_deg (float): Target pitch angle (degrees). Positive = nose down.
        yaw_deg (float): Target yaw angle (degrees).
        stand_height (float): Nominal body height above ground (m).
        height_compensation (float): Auto-lower body per rad of lean. 0.15–0.2 for large angles.
        n_steps (int): IK samples for each ramp segment (ramp-up, ramp-down).
        return_to_neutral (bool): Append a return-to-neutral ramp if True.
        dt (float): Time step (s).
        output_dir (str): Output directory for the CSV file.
        prep_time (float): Cosine ramp from home θ=17° to neutral pose (seconds).

    Returns:
        str: Path of the generated CSV file.
    """
    from legwheel.planners.pose_planner import PosePlanner

    print("=========================================")
    print(" CorgiRobot Lean Pose CSV Generator      ")
    print("=========================================")
    print(f"  Stand height      : {stand_height:.3f} m")
    print(f"  Roll              : {roll_deg:+.1f}°")
    print(f"  Pitch             : {pitch_deg:+.1f}°")
    print(f"  Yaw               : {yaw_deg:+.1f}°")
    print(f"  Height comp.      : {height_compensation:.2f} m/rad")
    print(f"  Steps per segment : {n_steps}")
    print(f"  Return to neutral : {return_to_neutral}")
    print(f"  dt                : {dt*1000:.1f} ms")
    print()

    os.makedirs(output_dir, exist_ok=True)

    pp = PosePlanner(stand_height=stand_height, dt=dt)

    print("Planning lean trajectory...")
    cmds = pp.plan_lean(
        roll=np.deg2rad(roll_deg),
        pitch=np.deg2rad(pitch_deg),
        yaw=np.deg2rad(yaw_deg),
        n_steps=n_steps,
        return_to_neutral=return_to_neutral,
        height_compensation=height_compensation,
    )

    hw_cmds = _to_hw_order(cmds)

    # 5s prep: cosine ramp from home (θ=17°, rest=0) to first gait frame
    N_prep = int(prep_time / dt)
    theta_home = np.radians(17.0)
    home_pose = np.zeros(12)
    home_pose[[0, 2, 4, 6]] = theta_home  # set all 4 theta joints

    t_interp = np.linspace(0, 1, N_prep)
    alpha = 0.5 * (1 - np.cos(np.pi * t_interp))[:, np.newaxis]
    prep_cmds = (1 - alpha) * home_pose + alpha * hw_cmds[0]

    final_cmds = np.vstack([prep_cmds, hw_cmds])

    # Filename encodes key parameters
    sign_r = "p" if roll_deg >= 0 else "n"
    sign_p = "p" if pitch_deg >= 0 else "n"
    sign_y = "p" if yaw_deg >= 0 else "n"
    filename = (
        f"Lean_R{sign_r}{abs(roll_deg):.1f}"
        f"_P{sign_p}{abs(pitch_deg):.1f}"
        f"_Y{sign_y}{abs(yaw_deg):.1f}"
        f"_H{stand_height:.2f}"
        f"_C{height_compensation:.2f}"
        f"_N{n_steps}"
        f"_prep{prep_time:.1f}"
        f"_dt{dt:g}"
        f"{'_ret' if return_to_neutral else ''}.csv"
    )
    filepath = os.path.join(output_dir, filename)

    np.savetxt(filepath, final_cmds, delimiter=",", fmt="%.6f")

    total = final_cmds.shape[0]
    print(f"\n[SUCCESS]")
    print(f"  Total frames : {total} ({total*dt:.1f} s including {prep_time:.1f}s prep)")
    print(f"  Saved to     : {filepath}")
    return filepath


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate CorgiRobot whole-body lean pose CSV.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--roll",  type=float, default=0.0,  help="Target roll  (deg, + = left up)")
    parser.add_argument("--pitch", type=float, default=0.0,  help="Target pitch (deg, + = nose down)")
    parser.add_argument("--yaw",   type=float, default=0.0,  help="Target yaw   (deg)")
    parser.add_argument("-z", "--height", type=float, default=0.30, help="Stand height (m)")
    parser.add_argument("--compensation", type=float, default=0.0,
                        help="Height compensation (m/rad). Use 0.15–0.2 for large angles.")
    parser.add_argument("-n", "--steps", type=int, default=500,
                        help="IK samples per ramp segment")
    parser.add_argument("--no-return", action="store_true",
                        help="Do NOT append a return-to-neutral ramp")
    parser.add_argument("-dt", "--dt", type=float, default=0.001, help="Time step (s)")
    parser.add_argument("--prep", type=float, default=3.0,
                        help="Prep sequence duration (s)")
    parser.add_argument("-o", "--outdir", type=str, default="outputs/csv",
                        help="Output directory")

    a = parser.parse_args()
    generate_lean_csv(
        roll_deg=a.roll,
        pitch_deg=a.pitch,
        yaw_deg=a.yaw,
        stand_height=a.height,
        height_compensation=a.compensation,
        n_steps=a.steps,
        return_to_neutral=not a.no_return,
        dt=a.dt,
        output_dir=a.outdir,
        prep_time=a.prep,
    )
