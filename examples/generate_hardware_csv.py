"""
Hardware CSV Generator for CorgiRobot 12-DOF Gait Trajectories.

Generates a CSV file containing joint trajectories for real hardware experiments.
The output format is tailored for the existing hardware controller:
- 12 Columns, NO header.
- Order: 
  [FL_theta, FL_beta, FR_theta, FR_beta, RR_theta, RR_beta, RL_theta, RL_beta, FL_gamma, FR_gamma, RR_gamma, RL_gamma]
  (This separates the 8 sagittal motors from the 4 new ABAD motors)
"""

from legwheel.planners.gait_generator_3d import GaitGenerator3D
import os
import sys
import argparse
from datetime import datetime
import numpy as np

# Ensure legwheel is in path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


def generate_hardware_csv(
    twist=[0.0, 0.1, 0.0],
    gait_type="Trot",
    stand_height=0.25,
    step_height=0.04,
    period=5.98,
    dt=0.001,
    n_cycles=5,
    output_dir="outputs/csv"
):
    print("=========================================")
    print(" CorgiRobot Hardware CSV Generator       ")
    print("=========================================")

    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # 1. Generate Gait
    gait = GaitGenerator3D(
        stand_height=stand_height,
        twist=twist,
        step_height=step_height,
        period=period,
        gait_type=gait_type,
        dt=dt
    )
    gait.print_summary()

    print(f"\nGenerating {n_cycles} cycles of {gait_type} gait...")
    # commands shape: (N_frames, 12)
    # Default order from GaitGenerator3D:
    # [FL_t, FL_b, FL_g, FR_t, FR_b, FR_g, RR_t, RR_b, RR_g, RL_t, RL_b, RL_g]
    raw_cmds = gait.generate_full_gait(n_cycles=n_cycles)

    # 2. Reorder columns to match hardware specification
    # Target HW Order:
    #   0: FL_theta, 1: FL_beta
    #   2: FR_theta, 3: FR_beta
    #   4: RR_theta, 5: RR_beta
    #   6: RL_theta, 7: RL_beta
    #   8: FL_gamma, 9: FR_gamma, 10: RR_gamma, 11: RL_gamma

    N_frames = raw_cmds.shape[0]
    hw_cmds = np.zeros((N_frames, 12))

    # Safely map columns
    # FL
    hw_cmds[:, 0] = raw_cmds[:, 0]  # theta
    hw_cmds[:, 1] = raw_cmds[:, 1]  # beta
    hw_cmds[:, 8] = raw_cmds[:, 2]  # gamma
    # FR
    hw_cmds[:, 2] = raw_cmds[:, 3]
    hw_cmds[:, 3] = raw_cmds[:, 4]
    hw_cmds[:, 9] = raw_cmds[:, 5]
    # RR
    hw_cmds[:, 4] = raw_cmds[:, 6]
    hw_cmds[:, 5] = raw_cmds[:, 7]
    hw_cmds[:, 10] = raw_cmds[:, 8]
    # RL
    hw_cmds[:, 6] = raw_cmds[:, 9]
    hw_cmds[:, 7] = raw_cmds[:, 10]
    hw_cmds[:, 11] = raw_cmds[:, 11]

    # 3. Generate Unique Filename by Parameters
    # Uses the internal post-guard scaled values from the gait generator
    filename = f"{gait.get_parameter_string()}.csv"
    filepath = os.path.join(output_dir, filename)

    # 4. Add 5-second startup preparation sequence
    prep_time = 5.0
    N_prep = int(prep_time / dt)

    # Home position: theta=17 deg, beta=0, gamma=0
    # HW Order: [FL_t, FL_b, FR_t, FR_b, RR_t, RR_b, RL_t, RL_b, FL_g, FR_g, RR_g, RL_g]
    theta_home = np.radians(17)
    home_pose = np.zeros(12)
    # Set all 4 theta joints to 17 degrees
    home_pose[[0, 2, 4, 6]] = theta_home

    # Smooth cosine interpolation from home_pose to hw_cmds[0]
    t_interp = np.linspace(0, 1, N_prep)
    alpha = 0.5 * (1 - np.cos(np.pi * t_interp))
    alpha = alpha[:, np.newaxis]  # shape (N_prep, 1)

    prep_cmds = (1 - alpha) * home_pose + alpha * hw_cmds[0]

    # Concatenate prep sequence and gait sequence
    final_cmds = np.vstack((prep_cmds, hw_cmds))
    total_frames = final_cmds.shape[0]

    # 5. Save to CSV without headers
    np.savetxt(filepath, final_cmds, delimiter=",", fmt="%.6f")

    print("\n[SUCCESS]")
    print(
        f"Generated {total_frames} frames (including {N_prep} frames of 5s prep sequence).")
    print(f"Saved to: {filepath}")
    return filepath


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate CorgiRobot 12-DOF Gait CSV for hardware experiments.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    # Optional arguments with shorthand flags
    parser.add_argument("-g", "--gait", type=str, default="Walk",
                        help="Gait type (Trot, Walk, Pace, Bound, Pronk)")
    parser.add_argument("-vx", "--vx", type=float,
                        default=0.0, help="Forward velocity (m/s)")
    parser.add_argument("-vy", "--vy", type=float,
                        default=0.1, help="Lateral velocity (m/s)")
    parser.add_argument("-wz", "--wz", type=float,
                        default=0.0, help="Yaw velocity (rad/s)")
    parser.add_argument("-z", "--height", type=float,
                        default=0.25, help="Standing height (m)")
    parser.add_argument("-s", "--step", type=float,
                        default=0.04, help="Step height (m)")
    parser.add_argument("-p", "--period", type=float,
                        default=4, help="Gait period (s)")
    parser.add_argument("-c", "--cycles", type=int, default=10,
                        help="Number of gait cycles to generate")
    parser.add_argument("-dt", "--dt", type=float,
                        default=0.001, help="Time step (s)")
    parser.add_argument("-o", "--outdir", type=str,
                        default="outputs/csv", help="Output directory")

    args = parser.parse_args()

    generate_hardware_csv(
        twist=[args.wz, args.vx, args.vy],
        gait_type=args.gait,
        stand_height=args.height,
        step_height=args.step,
        period=args.period,
        dt=args.dt,
        n_cycles=args.cycles,
        output_dir=args.outdir
    )
