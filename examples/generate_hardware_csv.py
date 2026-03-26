"""
Hardware CSV Generator for CorgiRobot 12-DOF Gait Trajectories.

Generates a CSV file containing joint trajectories for real hardware experiments.
The output format is tailored for the existing hardware controller:
- 12 Columns, NO header.
- Order: 
  [FL_theta, FL_beta, FR_theta, FR_beta, RR_theta, RR_beta, RL_theta, RL_beta, FL_gamma, FR_gamma, RR_gamma, RL_gamma]
  (This separates the 8 sagittal motors from the 4 new ABAD motors)
"""

import os
import sys
import argparse
from datetime import datetime
import numpy as np

# Ensure legwheel is in path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from legwheel.planners.gait_generator_3d import GaitGenerator3D

def generate_hardware_csv(
    twist=[0.0, 0.15, 0.0],
    gait_type="Trot",
    stand_height=0.31,
    step_height=0.04,
    period=1.0,
    dt=0.005,
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

    # 3. Generate Unique Filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    # Format: GaitName_Vx0.15_Vy0.00_Wz0.00_H0.31_P1.0_Timestamp.csv
    filename = f"{gait_type}_Vx{twist[1]:.2f}_Vy{twist[2]:.2f}_Wz{twist[0]:.2f}_H{stand_height:.2f}_P{period:.1f}_{timestamp}.csv"
    filepath = os.path.join(output_dir, filename)

    # 4. Save to CSV without headers
    np.savetxt(filepath, hw_cmds, delimiter=",", fmt="%.6f")

    print("\n[SUCCESS]")
    print(f"Generated {N_frames} frames.")
    print(f"Saved to: {filepath}")
    return filepath

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate CorgiRobot 12-DOF Gait CSV for hardware experiments.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    # Optional arguments with shorthand flags
    parser.add_argument("-g", "--gait", type=str, default="Trot", help="Gait type (Trot, Walk, Pace, Bound, Pronk)")
    parser.add_argument("-vx", "--vx", type=float, default=0.15, help="Forward velocity (m/s)")
    parser.add_argument("-vy", "--vy", type=float, default=0.0, help="Lateral velocity (m/s)")
    parser.add_argument("-wz", "--wz", type=float, default=0.0, help="Yaw velocity (rad/s)")
    parser.add_argument("-z", "--height", type=float, default=0.31, help="Standing height (m)")
    parser.add_argument("-s", "--step", type=float, default=0.04, help="Step height (m)")
    parser.add_argument("-p", "--period", type=float, default=1.0, help="Gait period (s)")
    parser.add_argument("-c", "--cycles", type=int, default=10, help="Number of gait cycles to generate")
    parser.add_argument("-dt", "--dt", type=float, default=0.005, help="Time step (s)")
    parser.add_argument("-o", "--outdir", type=str, default="outputs/csv", help="Output directory")

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
