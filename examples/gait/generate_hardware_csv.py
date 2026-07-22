"""
Hardware CSV Generator for CorgiRobot 12-DOF Gait Trajectories.

Generates a CSV file containing joint trajectories for real hardware experiments.
The output format is tailored for the existing hardware controller:
- 12 Columns, NO header.
- Order:
  [FL_theta, FL_beta, FR_theta, FR_beta, RR_theta, RR_beta, RL_theta, RL_beta,
   FL_gamma, FR_gamma, RR_gamma, RL_gamma]
  (Separates 8 sagittal motors from 4 ABAD motors)

With --launch, the full sequence is:
  [prep: cosine ramp home → neutral]
  [launch: N_ramp cycles, v from ramp_floor% → 100%, phase-shifted to all-stance]
  [steady: N_cycles at full speed]
"""

from legwheel.planners.gait_generator_3d import GaitGenerator3D
import os
import sys
import argparse
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


def _to_hw_order(raw_cmds: np.ndarray) -> np.ndarray:
    """Reorder (N, 12) planner output to hardware column order."""
    N = raw_cmds.shape[0]
    hw = np.zeros((N, 12))
    hw[:, 0] = raw_cmds[:, 0]   # FL theta
    hw[:, 1] = raw_cmds[:, 1]   # FL beta
    hw[:, 8] = raw_cmds[:, 2]   # FL gamma
    hw[:, 2] = raw_cmds[:, 3]   # FR theta
    hw[:, 3] = raw_cmds[:, 4]   # FR beta
    hw[:, 9] = raw_cmds[:, 5]   # FR gamma
    hw[:, 4] = raw_cmds[:, 6]   # RR theta
    hw[:, 5] = raw_cmds[:, 7]   # RR beta
    hw[:, 10] = raw_cmds[:, 8]  # RR gamma
    hw[:, 6] = raw_cmds[:, 9]   # RL theta
    hw[:, 7] = raw_cmds[:, 10]  # RL beta
    hw[:, 11] = raw_cmds[:, 11] # RL gamma
    return hw


def generate_hardware_csv(
    twist=[0.0, 0.1, 0.0],
    gait_type="Trot",
    stand_height=0.25,
    step_height=0.04,
    period=5.98,
    dt=0.001,
    n_cycles=5,
    output_dir="outputs/csv",
    with_launch=False,
    n_ramp=3,
    ramp_floor=0.1,
    lead_fraction=0.5,
):
    print("=========================================")
    print(" CorgiRobot Hardware CSV Generator       ")
    print("=========================================")

    os.makedirs(output_dir, exist_ok=True)

    # 1. Steady-state gait
    gait = GaitGenerator3D(
        stand_height=stand_height,
        twist=twist,
        step_height=step_height,
        period=period,
        gait_type=gait_type,
        dt=dt,
        lead_fraction=lead_fraction,
    )
    gait.print_summary()

    print(f"\nGenerating {n_cycles} steady cycles of {gait_type}...")
    hw_cmds = _to_hw_order(gait.generate_full_gait(n_cycles=n_cycles))

    # 2. Optional launch ramp sequence
    launch_hw_cmds = None
    if with_launch:
        from legwheel.planners.launch_controller import LaunchController
        lc = LaunchController(
            gait_type=gait_type,
            stand_height=stand_height,
            twist=twist,
            step_height=step_height,
            period=period,
            dt=dt,
            n_ramp=n_ramp,
            ramp_floor=ramp_floor,
            lead_fraction=lead_fraction,
        )
        lc.print_summary()
        launch_hw_cmds = _to_hw_order(lc.generate_launch_sequence())

    # 3. Prep sequence: cosine ramp from home (θ=17°) to first gait frame
    prep_time = 5.0
    N_prep = int(prep_time / dt)
    theta_home = np.radians(17.0)
    home_pose = np.zeros(12)
    home_pose[[0, 2, 4, 6]] = theta_home  # all 4 theta joints

    first_frame = launch_hw_cmds[0] if launch_hw_cmds is not None else hw_cmds[0]
    t_interp = np.linspace(0, 1, N_prep)
    alpha = (0.5 * (1 - np.cos(np.pi * t_interp)))[:, np.newaxis]
    prep_cmds = (1 - alpha) * home_pose + alpha * first_frame

    # 4. Stack all segments
    segments = [prep_cmds]
    if launch_hw_cmds is not None:
        segments.append(launch_hw_cmds)
    segments.append(hw_cmds)
    final_cmds = np.vstack(segments)

    # 5. Filename
    suffix = f"_L{n_ramp}F{int(ramp_floor * 100)}" if with_launch else ""
    filename = f"{gait.get_parameter_string()}{suffix}.csv"
    filepath = os.path.join(output_dir, filename)

    np.savetxt(filepath, final_cmds, delimiter=",", fmt="%.6f")

    N_launch = len(launch_hw_cmds) if launch_hw_cmds is not None else 0
    print("\n[SUCCESS]")
    print(f"  Prep      : {N_prep} frames ({prep_time:.1f} s)")
    if with_launch:
        print(f"  Launch    : {N_launch} frames ({N_launch*dt:.1f} s, {n_ramp} ramp cycles)")
    print(f"  Steady    : {len(hw_cmds)} frames ({len(hw_cmds)*dt:.1f} s, {n_cycles} cycles)")
    print(f"  Total     : {len(final_cmds)} frames ({len(final_cmds)*dt:.1f} s)")
    print(f"  Saved to  : {filepath}")
    return filepath


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate CorgiRobot 12-DOF Gait CSV for hardware experiments.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("-g", "--gait",   type=str,   default="Walk",  help="Gait type")
    parser.add_argument("-vx", "--vx",    type=float, default=0.0,     help="Forward velocity (m/s)")
    parser.add_argument("-vy", "--vy",    type=float, default=0.1,     help="Lateral velocity (m/s)")
    parser.add_argument("-wz", "--wz",    type=float, default=0.0,     help="Yaw velocity (rad/s)")
    parser.add_argument("-z", "--height", type=float, default=0.25,    help="Standing height (m)")
    parser.add_argument("-s", "--step",   type=float, default=0.04,    help="Step height (m)")
    parser.add_argument("-p", "--period", type=float, default=4,       help="Gait period (s)")
    parser.add_argument("-c", "--cycles", type=int,   default=10,      help="Number of gait cycles")
    parser.add_argument("-dt", "--dt",    type=float, default=0.001,   help="Time step (s)")
    parser.add_argument("-o", "--outdir", type=str,   default="outputs/csv", help="Output directory")
    parser.add_argument("--launch",       action="store_true",         help="Prepend launch ramp sequence")
    parser.add_argument("--ramp-cycles",  type=int,   default=3,       help="Number of ramp cycles")
    parser.add_argument("--ramp-floor",   type=float, default=0.1,     help="Starting velocity fraction (0–1)")
    parser.add_argument("--lead-fraction", type=float, default=0.5,
                        help="Lateral stance lead fraction κ (0–1)")

    args = parser.parse_args()
    generate_hardware_csv(
        twist=[args.wz, args.vx, args.vy],
        gait_type=args.gait,
        stand_height=args.height,
        step_height=args.step,
        period=args.period,
        dt=args.dt,
        n_cycles=args.cycles,
        output_dir=args.outdir,
        with_launch=args.launch,
        n_ramp=args.ramp_cycles,
        ramp_floor=args.ramp_floor,
        lead_fraction=args.lead_fraction,
    )
