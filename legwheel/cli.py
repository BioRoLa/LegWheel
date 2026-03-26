#!/usr/bin/env python3
"""
Command Line Interface for LegWheel Library.
"""

import argparse
import sys
import numpy as np

def cmd_check(args):
    from legwheel.planners.gait_generator_3d import GaitGenerator3D, GAIT_LIBRARY
    from legwheel.config import RobotParams
    from legwheel.models.corgi_leg import CorgiLegKinematics
    from legwheel.utils.fitted_coefficient import inv_G_dist_poly

    if args.gait not in GAIT_LIBRARY:
        print(f"Error: Unknown gait '{args.gait}'. Available: {list(GAIT_LIBRARY.keys())}")
        return

    stance_duty = GAIT_LIBRARY[args.gait]["stance_duty"]
    print(f"=======================================")
    print(f" Corgi LegWheel Parameter Checker")
    print(f"=======================================")
    print(f" Gait:        {args.gait} (Duty: {stance_duty:.2f})")
    print(f" Period (T):  {args.period:.2f} s")
    print(f" Stand Height:{args.height:.3f} m")
    print(f" Step Height: {args.step:.3f} m")
    print(f" Twist Cmd:   [Vx={args.vx:.3f}, Vy={args.vy:.3f}, Wz={args.wz:.3f}]")
    print(f"---------------------------------------\n")

    errors = []
    warnings = []

    # 1. Height Check (Theta limits)
    H_O = args.height - RobotParams.WHEEL_RADIUS_PITCH
    R_link = RobotParams.WHEEL_RADIUS_PITCH * 0.2225
    
    try:
        G_dist = H_O / np.cos(0.0) + R_link
        theta0_rad = inv_G_dist_poly(G_dist)
        theta0_deg = np.rad2deg(theta0_rad)
        
        if theta0_deg < RobotParams.MIN_THETA_DEG:
            errors.append(f"Height Guard: Stand Height {args.height}m is too HIGH. Motor theta ({theta0_deg:.1f}°) drops below minimum limit ({RobotParams.MIN_THETA_DEG}°).")
        elif theta0_deg > RobotParams.MAX_THETA_DEG:
            errors.append(f"Height Guard: Stand Height {args.height}m is too LOW. Motor theta ({theta0_deg:.1f}°) exceeds maximum limit ({RobotParams.MAX_THETA_DEG}°).")
        else:
            print(f" [OK] Stand Height: \tTheta_0 = {theta0_deg:.1f}° (Safe)")
    except Exception as e:
        errors.append(f"Height Guard: Failed to solve initial theta for {args.height}m. Physically impossible configuration.")

    # 2. Twist Workspace Check
    L1 = RobotParams.WHEEL_RADIUS_PITCH * RobotParams.L1_RATIO
    R_arc = np.sqrt(L1**2 - R_link**2)
    BETA_MAX = np.deg2rad(40)
    GAMMA_MAX = np.deg2rad(8)

    D_x_max = 2 * H_O * np.tan(BETA_MAX) + 2 * R_arc * BETA_MAX
    v_x_limit = D_x_max / (args.period * stance_duty)
    
    D_y_max = 2 * args.height * np.sin(GAMMA_MAX)
    v_y_limit = D_y_max / (args.period * stance_duty)

    leg_kine = [CorgiLegKinematics(i) for i in range(4)]
    hip_positions = [leg.p_Mi_in_B for leg in leg_kine]
    
    scale_x = 1.0
    scale_y = 1.0
    for r_hip in hip_positions:
        hx = args.vx - args.wz * r_hip[1]
        hy = args.vy + args.wz * r_hip[0]
        if abs(hx) > v_x_limit:
            scale_x = min(scale_x, v_x_limit / abs(hx))
        if abs(hy) > v_y_limit:
            scale_y = min(scale_y, v_y_limit / abs(hy))
            
    global_scale = min(scale_x, scale_y)
    
    if global_scale < 1.0:
        eff_vx = args.vx * global_scale
        eff_vy = args.vy * global_scale
        eff_wz = args.wz * global_scale
        
        if scale_x < 1.0:
            warnings.append(f"Velocity Guard (X): Twist downscaled to {global_scale*100:.1f}%. (Cmd Vx={args.vx:.3f} -> {eff_vx:.3f} m/s)")
        if scale_y < 1.0:
            warnings.append(f"Velocity Guard (Y): Twist downscaled to {global_scale*100:.1f}%. (Cmd Vy={args.vy:.3f} -> {eff_vy:.3f} m/s)")
    elif len(errors) == 0:
        print(f" [OK] Twist Velocity:\tWithin leg geometric workspace limits.")

    # 3. Dynamic Step Height Check
    try:
        import io
        from contextlib import redirect_stdout
        with io.StringIO() as buf, redirect_stdout(buf):
            gait = GaitGenerator3D(stand_height=args.height, twist=[args.wz, args.vx, args.vy],
                                   step_height=args.step, period=args.period, gait_type=args.gait)
            global_step_scale = gait.planners[0].input_step_scale
            if global_step_scale is None: 
                global_step_scale = 1.0
                
        if global_step_scale < 1.0:
            eff_step = args.step * global_step_scale
            warnings.append(f"Step Height Guard: Downscaled to {global_step_scale*100:.1f}%. (Cmd H={args.step:.3f} -> {eff_step:.3f} m)")
        elif len(errors) == 0:
            print(f" [OK] Step Height: \tSwing kinematics can fully realize {args.step:.3f} m clearance.")
            
    except Exception as e:
        errors.append(f"Gait Generation failed: {str(e)}")

    print(f"\n=======================================")
    print(f" Results SUMMARY")
    print(f"=======================================")
    if len(errors) == 0 and len(warnings) == 0:
        print("✅ PASS: All parameters are safe and within physical bounds.")
    else:
        for err in errors:
            print(f"❌ ERROR: {err}")
        for warn in warnings:
            print(f"⚠️ WARNING: {warn}")

def cmd_ik(args):
    from legwheel.models.corgi_leg import CorgiLegKinematics
    kin = CorgiLegKinematics(args.leg)
    name = ['FL', 'FR', 'RR', 'RL'][args.leg]
    print(f"--- Calculate IK for Limb: {name} (Index: {args.leg}) ---")
    
    target_pos = np.array([args.x, args.y, args.z])
    print(f"Target Position [x, y, z] in {{B}}: {target_pos}")
    
    try:
        q_calc = kin.inverse_kinematics(target_pos)
        print(f"✅ IK Converged! (Theta, Beta, Gamma)")
        print(f"Joint Angles [rad]: {q_calc}")
        print(f"Joint Angles [deg]: {np.rad2deg(q_calc)}")
    except Exception as e:
        print(f"❌ ERROR: IK failed to converge. {e}")

def cmd_generate(args):
    import os
    import sys
    
    # Try importing directly from examples directory if available
    examples_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'examples'))
    if os.path.isdir(examples_dir):
        sys.path.append(examples_dir)
        try:
            from generate_hardware_csv import generate_hardware_csv
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
            return
        except ImportError:
            pass

    print("⚠️  Warning: direct import of calculate module failed, ensure you are running from source repo or package has shipped 'examples'.")

def cmd_ui(args):
    import os
    import sys
    import subprocess
    script_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'examples', 'generate_csv_ui.py'))
    if os.path.isfile(script_path):
        subprocess.run([sys.executable, script_path])
    else:
        print(f"⚠️  Warning: Could not find {script_path}")

def cmd_view(args):
    import os
    import sys
    import subprocess
    script_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'examples', 'csv_viewer.py'))
    if os.path.isfile(script_path):
        subprocess.run([sys.executable, script_path, args.csv_file])
    else:
        print(f"⚠️  Warning: Could not find {script_path}")

def main():
    parser = argparse.ArgumentParser(description="LegWheel 機器狗運動學 CLI 工具")
    subparsers = parser.add_subparsers(dest='command', help='可用的子指令')

    # Subcommand: check
    parser_check = subparsers.add_parser('check', help='檢查步態參數是否安全')
    parser_check.add_argument("--height", type=float, default=0.31, help="Target standing height (m)")
    parser_check.add_argument("--vx", type=float, default=0.15, help="Forward velocity (m/s)")
    parser_check.add_argument("--vy", type=float, default=0.0, help="Lateral velocity (m/s)")
    parser_check.add_argument("--wz", type=float, default=0.0, help="Yaw angular velocity (rad/s)")
    parser_check.add_argument("--step", type=float, default=0.04, help="Step swing clearance height (m)")
    parser_check.add_argument("--period", "-p", type=float, default=1.0, help="Gait period (s)")
    parser_check.add_argument("--gait", "-g", type=str, default="Trot", help="Gait type (Trot, Pace, Bound, etc.)")
    
    # Subcommand: ik
    parser_ik = subparsers.add_parser('ik', help='計算單腳逆運動學')
    parser_ik.add_argument('--leg', type=int, choices=[0, 1, 2, 3], default=0, help='Limb Index (0=FL, 1=FR, 2=RR, 3=RL)')
    parser_ik.add_argument('--x', type=float, required=True, help='X 座標 (m) in {B}')
    parser_ik.add_argument('--y', type=float, required=True, help='Y 座標 (m) in {B}')
    parser_ik.add_argument('--z', type=float, required=True, help='Z 座標 (m) in {B}')

    # Subcommand: generate-gait
    parser_gen = subparsers.add_parser('generate', help='產生硬體用的連續步態軌跡 CSV')
    parser_gen.add_argument("-g", "--gait", type=str, default="Trot", help="Gait type")
    parser_gen.add_argument("-vx", "--vx", type=float, default=0.0, help="Forward velocity (m/s)")
    parser_gen.add_argument("-vy", "--vy", type=float, default=0.1, help="Lateral velocity (m/s)")
    parser_gen.add_argument("-wz", "--wz", type=float, default=0.0, help="Yaw velocity (rad/s)")
    parser_gen.add_argument("-z", "--height", type=float, default=0.25, help="Standing height (m)")
    parser_gen.add_argument("-s", "--step", type=float, default=0.04, help="Step height (m)")
    parser_gen.add_argument("-p", "--period", type=float, default=4, help="Gait period (s)")
    parser_gen.add_argument("-c", "--cycles", type=int, default=10, help="Number of gait cycles")
    parser_gen.add_argument("-dt", "--dt", type=float, default=0.001, help="Time step (s)")
    parser_gen.add_argument("-o", "--outdir", type=str, default="outputs/csv", help="Output directory")

    # Subcommand: ui
    parser_ui = subparsers.add_parser('ui', help='開啟互動式 CSV 生成器 (Tkinter UI)')

    # Subcommand: view
    parser_view = subparsers.add_parser('view', help='開啟 3D 視覺化工具來播放 CSV 軌跡')
    parser_view.add_argument('csv_file', type=str, help='要播放的 CSV 檔案路徑')

    args = parser.parse_args()

    # If no command is provided, print help
    if args.command is None:
        parser.print_help()
        sys.exit(1)

    if args.command == 'check':
        cmd_check(args)
    elif args.command == 'ik':
        cmd_ik(args)
    elif args.command == 'generate':
        cmd_generate(args)
    elif args.command == 'ui':
        cmd_ui(args)
    elif args.command == 'view':
        cmd_view(args)

if __name__ == "__main__":

    main()
