#!/usr/bin/env python3
"""
check_parameters.py

A simple utility script to check if the given gait parameters will trigger
the built-in safety boundaries and scaling guards inside the LegWheel library.

Usage:
    python check_parameters.py --height 0.3 --vx 0.15 --vy 0.0 --period 1.0 --gait Trot
"""

import argparse
import sys
import numpy as np

# Adjust path if script is run directly from examples folder
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from legwheel.planners.gait_generator_3d import GaitGenerator3D, GAIT_LIBRARY
from legwheel.config import RobotParams
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.utils.fitted_coefficient import inv_G_dist_poly

def main():
    parser = argparse.ArgumentParser(description="LegWheel Trajectory Parameter Checker")
    parser.add_argument("--height", type=float, default=0.31, help="Target standing height (m)")
    parser.add_argument("--vx", type=float, default=0.15, help="Forward velocity (m/s)")
    parser.add_argument("--vy", type=float, default=0.0, help="Lateral velocity (m/s)")
    parser.add_argument("--wz", type=float, default=0.0, help="Yaw angular velocity (rad/s)")
    parser.add_argument("--step", type=float, default=0.04, help="Step swing clearance height (m)")
    parser.add_argument("--period", "-p", type=float, default=1.0, help="Gait period (s)")
    parser.add_argument("--gait", "-g", type=str, default="Trot", choices=list(GAIT_LIBRARY.keys()), help="Gait type")
    
    args = parser.parse_args()

    # Get gait default duty
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
    R_link = RobotParams.WHEEL_RADIUS_PITCH * 0.2225 # approx offset derived from geometric relations
    
    # Calculate initial theta based on static height
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


    # 2. Twist Workspace Check (Velocity scaling)
    L1 = RobotParams.WHEEL_RADIUS_PITCH * RobotParams.L1_RATIO
    R_arc = np.sqrt(L1**2 - R_link**2)
    BETA_MAX = np.deg2rad(40)
    GAMMA_MAX = np.deg2rad(8)

    D_x_max = 2 * H_O * np.tan(BETA_MAX) + 2 * R_arc * BETA_MAX
    v_x_limit = D_x_max / (args.period * stance_duty)
    
    D_y_max = 2 * args.height * np.sin(GAMMA_MAX)
    v_y_limit = D_y_max / (args.period * stance_duty)

    # Compute per-leg velocities at hip mounts
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
            warnings.append(f"Velocity Guard (X): Leg requires > 40° swing beta. Twist downscaled to {global_scale*100:.1f}%. (Cmd Vx={args.vx:.3f} -> {eff_vx:.3f} m/s)")
        if scale_y < 1.0:
            warnings.append(f"Velocity Guard (Y): Leg requires > 8° lateral gamma. Twist downscaled to {global_scale*100:.1f}%. (Cmd Vy={args.vy:.3f} -> {eff_vy:.3f} m/s)")
    elif len(errors) == 0:
        print(f" [OK] Twist Velocity:\tWithin leg geometric workspace limits.")

    # 3. Dynamic Step Height Check (Scale down if workspace is heavily utilized)
    try:
        # We can leverage the actual GaitGenerator to test what it outputs.
        # It suppresses stdout or we just use it and rely on its side effects.
        # Note: Suppress GaitGenerator3D prints during internal check
        import io
        from contextlib import redirect_stdout
        with io.StringIO() as buf, redirect_stdout(buf):
            gait = GaitGenerator3D(stand_height=args.height, twist=[args.wz, args.vx, args.vy],
                                   step_height=args.step, period=args.period, gait_type=args.gait)
            global_step_scale = gait.planners[0].input_step_scale
            if global_step_scale is None: 
                global_step_scale = 1.0 # Fallback in case attribute structure changes
                
        if global_step_scale < 1.0:
            eff_step = args.step * global_step_scale
            warnings.append(f"Step Height Guard: Heavy lateral/sagittal usage. Step clearance downscaled to {global_step_scale*100:.1f}%. (Cmd H={args.step:.3f} -> {eff_step:.3f} m)")
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
            
if __name__ == "__main__":
    main()
