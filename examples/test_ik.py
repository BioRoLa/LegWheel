"""
Inverse Kinematics Test Script for Corgi Robot.
This script verifies the accuracy of the IK solver by performing a 
Round-Trip test: Joint Angles -> FK -> Cartesian Target -> IK -> Recovered Angles.
"""

import numpy as np
from legwheel.models.corgi_leg import CorgiLegKinematics

def test_leg_ik(leg_index):
    limb_names = ['FL', 'FR', 'RR', 'RL']
    name = limb_names[leg_index]
    print(f"\n--- Testing IK for Limb: {name} (Index {leg_index}) ---")
    
    kin = CorgiLegKinematics(leg_index)
    
    # 1. Define test joint angles [theta, beta, gamma]
    # theta ~ 110 deg (extended), beta ~ 10 deg (tilted), gamma ~ 15 deg (abducted)
    q_true = np.array([np.deg2rad(110), np.deg2rad(10), np.deg2rad(15)])
    
    # 2. Calculate Forward Kinematics to get the "Ground Truth" target position
    target_pos = kin.forward_kinematics(q_true[0], q_true[1], q_true[2])
    print(f"Target Position {name} [x, y, z] in {{R}}: {target_pos}")
    
    # 3. Run Inverse Kinematics
    # We provide a slight offset from the true angles as an initial guess to test convergence
    guess = q_true + np.deg2rad(5) 
    q_calc = kin.inverse_kinematics(target_pos, guess_q=guess)
    
    # 4. Verification
    # Re-calculate FK from the IK result
    final_pos = kin.forward_kinematics(*q_calc)
    
    pos_error = np.linalg.norm(target_pos - final_pos)
    angle_error = np.linalg.norm(q_true - q_calc)
    
    print(f"Recovered Angles [deg]: {np.rad2deg(q_calc)}")
    print(f"Position Error: {pos_error:.6e} m")
    print(f"Angle RMS Error: {np.rad2deg(angle_error):.6e} deg")
    
    if pos_error < 1e-4:
        print("RESULT: IK successfully converged to the target position.")
    else:
        print("RESULT: IK failed to converge within tolerance.")

if __name__ == "__main__":
    # Test a Left leg (FL)
    test_leg_ik(0)
    
    # Test a Right leg (FR) to verify symmetry logic
    test_leg_ik(1)
