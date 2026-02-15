import numpy as np
import sys
import os

# Add the project root to sys.path to allow imports from legwheel
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from legwheel.models.corgi_leg import CorgiLegKinematics

def validate_transforms():
    """
    Validates the coordinate transformation functions in CorgiLegKinematics.
    Checks for:
    1. Inverse consistency (L -> M -> L and M -> B -> M)
    2. Transformation chain consistency (L -> M -> B vs. direct L -> B)
    3. Module origin consistency
    """
    legs = ["FL", "FR", "RR", "RL"]
    gammas = [0.0, np.pi/6, -np.pi/4, np.pi/2]
    test_points = [
        np.array([0.0, 0.0, 0.0]),
        np.array([0.1, 0.0, 0.0]),
        np.array([0.0, 0.1, 0.0]),
        np.array([0.0, 0.0, 0.1]),
        np.array([0.1, -0.2, 0.3]),
    ]

    print("Starting validation of CorgiLegKinematics transformations...")
    
    all_passed = True

    for i, leg_name in enumerate(legs):
        print("\n--- Validating Leg {}: {} ---".format(i, leg_name))
        kin = CorgiLegKinematics(i)
        
        for gamma in gammas:
            for p_L in test_points:
                # 1. Test Leg to Module and back
                p_M = kin._L_to_M(p_L, gamma)
                p_L_recon = kin._M_to_L(p_M, gamma)
                err_LM = np.linalg.norm(p_L - p_L_recon)
                
                # 2. Test Module to Body and back
                p_B = kin._M_to_B(p_M, gamma)
                p_M_recon = kin._B_to_M(p_B, gamma)
                err_MB = np.linalg.norm(p_M - p_M_recon)
                
                # 3. Test direct transform to body
                p_B_direct = kin._transform_to_body(p_L, gamma)
                err_direct = np.linalg.norm(p_B - p_B_direct)
                
                tolerance = 1e-10
                if err_LM > tolerance or err_MB > tolerance or err_direct > tolerance:
                    print("    FAILED for point {} at gamma={:.2f}".format(p_L, gamma))
                    print("      L->M->L error: {:.2e}".format(err_LM))
                    print("      M->B->M error: {:.2e}".format(err_MB))
                    print("      Direct transform error: {:.2e}".format(err_direct))
                    all_passed = False
                
            # 4. Consistency check for Module Origin
            mo_B = kin._M_to_B(np.array([0, 0, 0]), gamma)
            err_origin = np.linalg.norm(mo_B - kin.p_Mi_in_B)
            if err_origin > tolerance:
                print("    Origin mismatch! _M_to_B(0)={}, p_Mi_in_B={}".format(mo_B, kin.p_Mi_in_B))
                all_passed = False

    if all_passed:
        print("\nSUCCESS: All coordinate transformation validations passed!")
    else:
        print("\nFAILURE: Some validations failed. See details above.")

if __name__ == "__main__":
    validate_transforms()
