import numpy as np
from legwheel.utils.screw import Screw

def main():
    print("--- Testing Screw Utility ---")
    
    # 1. Create a pure rotation screw around Z-axis passing through (0.1, 0, 0)
    q = np.array([0.1, 0, 0])
    s = np.array([0, 0, 1])
    h = 0.0 # pitch 0
    
    unit_screw = Screw.from_axis(q, s, h)
    print(f"Unit Screw (omega, v): {unit_screw}")
    
    # 2. Exponential mapping (Rotate 90 degrees)
    theta = np.pi / 2
    T = unit_screw.exp6(theta)
    print("\nTransformation Matrix (90 deg rotation around offset axis):")
    print(np.round(T, 4))
    
    # 3. Test Adjoint Transformation
    # Move the frame along X by 0.5
    T_move = np.eye(4)
    T_move[0, 3] = 0.5
    
    transformed_screw = unit_screw.transform(T_move)
    print(f"\nTransformed Screw (moved by 0.5 in X): {transformed_screw}")
    
    # Expected: omega stays [0,0,1], v should change due to leverage arm
    
    # 4. Pure Translation Screw
    v_trans = np.array([0, 0, 0, 1, 0, 0]) # Translate along X
    screw_trans = Screw(v_trans)
    T_trans = screw_trans.exp6(0.2)
    print("\nPure Translation Transformation (0.2m along X):")
    print(np.round(T_trans, 4))

if __name__ == "__main__":
    main()
