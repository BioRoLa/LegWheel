import numpy as np
import sys
import os

# Add the project root to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from legwheel.planners.trajectory_planning_3d import TrajectoryPlanner3D

def test_stance_rt_solver():
    print("Testing TrajectoryPlanner3D.stance_rt_solver...")
    
    # Initialize planner for Leg 0 (FL)
    planner = TrajectoryPlanner3D(leg_index=0, stand_height=0.3, step_length=0.4)
    
    # Initial state
    q_current = np.array([np.deg2rad(50), np.deg2rad(10), 0.0])
    v_hip = np.array([0.1, 0.0, 0.0])  # Moving forward at 0.1 m/s
    
    print(f"Initial q: {q_current}")
    print(f"Hip velocity: {v_hip}")
    
    # Run for a few steps
    n_steps = 5
    q = q_current
    
    for i in range(n_steps):
        try:
            q_next = planner.stance_rt_solver(v_hip=v_hip, q=q)
            print(f"Step {i+1}: q = {q_next}")
            
            # Check convergence by calculating the cost at the result
            # We'll re-calculate the cost logic here to verify
            contact_0 = planner.kin.foot_rim_contact_fk(*q)
            FK_0 = planner.kin.forward_kinematics(*q, alpha=contact_0[0], w=contact_0[1])
            
            contact_1 = planner.kin.foot_rim_contact_fk(*q_next)
            FK_1 = planner.kin.forward_kinematics(*q_next, alpha=contact_1[0], w=contact_1[1])
            
            hip_mov = v_hip * planner.dt
            # Rolling compensation
            roll_dist = (contact_1[0] - contact_0[0]) * planner.kin.solver.foot_radius
            err_vec = FK_0 - FK_1 - hip_mov + np.array([roll_dist, 0, 0])
            error = np.linalg.norm(err_vec)
            
            print(f"  Error: {error:.2e}")
            q = q_next
            
        except Exception as e:
            print(f"  FAILED at step {i+1}: {e}")
            break

if __name__ == "__main__":
    test_stance_rt_solver()
