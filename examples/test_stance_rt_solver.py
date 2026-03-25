from legwheel.planners.trajectory_planning_3d import TrajectoryPlanner3D
import numpy as np
import sys
import os

# Add the project root to sys.path
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..')))


def test_stance_rt_solver():
    print("Testing TrajectoryPlanner3D.stance_rt_solver (Rolling Jacobian §5.3.2)...")

    # Initialize planner for Leg 0 (FL)
    planner = TrajectoryPlanner3D(
        leg_index=0, stand_height=0.3, velocity=[0.15, 0.0, 0.0])

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

            # Verify via FK: the foot position change should approximate -v_hip * dt
            contact_0 = planner.kin.foot_rim_contact_fk(*q)
            FK_0 = planner.kin.forward_kinematics(
                *q, alpha=contact_0[0], w=contact_0[1])

            contact_1 = planner.kin.foot_rim_contact_fk(*q_next)
            FK_1 = planner.kin.forward_kinematics(
                *q_next, alpha=contact_1[0], w=contact_1[1])

            foot_delta = FK_1 - FK_0
            expected_delta = -v_hip * planner.dt
            error = np.linalg.norm(foot_delta - expected_delta)

            print(f"  Foot delta: {foot_delta}")
            print(f"  Expected:   {expected_delta}")
            print(f"  Error: {error:.2e}")
            q = q_next

        except Exception as e:
            print(f"  FAILED at step {i+1}: {e}")
            break


def test_leg_twist():
    print("\nTesting compute_leg_twist (§5.3.2 Twist Synthesis)...")

    planner = TrajectoryPlanner3D(
        leg_index=0, stand_height=0.3, velocity=[0.15, 0.0, 0.0])

    q = np.array([np.deg2rad(50), np.deg2rad(10), 0.0])
    v_hip = np.array([0.1, 0.0, 0.0])

    # First get joint velocities from stance solver
    q_next = planner.stance_rt_solver(v_hip=v_hip, q=q)
    q_dot = (q_next - q) / planner.dt

    print(f"q_dot: {q_dot}")

    # Compute twist
    V_matrix, V_screw = planner.compute_leg_twist(q, q_dot)

    print(f"Twist matrix:\n{np.round(V_matrix, 6)}")
    print(f"Twist screw: {V_screw}")

    # Verify: twist matrix should be 4x4 with antisymmetric rotation block
    R_block = V_matrix[:3, :3]
    skew_check = np.linalg.norm(R_block + R_block.T)
    print(f"Skew-symmetric check (should be ~0): {skew_check:.2e}")

    if skew_check < 1e-10:
        print("RESULT: Twist matrix rotation block is correctly antisymmetric.")
    else:
        print("RESULT: WARNING - Rotation block is NOT antisymmetric!")


if __name__ == "__main__":
    test_stance_rt_solver()
    test_leg_twist()
