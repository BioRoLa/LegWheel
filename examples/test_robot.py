import numpy as np
from legwheel.models.corgi_robot import CorgiRobot

def main():
    print("--- Testing CorgiRobot Object ---")
    robot = CorgiRobot()
    
    # 1. Test Forward Kinematics for all legs
    # Standing pose: theta=110 deg, beta=0, gamma=0
    q_stand = [np.deg2rad(110), 0.0, 0.0]
    q_all = [q_stand for _ in range(4)]
    
    feet_R = robot.get_leg_positions(q_all)
    print(f"\nFeet positions in Robot Frame {{R}} (Standing Pose):")
    for i, name in enumerate(['FL', 'FR', 'RR', 'RL']):
        print(f" {name}: {np.round(feet_R[i], 4)}")
        
    # 2. Test World Transformation
    # Place robot at z=0.3m, Tilt pitch by 5 degrees
    robot.base_pos = np.array([0, 0, 0.3])
    robot.base_ori = np.array([0, np.deg2rad(5), 0])
    
    foot_W = robot.robot_to_world(feet_R[0])
    print(f"\nFL Foot in World Frame {{W}} (z=0.3, pitch=5 deg):")
    print(f" {np.round(foot_W, 4)}")
    
    # 3. Test Full Robot IK
    print(f"\nRunning Full Robot IK...")
    # Target: reach the current world positions
    # Provide a guess slightly offset from q_stand to verify convergence
    guess_q = [np.array(q_stand) + np.deg2rad(2) for _ in range(4)]
    recovered_q = robot.inverse_kinematics(feet_R, guess_q=guess_q)
    
    print(f"Recovered angles for FL (deg): {np.rad2deg(recovered_q[0])}")
    
    error = np.linalg.norm(np.array(q_stand) - np.array(recovered_q[0]))
    if error < 1e-4:
        print("Success: IK recovered consistent joint angles.")
    else:
        print(f"IK Diverged. Error: {error}")

if __name__ == "__main__":
    main()
