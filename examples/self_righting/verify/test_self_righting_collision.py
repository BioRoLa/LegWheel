import numpy as np
from legwheel.models.corgi_robot import CorgiRobot
from legwheel.models.collision_model import CorgiCollisionModel

def test_self_righting_scenario(scenario_name, roll_deg, pitch_deg, q_list):
    print(f"\n==============================================")
    print(f"Scenario: {scenario_name}")
    print(f"Base Pose: Roll={roll_deg}°, Pitch={pitch_deg}°")
    
    # 1. Initialize Robot and Collision Model
    robot = CorgiRobot()
    robot.base_ori = np.array([np.deg2rad(roll_deg), np.deg2rad(pitch_deg), 0.0])
    # Assume base pos is high enough, we will calculate the lowest Z and subtract it
    robot.base_pos = np.array([0, 0, 0.5]) 
    
    collision_model = CorgiCollisionModel(robot)
    
    # 2. Calculate Collision Points
    min_z, contact_points = collision_model.find_contact_pivots(q_list)
    
    # 3. Ground Correction (Shift everything so lowest point is Z=0)
    print(f"\nLowest point was at Z = {min_z:.4f} m (shifting to Ground Z=0)")
    robot.base_pos[2] -= min_z
    
    # Recalculate with corrected Z
    _, corrected_contacts = collision_model.find_contact_pivots(q_list)
    
    print("\n--- Contact Pivot Points (Z ≈ 0) ---")
    for cp in corrected_contacts:
        p = cp["pos"]
        print(f"  {cp['label']:<15} -> X: {p[0]:.4f}, Y: {p[1]:.4f}, Z: {p[2]:.4f}")

    # Also list the CoM in World Frame (assuming CoM is at Body Frame origin with bias)
    com_B = np.array([0, 0, 0])
    com_W = robot.body_to_world(com_B)
    print(f"\n--- Center of Mass (CoM) ---")
    print(f"  CoM -> X: {com_W[0]:.4f}, Y: {com_W[1]:.4f}, Z: {com_W[2]:.4f}")

if __name__ == "__main__":
    # Joint Configuration: [theta, beta, gamma]
    # Default standing posture, roughly
    q_stand = [np.deg2rad(60), np.deg2rad(90), np.deg2rad(0)]
    q_list = [q_stand, q_stand, q_stand, q_stand]
    
    # Scenario A: Side-Fall (Roll = 90 deg)
    test_self_righting_scenario("Side-Fall (Roll 90°)", roll_deg=90, pitch_deg=0, q_list=q_list)
    
    # Scenario B: Upside-Down (Roll = 180 deg)
    test_self_righting_scenario("Upside-Down (Roll 180°)", roll_deg=180, pitch_deg=0, q_list=q_list)
    
    # Scenario C: Side-Fall with ABAD pushing (Gamma = 30 deg on side legs)
    q_push = [np.deg2rad(60), np.deg2rad(90), np.deg2rad(30)] # Gamma pushing out
    q_list_push = [q_stand, q_stand, q_stand, q_stand]
    # Left legs are pushing ground (index 0, 3)
    q_list_push[0] = q_push
    q_list_push[3] = q_push
    
    test_self_righting_scenario("Side-Fall Pushing (Left legs Gamma 30°)", roll_deg=90, pitch_deg=0, q_list=q_list_push)
