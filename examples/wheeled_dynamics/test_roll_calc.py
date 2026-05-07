import sys
from pathlib import Path
import numpy as np

project_root = Path(__file__).resolve().parent
sys.path.append(str(project_root))

from legwheel.models.corgi_robot import CorgiRobot
from legwheel.models.collision_model import CorgiCollisionModel
from scipy.optimize import minimize_scalar

robot = CorgiRobot()
collision = CorgiCollisionModel(robot)

def get_roll_for_gamma(gamma_bank_deg):
    gamma_bank = np.deg2rad(gamma_bank_deg)
    
    # Left turn banking: left side drops (abducts), right side rises (adducts)
    # Left legs: gamma > 0 (Abduction)
    # Right legs: gamma < 0 (Adduction)
    theta = np.deg2rad(75.0)
    beta = np.deg2rad(0.0)
    
    q_FL = [theta, beta, gamma_bank]
    q_FR = [theta, beta, -gamma_bank]
    q_RR = [theta, beta, -gamma_bank]
    q_RL = [theta, beta, gamma_bank]
    
    q_list = [q_FL, q_FR, q_RR, q_RL]
    
    def objective(roll_deg):
        robot.base_ori = [np.deg2rad(roll_deg), 0, 0]
        wheel_pts_W = collision.get_wheel_contact_points(q_list)
        # FL:0, FR:1, RR:2, RL:3
        left_z = min(wheel_pts_W[0][2], wheel_pts_W[3][2])
        right_z = min(wheel_pts_W[1][2], wheel_pts_W[2][2])
        return (left_z - right_z)**2

    res = minimize_scalar(objective, bounds=(-45, 45), method='bounded')
    return res.x

print(f"Banking command: 20 deg")
print(f"Induced body roll: {get_roll_for_gamma(20):.2f} deg")
print(f"Banking command: 10 deg")
print(f"Induced body roll: {get_roll_for_gamma(10):.2f} deg")
print(f"Banking command: 30 deg")
print(f"Induced body roll: {get_roll_for_gamma(30):.2f} deg")
