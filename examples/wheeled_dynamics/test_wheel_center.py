import sys
from pathlib import Path
import numpy as np

project_root = Path(__file__).resolve().parent
sys.path.append(str(project_root))

from legwheel.models.corgi_robot import CorgiRobot

robot = CorgiRobot()
gamma_bank = np.deg2rad(20)
q_list = [
    [np.deg2rad(75.0), 0.0, gamma_bank],
    [np.deg2rad(75.0), 0.0, -gamma_bank],
    [np.deg2rad(75.0), 0.0, -gamma_bank],
    [np.deg2rad(75.0), 0.0, gamma_bank]
]

for roll_deg in [-10, 0, 10]:
    robot.base_ori = [np.deg2rad(roll_deg), 0, 0]
    
    # Left foot (FL) center
    fl_B = robot.legs[0].forward_kinematics(*q_list[0])
    fl_W = robot.body_to_world(fl_B)
    
    # Right foot (FR) center
    fr_B = robot.legs[1].forward_kinematics(*q_list[1])
    fr_W = robot.body_to_world(fr_B)
    
    print(f"Roll: {roll_deg:3d} | FL_W Z: {fl_W[2]:6.3f} | FR_W Z: {fr_W[2]:6.3f}")

print("FR_B:", fr_B)
print("FL_B:", fl_B)
