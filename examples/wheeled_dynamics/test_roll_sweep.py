import sys
from pathlib import Path
import numpy as np

project_root = Path(__file__).resolve().parent
sys.path.append(str(project_root))

from legwheel.models.corgi_robot import CorgiRobot
from legwheel.models.collision_model import CorgiCollisionModel

robot = CorgiRobot()
collision = CorgiCollisionModel(robot)

gamma_bank = np.deg2rad(20)
q_list = [
    [np.deg2rad(75.0), 0.0, gamma_bank],
    [np.deg2rad(75.0), 0.0, -gamma_bank],
    [np.deg2rad(75.0), 0.0, -gamma_bank],
    [np.deg2rad(75.0), 0.0, gamma_bank],
]

for roll_deg in [-15, -10, -5, 0, 5, 10, 15]:
    robot.base_ori = [np.deg2rad(roll_deg), 0, 0]
    pts = collision.get_wheel_contact_points(q_list)
    left_z = min(pts[0][2], pts[3][2])
    right_z = min(pts[1][2], pts[2][2])
    print(f"Roll: {roll_deg:3d} | Left Z: {left_z:6.3f} | Right Z: {right_z:6.3f} | Diff: {left_z - right_z:6.3f}")

