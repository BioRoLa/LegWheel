from legwheel.models.corgi_leg import CorgiLegKinematics
import numpy as np

# Left Front Leg
fl = CorgiLegKinematics(True, True)
# Right Front Leg
fr = CorgiLegKinematics(False, True)

theta, beta = 90, 0
for gamma in [0, np.deg2rad(20), np.deg2rad(-20)]:
    p_fl = fl.forward_kinematics(np.deg2rad(theta), np.deg2rad(beta), gamma)
    p_fr = fr.forward_kinematics(np.deg2rad(theta), np.deg2rad(beta), gamma)
    print(f"Gamma = {np.rad2deg(gamma):.1f} deg")
    print(f"  FL: {p_fl}")
    print(f"  FR: {p_fr}")

