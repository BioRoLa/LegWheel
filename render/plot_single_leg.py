import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.config import RobotParams

def plot_single_leg(leg_index=0, theta=np.deg2rad(90), beta=0.0, gamma=0.0):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    kin = CorgiLegKinematics(leg_index)
    label = ['FL', 'FR', 'RR', 'RL'][leg_index]
    
    # 1. Plot the leg mechanism
    kin.plot_leg_3d(theta, beta, gamma, ax)
    
    # 2. Plot Module Frame Origin (Roll Axis) in Body Frame {B}
    mo = kin.p_Mi_in_B
    ax.scatter(mo[0], mo[1], mo[2], color='k', s=50, label='Module Origin')
    
    # 3. Visualization of local frames
    kin.plot_frames(ax, gamma)

    # 4. Finalize Plot
    ax.set_xlabel('X (Front)')
    ax.set_ylabel('Y (Left)')
    ax.set_zlabel('Z (Up)')
    ax.set_title(f'Leg {label} Debug Plot (Body Frame {{B}})\ntheta={np.rad2deg(theta):.1f}°, beta={np.rad2deg(beta):.1f}°, gamma={np.rad2deg(gamma):.1f}°')
    
    # Equal aspect ratio
    max_range = 0.3
    ax.set_xlim(mo[0]-max_range, mo[0]+max_range)
    ax.set_ylim(mo[1]-max_range, mo[1]+max_range)
    ax.set_zlim(-0.4, 0.1)
    
    ax.legend()
    plt.show()

if __name__ == "__main__":
    # Debug Front Right Leg (FR) - index 1
    # Right legs are often where symmetry bugs appear
    plot_single_leg(leg_index=1, theta=np.deg2rad(90), beta=0.0, gamma=np.deg2rad(20))