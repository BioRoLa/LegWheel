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
    
    # 2. Plot Module Frame Origin (Roll Axis)
    mo = kin.p_Mi_in_R
    ax.scatter(mo[0], mo[1], mo[2], color='k', s=50, label='Module Origin')
    
    # 3. Plot Local Axes for Module Frame {Mi}
    # To see how Mi is oriented in R
    axis_len = 0.1
    # We need to compute where the axes point in R
    # In CorgiLegKinematics, p_R = R_M_to_R @ p_M + p_Mi
    # So unit vectors are the columns of R_M_to_R (which isn't explicitly stored as an attr but is in _transform)
    
    # Let's extract R_M_to_R manually for visualization based on the current code
    if kin.is_left:
        R_M_to_R = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
    else:
        R_M_to_R = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        
    x_axis = R_M_to_R[:, 0] * axis_len
    y_axis = R_M_to_R[:, 1] * axis_len
    z_axis = R_M_to_R[:, 2] * axis_len
    
    ax.quiver(mo[0], mo[1], mo[2], x_axis[0], x_axis[1], x_axis[2], color='r', label='X_M (Lateral)')
    ax.quiver(mo[0], mo[1], mo[2], y_axis[0], y_axis[1], y_axis[2], color='g', label='Y_M (Up)')
    ax.quiver(mo[0], mo[1], mo[2], z_axis[0], z_axis[1], z_axis[2], color='b', label='Z_M (Longitudinal)')

    # 4. Finalize Plot
    ax.set_xlabel('X (Front)')
    ax.set_ylabel('Y (Left)')
    ax.set_zlabel('Z (Up)')
    ax.set_title(f'Leg {label} Debug Plot\ntheta={np.rad2deg(theta):.1f}°, beta={np.rad2deg(beta):.1f}°, gamma={np.rad2deg(gamma):.1f}°')
    
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
