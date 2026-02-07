import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.config import RobotParams

def draw_corgi_robot(ax, theta=np.deg2rad(90), beta=0.0, gamma=0.0, show_axes=True):
    """
    Draws the Corgi robot (chassis + legs + frames) on the provided 3D axis.
    Reused by plotting and animation scripts.
    """
    # 1. Plot Robot Chassis (Octagonal Prism)
    l = RobotParams.CHASSIS_LENGTH
    w = RobotParams.CHASSIS_WIDTH
    h = RobotParams.CHASSIS_HEIGHT
    z_chassis = RobotParams.ABAD_AXIS_OFFSET
    
    c = 0.04 # Chamfer distance for cross-section
    y_points = np.array([w/2 - c, w/2, w/2, w/2 - c, -w/2 + c, -w/2, -w/2, -w/2 + c, w/2 - c])
    z_points = np.array([h/2, h/2 - c, -h/2 + c, -h/2, -h/2, -h/2 + c, h/2 - c, h/2, h/2]) + z_chassis
    
    # Plot Faces and Edges
    ax.plot(np.full_like(y_points, l/2), y_points, z_points, 'k-', linewidth=2)
    ax.plot(np.full_like(y_points, -l/2), y_points, z_points, 'k-', linewidth=2)
    for i in range(8):
        ax.plot([l/2, -l/2], [y_points[i], y_points[i]], [z_points[i], z_points[i]], 'k-', linewidth=1)
    
    # 2. Plot Detailed Legs and Frames
    for i in range(4):
        kin = CorgiLegKinematics(i)
        kin.plot_leg_3d(theta, beta, gamma, ax)
        kin.plot_frames(ax, gamma)

    # 3. Finalize Plot Style
    if show_axes:
        ax.set_xlabel('X (Front)')
        ax.set_ylabel('Y (Left)')
        ax.set_zlabel('Z (Up)')
        ax.grid(True)
    else:
        ax.set_axis_off()
        ax.grid(False)
    
    max_range = 0.4
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(-0.4, 0.2)
    ax.set_box_aspect([1,1,0.75])

def plot_corgi_robot(theta=np.deg2rad(90), beta=0.0, gamma=0.0):
    """Creates a static 3D plot of the Corgi robot."""
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    draw_corgi_robot(ax, theta, beta, gamma)
    
    ax.set_title(f'Corgi Robot 3D Visualization\ntheta={np.rad2deg(theta):.1f}°, beta={np.rad2deg(beta):.1f}°, gamma={np.rad2deg(gamma):.1f}°')
    plt.show()

if __name__ == "__main__":
    plot_corgi_robot(theta=np.deg2rad(110), beta=np.deg2rad(10), gamma=np.deg2rad(15))