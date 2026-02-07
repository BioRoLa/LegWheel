import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.config import RobotParams

def plot_corgi_robot(theta=np.deg2rad(90), beta=0.0, gamma=0.0):
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 1. Plot Robot Chassis (Octagonal Prism)
    # Geometric center at (0, 0, ABAD_AXIS_OFFSET)
    l = RobotParams.CHASSIS_LENGTH
    w = RobotParams.CHASSIS_WIDTH
    h = RobotParams.CHASSIS_HEIGHT
    z_chassis = RobotParams.ABAD_AXIS_OFFSET
    
    # Define cross-section octagon in Y-Z plane based on view description:
    # front view:   ____    | top view:   ____    | side view:   
    #             /      \  |            |    |   |         _____________      
    #            |        | |            |    |   |        |_____________|      
    #             \ ____ /  |            |____|   |              
    
    c = 0.04 # Chamfer distance for the Y-Z cross-section
    # 8 vertices of the octagon cross-section
    y_points = np.array([w/2 - c, w/2, w/2, w/2 - c, -w/2 + c, -w/2, -w/2, -w/2 + c, w/2 - c])
    z_points = np.array([h/2, h/2 - c, -h/2 + c, -h/2, -h/2, -h/2 + c, h/2 - c, h/2, h/2]) + z_chassis
    
    # Plot Front Face (at x = l/2)
    ax.plot(np.full_like(y_points, l/2), y_points, z_points, 'k-', linewidth=2)
    # Plot Rear Face (at x = -l/2)
    ax.plot(np.full_like(y_points, -l/2), y_points, z_points, 'k-', linewidth=2)
    # Plot Longitudinal Edges (connecting front and rear)
    for i in range(8):
        ax.plot([l/2, -l/2], [y_points[i], y_points[i]], [z_points[i], z_points[i]], 'k-', linewidth=1)
    
    # 2. Plot Detailed Legs and Frames
    for i in range(4):
        kin = CorgiLegKinematics(i)
        # Use the detailed 3D plot method from the model
        kin.plot_leg_3d(theta, beta, gamma, ax)
        # Plot coordinate frames
        kin.plot_frames(ax, gamma)

    # 3. Finalize Plot
    ax.set_xlabel('X (Front)')
    ax.set_ylabel('Y (Left)')
    ax.set_zlabel('Z (Up)')
    ax.set_title(f'Corgi Robot 3D Visualization (Mechanism & Frames)\ntheta={np.rad2deg(theta):.1f}°, beta={np.rad2deg(beta):.1f}°, gamma={np.rad2deg(gamma):.1f}°')
    
    # Equal aspect ratio
    max_range = 0.4
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(-0.4, 0.2)
    
    plt.show()

if __name__ == "__main__":
    # Test with standard standing pose and some ABAD angle
    plot_corgi_robot(theta=np.deg2rad(110), beta=np.deg2rad(10), gamma=np.deg2rad(15))