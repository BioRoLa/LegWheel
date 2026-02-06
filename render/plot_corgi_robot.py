import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.config import RobotParams

def plot_corgi_robot(theta=np.deg2rad(90), beta=0.0, gamma=0.0):
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 1. Plot Robot Chassis (Box)
    l = RobotParams.WHEEL_BASE
    w = RobotParams.BODY_WIDTH
    
    # Simple rectangle for body at z=0 (in robot frame {R})
    corners = np.array([
        [l/2, w/2, 0],
        [l/2, -w/2, 0],
        [-l/2, -w/2, 0],
        [-l/2, w/2, 0],
        [l/2, w/2, 0]
    ])
    ax.plot(corners[:, 0], corners[:, 1], corners[:, 2], 'k-', linewidth=3, label="Body")
    
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
    ax.set_zlim(-0.4, 0.1)
    
    plt.show()

if __name__ == "__main__":
    # Test with standard standing pose and some ABAD angle
    plot_corgi_robot(theta=np.deg2rad(110), beta=np.deg2rad(10), gamma=np.deg2rad(15))