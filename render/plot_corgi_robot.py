import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.config import RobotParams

def plot_corgi_robot(theta=np.deg2rad(90), beta=0.0, gamma=0.0):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 1. Plot Robot Chassis (Box)
    l = RobotParams.WHEEL_BASE
    w = RobotParams.BODY_WIDTH
    h = 0.05 # Placeholder thickness
    
    # Simple rectangle for body
    corners = np.array([
        [l/2, w/2, 0],
        [l/2, -w/2, 0],
        [-l/2, -w/2, 0],
        [-l/2, w/2, 0],
        [l/2, w/2, 0]
    ])
    ax.plot(corners[:, 0], corners[:, 1], corners[:, 2], 'k-', linewidth=2, label="Body")
    
    # 2. Plot Legs
    colors = ['r', 'g', 'b', 'm']
    labels = ['FL', 'FR', 'RR', 'RL']
    
    for i in range(4):
        kin = CorgiLegKinematics(i)
        
        # Hip position (Origin of Module Frame in Robot Frame)
        hip = kin.p_Mi_in_R
        
        # Contact point (calculated via FK)
        # Note: We pass the joint angles to the 3D FK
        contact = kin.forward_kinematics(theta, beta, gamma)
        
        # Plot Hip-to-Contact line (Simplified leg visualization)
        ax.plot([hip[0], contact[0]], 
                [hip[1], contact[1]], 
                [hip[2], contact[2]], 
                color=colors[i], marker='o', label=labels[i])
        
        # Plot Hip joint
        ax.scatter(hip[0], hip[1], hip[2], color='k', s=20)

    # 3. Finalize Plot
    ax.set_xlabel('X (Front)')
    ax.set_ylabel('Y (Left)')
    ax.set_zlabel('Z (Up)')
    ax.set_title(f'Corgi Robot 3D Visualization\ntheta={np.rad2deg(theta):.1f}°, beta={np.rad2deg(beta):.1f}°, gamma={np.rad2deg(gamma):.1f}°')
    
    # Equal aspect ratio
    max_range = np.array([l, w, 0.5]).max() / 2.0
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(-max_range, max_range)
    
    ax.legend()
    plt.show()

if __name__ == "__main__":
    # Test with standard standing pose
    plot_corgi_robot(theta=np.deg2rad(90), beta=0.0, gamma=np.deg2rad(15))
