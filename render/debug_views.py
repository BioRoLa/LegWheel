import numpy as np
import matplotlib.pyplot as plt
from legwheel.models.corgi_leg import CorgiLegKinematics
import os

def plot_debug_views(filename="debug_views.png"):
    theta = np.deg2rad(110)
    beta = np.deg2rad(10)
    gamma = np.deg2rad(0) # Keep gamma 0 to verify base planes first
    
    kin = CorgiLegKinematics(0) # FL Leg
    
    fig = plt.figure(figsize=(15, 5))
    
    # 1. Front View (Y-Z plane, looking from +X)
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.view_init(elev=0, azim=0) # Front view
    ax1.set_title("Front View (From +X)")
    kin.plot_leg_3d(theta, beta, gamma, ax1)
    kin.plot_frames(ax1, gamma)
    
    # 2. Side View (X-Z plane, looking from -Y)
    ax2 = fig.add_subplot(132, projection='3d')
    ax2.view_init(elev=0, azim=-90) # Side view
    ax2.set_title("Side View (From -Y)")
    kin.plot_leg_3d(theta, beta, gamma, ax2)
    kin.plot_frames(ax2, gamma)

    # 3. Top View (X-Y plane, looking from +Z)
    ax3 = fig.add_subplot(133, projection='3d')
    ax3.view_init(elev=90, azim=-90) # Top view
    ax3.set_title("Top View (From +Z)")
    kin.plot_leg_3d(theta, beta, gamma, ax3)
    kin.plot_frames(ax3, gamma)

    # Save
    path = os.path.join("render", filename)
    plt.tight_layout()
    plt.savefig(path)
    print(f"Saved debug views to {path}")
    plt.close()

if __name__ == "__main__":
    plot_debug_views()
