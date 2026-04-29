import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

from legwheel.models.corgi_robot import CorgiRobot
from legwheel.models.collision_model import CorgiCollisionModel

def plot_scenario(ax, title, roll_deg, pitch_deg, q_list):
    """
    Plots the collision model bounding boxes, M6 studs, wheel contacts,
    and highlights the ground contact pivots in 3D.
    """
    # 1. Initialize Robot and Collision Model
    robot = CorgiRobot()
    robot.base_ori = np.array([np.deg2rad(roll_deg), np.deg2rad(pitch_deg), 0.0])
    robot.base_pos = np.array([0, 0, 0.5]) 
    
    col_model = CorgiCollisionModel(robot)
    
    # 2. Shift to ground
    min_z, contact_points = col_model.find_contact_pivots(q_list)
    robot.base_pos[2] -= min_z
    
    # 3. Get Points in World Frame
    pts = col_model.get_all_collision_points(q_list)
    chassis_pts = pts["chassis"]
    stud_pts = pts["m6_studs"]
    wheel_pts = pts["wheels"]
    
    # 4. Plot Chassis Wireframe
    for i in range(8):
        nxt = (i + 1) % 8
        ax.plot([chassis_pts[i,0], chassis_pts[nxt,0]], 
                [chassis_pts[i,1], chassis_pts[nxt,1]], 
                [chassis_pts[i,2], chassis_pts[nxt,2]], 'k-', lw=1.5, alpha=0.6)
        ax.plot([chassis_pts[i+8,0], chassis_pts[nxt+8,0]], 
                [chassis_pts[i+8,1], chassis_pts[nxt+8,1]], 
                [chassis_pts[i+8,2], chassis_pts[nxt+8,2]], 'k-', lw=1.5, alpha=0.6)
        ax.plot([chassis_pts[i,0], chassis_pts[i+8,0]], 
                [chassis_pts[i,1], chassis_pts[i+8,1]], 
                [chassis_pts[i,2], chassis_pts[i+8,2]], 'k-', lw=1.5, alpha=0.6)

    # 4.5 Plot Full Leg Mechanism (PlotLeg)
    # We dynamically patch the leg's transform function to output World Frame coordinates
    # so that the built-in plot_leg_3d renders correctly tilted and positioned on the ground.
    for i in range(4):
        leg = robot.legs[i]
        theta, beta, gamma = q_list[i]
        
        orig_transform = leg._transform_to_body
        
        def make_world_transform(orig_func):
            def wrapper(p_L, gamma_val=None, type="pos"):
                p_B = orig_func(p_L, gamma_val, type)
                if type == "pos":
                    return robot.body_to_world(p_B)
                else:
                    R_W = robot._rot_matrix(robot.base_ori)
                    return R_W @ p_B
            return wrapper
            
        leg._transform_to_body = make_world_transform(orig_transform)
        
        # This will now plot in World Frame
        leg.plot_leg_3d(theta, beta, gamma, ax)
        
        # Restore original function
        leg._transform_to_body = orig_transform

    # 5. Plot M6 Studs (Red) and Wheel Bottoms (Blue)
    ax.scatter(stud_pts[:,0], stud_pts[:,1], stud_pts[:,2], c='red', s=50, marker='o', label='M6 Studs')
    ax.scatter(wheel_pts[:,0], wheel_pts[:,1], wheel_pts[:,2], c='blue', s=50, marker='^', label='Wheel Bottoms')
    
    # 6. Plot Center of Mass (CoM)
    com = robot.body_to_world(np.array([0,0,0]))
    ax.scatter(com[0], com[1], com[2], c='green', s=150, marker='*', label='CoM')
    
    # Plot CoM projection on ground
    ax.plot([com[0], com[0]], [com[1], com[1]], [0, com[2]], 'g--', alpha=0.5)
    ax.scatter(com[0], com[1], 0, c='green', s=50, marker='x')

    # 7. Highlight Contact Pivots (Yellow large circles)
    _, corrected_contacts = col_model.find_contact_pivots(q_list)
    contact_coords = np.array([cp["pos"] for cp in corrected_contacts])
    if len(contact_coords) > 0:
        ax.scatter(contact_coords[:,0], contact_coords[:,1], contact_coords[:,2], 
                   c='yellow', edgecolors='black', s=200, zorder=5, label='Contact Pivots (Z=0)')
        
        # Connect pivots to form the Support Polygon Line/Area
        if len(contact_coords) >= 2:
            # Just draw lines between them for visual aid (convex hull in 2D is better but this works for now)
            # Simple line for 2 points, closed loop for >2
            for i in range(len(contact_coords)):
                nxt = (i + 1) % len(contact_coords)
                ax.plot([contact_coords[i,0], contact_coords[nxt,0]], 
                        [contact_coords[i,1], contact_coords[nxt,1]], 
                        [contact_coords[i,2], contact_coords[nxt,2]], 'y-', lw=3, zorder=4)

    # 8. Ground Plane
    xx, yy = np.meshgrid(np.linspace(-0.6, 0.6, 2), np.linspace(-0.6, 0.6, 2))
    zz = np.zeros_like(xx)
    ax.plot_surface(xx, yy, zz, color='gray', alpha=0.2)

    # Styling
    ax.set_title(title, pad=20)
    ax.set_xlabel('X (World)')
    ax.set_ylabel('Y (World)')
    ax.set_zlabel('Z (World)')
    ax.set_xlim([-0.5, 0.5])
    ax.set_ylim([-0.5, 0.5])
    ax.set_zlim([0, 0.6])
    ax.set_box_aspect([1, 1, 0.6])
    ax.legend(loc='upper right', fontsize='small')

if __name__ == "__main__":
    q_stand = [np.deg2rad(60), np.deg2rad(90), np.deg2rad(0)]
    q_list_stand = [q_stand, q_stand, q_stand, q_stand]

    q_push = [np.deg2rad(60), np.deg2rad(90), np.deg2rad(30)]
    q_list_push = [q_push, q_stand, q_stand, q_push] # Left legs pushing

    # Create figure with 2 subplots
    fig = plt.figure(figsize=(16, 8))
    
    # Scenario A: Side Fall
    ax1 = fig.add_subplot(121, projection='3d')
    plot_scenario(ax1, "Scenario A: Side-Fall (Passive)", 90, 0, q_list_stand)
    
    # Scenario C: Right side down, Right legs pushing out
    ax2 = fig.add_subplot(122, projection='3d')
    # If Roll=90 (Left side points up), the right side (-Y) is near ground.
    # We use Right legs (index 1 and 2) and try to push ground by swinging Gamma out.
    q_push = [np.deg2rad(60), np.deg2rad(90), np.deg2rad(45)] # Gamma 45 deg
    q_list_push = [q_stand, q_push, q_push, q_stand]
    
    plot_scenario(ax2, "Scenario B: Side-Fall (Right Legs Pushing)", 90, 0, q_list_push)

    plt.tight_layout()
    
    # Ensure output directory exists
    os.makedirs("output", exist_ok=True)
    save_path = "output/collision_scenarios.png"
    plt.savefig(save_path, dpi=300)
    print(f"Plot saved to: LegWheel/{save_path}")
    
    DISPLAY_SECONDS = 30
    plt.show(block=False)
    plt.pause(DISPLAY_SECONDS)
    plt.close('all')
