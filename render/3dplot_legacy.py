import numpy as np
import matplotlib.pyplot as plt
from legwheel.models import leg_model as LegModel
from legwheel.visualization import plot_leg as PlotLeg
# from mpl_toolkits.mplot3d import Axes3D

T = np.array([np.deg2rad(i) for i in range(17,160)])
B = np.array([i*0 for i in range(17,160)])
leg = LegModel.LegModel()
leg.forward(T,B)
# rim_dict[alpha] returns a list of rim points for the given alpha in cartesian coordinates
rim_dict = {}
# rim_dict_polar[alpha] returns a list of rim points for the given alpha in polar coordinates
rim_dict_polar = {}

for alpha in range(-180,180,1):
    # build rim_dict
    rim_dict[alpha] = leg.rim_point(alpha)
    rim_dict_polar[alpha] = []
    
    # Convert cartesian coordinates to polar coordinates
    for point in rim_dict[alpha]:
        r = np.sqrt(point[0]**2 + point[1]**2)
        theta = np.arctan2(point[1], point[0])
        rim_dict_polar[alpha].append((r, theta))
        
# Split alpha into three groups
alpha_keys = list(rim_dict.keys())
group1 = [a for a in alpha_keys if a < -40]
group2 = [a for a in alpha_keys if -40 <= a <= 40]
group3 = [a for a in alpha_keys if a > 40]

groups = [("Upper Rim L [-180,-40)", group1), ("Foot Rim [-40,40]", group2), ("Upper Rim R (40,180]", group3)]

fig = plt.figure(figsize=(18, 5))
for idx, (title, group) in enumerate(groups, 1):
    T_grid, alpha_grid = np.meshgrid(T, group)
    r_grid = np.zeros_like(T_grid, dtype=float)
    for i, a in enumerate(group):
        for j, t in enumerate(T):
            # Use the first rim point for each alpha and T
            vec = rim_dict_polar[a][j]
            r_grid[i, j] = vec[0] 
    ax3d = fig.add_subplot(1, 3, idx, projection='3d')
    contour = ax3d.contourf3D(T_grid, alpha_grid, r_grid, 50, cmap='viridis')
    ax3d.set_xlabel('T')
    ax3d.set_ylabel('alpha')
    ax3d.set_zlabel('r')
    ax3d.set_title(title)
    fig.colorbar(contour, ax=ax3d, shrink=0.5)

plt.tight_layout()
plt.show()
