import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from legwheel.models import leg_model as LegModel
from legwheel.visualization import plot_leg as PlotLeg
    
    
plot_leg = PlotLeg.PlotLeg()  # rad
# ax = plot_leg.plot_by_angle(np.deg2rad(17), np.deg2rad(0))
# plot_leg.setting(mark_size=10, line_width=3, color='red')
# ax.grid()

def plot_point(ax, point, color='red'):
    point = ax.plot(point[0], point[1], marker='o', color=color, markersize=5, zorder=plot_leg.leg_shape.zorder+0.00001)[0]

fig, ax = plt.subplots()
# ax = plot_leg.plot_by_angle(np.deg2rad(17), np.deg2rad(0))


# # plot leg with different theta
# def update(frame):
#     ax.clear()
#     plot_leg.plot_by_angle(np.deg2rad(frame), np.deg2rad(0), ax=ax)
#     plot_point(ax, plot_leg.rim_point(10), color='red')
#     # plot_leg.setting(mark_size=10, line_width=3, color='red')
#     ax.grid()
#     ax.set_title(f'Theta: {frame}°')
# ani = FuncAnimation(fig, update, frames=range(17, 161, 5), interval=100)
# ani.save(filename='./Output_datas/leg_extend.mp4', fps=10, writer='ffmpeg')

# plot rim point at leg
def update(frame, theta=100):
    ax.clear()
    plot_leg.plot_by_angle(np.deg2rad(theta), np.deg2rad(0), ax=ax)
    # plot_leg.setting(mark_size=10, line_width=3, color='red')
    plot_point(ax, plot_leg.rim_point(frame), color='blue')
    ax.grid()
    ax.set_title(f'Theta: {theta}°, Alpha: {frame}°')
ani = FuncAnimation(fig, update, frames=range(-180, 180, 3), interval=1)
ani.save(filename='./Output_datas/leg_rim_point.mp4', fps=10, writer='ffmpeg')

plt.show()