import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.animation import FuncAnimation
from legwheel.models import leg_model as LegModel
from legwheel.visualization import plot_leg as PlotLeg

def plot_axis(ax, size):
    kwarg = {'fontsize':24, 'font':'Times New Roman', 'fontstyle':'normal', 'color':'k'}
    arrow_length = size*2 - 0.03
    axis_color = ("#5A5A5A",0.5)      # color, transparency
    ax.arrow(-arrow_length/2, 0, arrow_length, 0,
            head_width=0.01, head_length=0.01,
            fc=axis_color, ec=axis_color, zorder=0)
    y_end_pos = arrow_length*5/6
    ax.arrow(0, -arrow_length/2, 0, y_end_pos,
            head_width=0.01, head_length=0.01,
            fc=axis_color, ec=axis_color, zorder=0)

    ax.text(arrow_length/2, -0.03, 'X', **kwarg)
    ax.text(0.01, y_end_pos-arrow_length/2, 'Y', **kwarg)

# def plot_leg_angles(plot_leg = [PlotLeg.PlotLeg(), PlotLeg.PlotLeg()],
#                     theta = np.deg2rad(100),
#                     beta = np.deg2rad(45),
#                     ax1 = plt.subplot(), ax2 = plt.subplot()):
#     plot_leg[0].plot_by_angle(np.deg2rad(17), np.deg2rad(0), ax = ax1)
#     plot_leg[1].plot_by_angle(theta, beta, ax = ax2)


def plot_alphas(ax, plot_leg=PlotLeg.PlotLeg(), cmap = plt.cm.plasma, **kwargs):
    # Set equal aspect ratio and axis
    ax.set_aspect('equal')
    fig_label_limit = 0.25
    ax.set_ylim(-fig_label_limit, fig_label_limit)
    ax.set_xlim(-fig_label_limit, fig_label_limit)
    ax.set_xlabel('X (m)', fontsize=14)
    ax.set_ylabel('Y (m)', fontsize=14)
    

    alphas_UR = plot_leg.rim_point(alpha_UR)
    alphas_F = plot_leg.rim_point(alpha_F)
    alphas_UL = plot_leg.rim_point(alpha_UL)

    L = len(alpha_UR) + len(alpha_F) + len(alpha_UL)
    c_alpha = 0.2
    # Plot with gradient colors based on alpha angles
    for i in range(len(alpha_UR)-1):
        ax.plot(alphas_UR[i:i+2, 0], alphas_UR[i:i+2, 1], color=(cmap((i+ len(alpha_F) + len(alpha_UL))/L), c_alpha), **kwargs)

    for i in range(len(alpha_F)-1):
        ax.plot(alphas_F[i:i+2, 0], alphas_F[i:i+2, 1], color=(cmap((i + len(alpha_UL))/L), c_alpha), **kwargs)

    for i in range(len(alpha_UL)-1):
        ax.plot(alphas_UL[i:i+2, 0], alphas_UL[i:i+2, 1], color=(cmap((len(alpha_UL))/L), c_alpha), **kwargs)

def plot_point(ax, point, color='red',plot_leg=PlotLeg.PlotLeg(), zorder=None, markersize=None):
    point = ax.plot(point[0], point[1], marker='o', color=color, markersize=markersize if markersize is not None else plot_leg.leg_shape.mark_size, zorder=zorder if zorder is not None else plot_leg.leg_shape.zorder+0.00001)[0]
    return point

# --------global settings--------
alpha_UR = np.linspace(40.1, 179.9, 100)
alpha_F = np.linspace(-40, 40, 100)
alpha_UL = np.linspace(-179.9, -40.1, 100)
alpha_List = np.concatenate((alpha_UL, alpha_F, alpha_UR))
fig_label_limit = 0.25
theta = np.deg2rad(100)
beta = np.deg2rad(45)
kwarg = {'fontsize':30, 'font':'Times New Roman', 'fontstyle':'normal'}
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(20, 8))
# --------------------------------
if __name__ == "__main__":
    matplotlib.rcParams['pdf.fonttype'] = 42
    matplotlib.rcParams['ps.fonttype'] = 42

    plot_leg = [PlotLeg.PlotLeg(), PlotLeg.PlotLeg()]  # rad
    for pl in plot_leg:
        pl.leg_shape.set_variable('Construction', False)
        pl.leg_shape.set_color_type('Black')
        pl.leg_shape.link_alpha = 1
        pl.leg_shape.line_width = 2.2
    # plot_leg_angles(plot_leg= plot_leg, theta=theta, beta=beta, ax1=ax1, ax2=ax2)
    
    def init():
        kwarg = {'fontsize':30, 'font':'Times New Roman', 'fontstyle':'normal'}
        plot_leg[0].plot_by_angle(np.deg2rad(17), np.deg2rad(0), ax = ax1)
        plot_leg[1].plot_by_angle(theta, beta, ax = ax2)
        # plot_alphas(ax1, plot_leg=plot_leg[0], linewidth=5, marker='o', markersize=5)
        # plot_alphas(ax2, plot_leg=plot_leg[1], linewidth=5, marker='o', markersize=5)
        cmap = plt.cm.plasma
        for ax in [ax1, ax2]:
            ax.set_aspect('equal')
            ax.set_ylim(-fig_label_limit, fig_label_limit)
            ax.set_xlim(-fig_label_limit, fig_label_limit)
            ax.set_xlabel('X (m)', fontsize=14)
            ax.set_ylabel('Y (m)', fontsize=14)
        # Add colorbar to show alpha angle mapping
        ax1.axis('off')  # Hide the first subplot axes
        ax2.axis('off')  # Hide the second subplot axes
        plot_axis(ax1, size=fig_label_limit)
        plot_axis(ax2, size=fig_label_limit)
        ax1.text(-fig_label_limit, fig_label_limit*5/6,'(a)', **kwarg)
        ax2.text(-fig_label_limit, fig_label_limit*5/6,'(b)', **kwarg)

        # ax3.axis('off')  # Hide the third subplot axes
        ax1.set_position([0.1, 0.1, 0.35, 0.8])  # Left subplot
        ax2.set_position([0.44, 0.1, 0.35, 0.8])  # Middle subplot
        ax3.set_position([0.8, 0.1, 0.02, 0.8])  # Right subplot (colorbar)

        sm = plt.cm.ScalarMappable(cmap=cmap)
        sm.set_array([np.deg2rad(-181), np.deg2rad(181)])
        cbar = fig.colorbar(sm, cax=ax3, orientation='vertical', fraction=0.02, pad=0.05)
        cbar.set_label('$\\alpha$ (radians)', fontsize=kwarg['fontsize'])
        for t in cbar.ax.get_yticklabels():
            # t.set(**kwarg)
            t.set_font('Times New Roman')
            t.set_fontsize(kwarg['fontsize'])
            t.set_fontname(kwarg['font'])
    
    def update(frame):
        ax1.clear()
        ax2.clear()
        init()
        alpha = alpha_List[int(frame*2%len(alpha_List))]
        cmap = plt.cm.plasma
        color = cmap((frame*2%len(alpha_List))/len(alpha_List))
        plot_point(ax1, plot_leg[0].rim_point(alpha), color=color, zorder=10, markersize=12)
        plot_point(ax2, plot_leg[1].rim_point(alpha), color=color, zorder=10, markersize=12)
        kwarg = {'fontsize':30, 'font':'Times New Roman', 'fontstyle':'normal'}
        fig.suptitle(f'Rim Point Representation $\\alpha$ = {np.deg2rad(alpha):.2f} rad',
                    x=0.5, y=0.95,
                    fontsize=kwarg['fontsize'],
                    fontname=kwarg['font'],
                    fontstyle=kwarg['fontstyle'])
    ani = FuncAnimation(fig, update, frames=int(len(alpha_List)//2), init_func=init, interval=100, repeat=False)
    ani.save(filename='./Output_datas/Alpha_Representation.mp4', fps=24, writer='ffmpeg')
    plt.close()
    # fig.suptitle('Rim Point Representation $\\alpha$', **kwarg)
    # plt.savefig('alpha.pdf', format='pdf', bbox_inches='tight')
    # ax.grid()
    # plt.show()