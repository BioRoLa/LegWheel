import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
import sys
import os

# Ensure the parent directory is in path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from render.plot_corgi_robot import draw_corgi_robot
from legwheel.config import OUTPUT_VIDEO_DIR

def animate_multi_view(save_mp4=False):
    # Switch to non-interactive backend if saving to avoid pop-ups
    if save_mp4:
        matplotlib.use('Agg')

    fig = plt.figure(figsize=(18, 6))
    
    # Define subplots
    ax_front = fig.add_subplot(131, projection='3d')
    ax_side = fig.add_subplot(132, projection='3d')
    ax_ortho = fig.add_subplot(133, projection='3d')
    
    axes = [ax_front, ax_side, ax_ortho]
    titles = ["Front View (Y-Z)", "Side View (X-Z)", "Orthogonal View"]

    def update(frame):
        # Define animation phases
        if frame < 60:
            t_deg = 90 + 50 * np.sin(frame * np.pi / 60)
            theta, beta, gamma = np.deg2rad(t_deg), 0.0, 0.0
            mode = "Theta (Standing)"
        elif frame < 120:
            b_deg = 30 * np.sin((frame - 60) * np.pi / 30)
            theta, beta, gamma = np.deg2rad(90), np.deg2rad(b_deg), 0.0
            mode = "Beta (Pitch)"
        else:
            g_deg = 25 * np.sin((frame - 120) * np.pi / 30)
            theta, beta, gamma = np.deg2rad(90), 0.0, np.deg2rad(g_deg)
            mode = "Gamma (ABAD)"

        for i, ax in enumerate(axes):
            ax.clear()
            # Draw without axes/grid
            draw_corgi_robot(ax, theta, beta, gamma, show_axes=False)
            ax.set_title(titles[i])
            
            # Re-apply view init
            if i == 0: ax.view_init(elev=0, azim=0)
            elif i == 1: ax.view_init(elev=0, azim=-90)
            elif i == 2: ax.view_init(elev=20, azim=45)

        fig.suptitle(f'Corgi Multi-View Animation - {mode}\ntheta={np.rad2deg(theta):.1f}°, beta={np.rad2deg(beta):.1f}°, gamma={np.rad2deg(gamma):.1f}°', fontsize=16)

    ani = FuncAnimation(fig, update, frames=180, interval=50)
    plt.tight_layout()
    
    if save_mp4:
        output_path = os.path.join(OUTPUT_VIDEO_DIR, "corgi_multi_view.mp4")
        print(f"Saving animation to {output_path}...")
        writer = FFMpegWriter(fps=20, metadata=dict(artist='Gemini CLI'), bitrate=1800)
        ani.save(output_path, writer=writer)
        plt.close(fig)
        print("Done.")
    else:
        plt.show()

if __name__ == "__main__":
    # Change to True to save as MP4 without showing plot
    animate_multi_view(save_mp4=False)
