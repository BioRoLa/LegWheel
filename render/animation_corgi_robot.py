import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter
import sys
import os

# Ensure the parent directory is in path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from render.plot_corgi_robot import draw_corgi_robot
from legwheel.config import OUTPUT_VIDEO_DIR

def animate_corgi_robot(save_mp4=False):
    if save_mp4:
        matplotlib.use('Agg')
        
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    def update(frame):
        ax.clear()
        # 90 frames total: 0-30 theta, 30-60 beta, 60-90 gamma
        if frame < 30:
            t_deg = 90 + 50 * np.sin(frame * np.pi / 30)
            theta, beta, gamma = np.deg2rad(t_deg), 0.0, 0.0
            mode = "Theta (Standing)"
        elif frame < 60:
            b_deg = 30 * np.sin((frame - 30) * np.pi / 15)
            theta, beta, gamma = np.deg2rad(90), np.deg2rad(b_deg), 0.0
            mode = "Beta (Pitch)"
        else:
            g_deg = 25 * np.sin((frame - 60) * np.pi / 15)
            theta, beta, gamma = np.deg2rad(90), 0.0, np.deg2rad(g_deg)
            mode = "Gamma (ABAD)"

        draw_corgi_robot(ax, theta, beta, gamma, show_axes=False)
        ax.set_title(f'Corgi Animation (Body Frame {{B}}) - {mode}\ntheta={np.rad2deg(theta):.1f}°, beta={np.rad2deg(beta):.1f}°, gamma={np.rad2deg(gamma):.1f}°')

    ani = FuncAnimation(fig, update, frames=90, interval=50)
    
    if save_mp4:
        try:
            output_path = os.path.join(OUTPUT_VIDEO_DIR, "corgi_animation.mp4")
            print(f"Saving animation to {output_path} (using FFMpeg)...")
            writer = FFMpegWriter(fps=20, metadata=dict(artist='Gemini CLI'), bitrate=1800)
            ani.save(output_path, writer=writer)
        except Exception:
            output_path = os.path.join(OUTPUT_VIDEO_DIR, "corgi_animation.gif")
            print(f"FFMpeg failed, saving to {output_path} (using Pillow)...")
            writer = PillowWriter(fps=20)
            ani.save(output_path, writer=writer)
            
        plt.close(fig)
        print("Done.")
    else:
        plt.show()

if __name__ == "__main__":
    animate_corgi_robot(save_mp4=False)