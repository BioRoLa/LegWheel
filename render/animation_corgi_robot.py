import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys
import os

# Ensure the parent directory is in path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from render.plot_corgi_robot import draw_corgi_robot

def animate_corgi_robot():
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    def update(frame):
        ax.clear()
        
        # Define animation phases
        if frame < 60:
            # Stand up/down
            t_deg = 90 + 50 * np.sin(frame * np.pi / 60)
            theta, beta, gamma = np.deg2rad(t_deg), 0.0, 0.0
            mode = "Theta (Standing)"
        elif frame < 120:
            # Pitch tilt
            b_deg = 30 * np.sin((frame - 60) * np.pi / 30)
            theta, beta, gamma = np.deg2rad(90), np.deg2rad(b_deg), 0.0
            mode = "Beta (Pitch)"
        else:
            # ABAD
            g_deg = 25 * np.sin((frame - 120) * np.pi / 30)
            theta, beta, gamma = np.deg2rad(90), 0.0, np.deg2rad(g_deg)
            mode = "Gamma (ABAD)"

        # Use the centralized drawing function
        draw_corgi_robot(ax, theta, beta, gamma)

        ax.set_title(f'Corgi Animation - {mode}\ntheta={np.rad2deg(theta):.1f}°, beta={np.rad2deg(beta):.1f}°, gamma={np.rad2deg(gamma):.1f}°')

    ani = FuncAnimation(fig, update, frames=180, interval=50)
    plt.show()

if __name__ == "__main__":
    animate_corgi_robot()