"""
Interactive CSV Trajectory Viewer for CorgiRobot.

Reads a 12-column hardware CSV (dt=1ms) and renders the 4-leg robot
with an interactive timeline slider and play/pause button.

CSV column order (HW format):
  [FL_t, FL_b, FR_t, FR_b, RR_t, RR_b, RL_t, RL_b, FL_g, FR_g, RR_g, RL_g]

Usage:
    python examples/csv_viewer.py <path_to_csv>
    python examples/csv_viewer.py outputs/csv/Walk_Vx0.10_Vy0.00_Wz0.00_H0.25_S0.029_P1.0.csv
"""

import sys
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from matplotlib.animation import FuncAnimation

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from legwheel.models.corgi_leg import CorgiLegKinematics

# --- Constants ---
LEG_COLORS = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red']
LEG_LABELS = ['FL', 'FR', 'RR', 'RL']
DT_INPUT = 0.001   # 1 ms
DT_RENDER = 0.010  # 10 ms
DOWNSAMPLE = int(DT_RENDER / DT_INPUT)  # Take every 10th frame


def hw_to_kin(hw_row):
    """
    Convert a single row from HW CSV order to kinematics order.

    HW:  [FL_t, FL_b, FR_t, FR_b, RR_t, RR_b, RL_t, RL_b, FL_g, FR_g, RR_g, RL_g]
    Kin: [[FL_t, FL_b, FL_g], [FR_t, FR_b, FR_g], [RR_t, RR_b, RR_g], [RL_t, RL_b, RL_g]]
    """
    return np.array([
        [hw_row[0], hw_row[1], hw_row[8]],   # FL
        [hw_row[2], hw_row[3], hw_row[9]],   # FR
        [hw_row[4], hw_row[5], hw_row[10]],  # RR
        [hw_row[6], hw_row[7], hw_row[11]],  # RL
    ])


def draw_ground_plane(ax, z_ground, extent=0.5):
    xx, yy = np.meshgrid(
        np.linspace(-extent, extent, 2),
        np.linspace(-extent, extent, 2)
    )
    zz = np.full_like(xx, z_ground)
    ax.plot_surface(xx, yy, zz, alpha=0.1, color='sienna', zorder=0)


def draw_body_frame(ax, legs):
    """Draw a simple body rectangle connecting the 4 hip points."""
    hips = np.array([leg.p_Mi_in_B for leg in legs])
    order = [0, 1, 2, 3, 0]
    body_pts = hips[order]
    ax.plot(body_pts[:, 0], body_pts[:, 1], body_pts[:, 2],
            'k-', linewidth=2, alpha=0.6)


def main():
    parser = argparse.ArgumentParser(
        description="Interactive CSV Trajectory Viewer for CorgiRobot.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Example:\n  python examples/csv_viewer.py outputs/csv/Walk_Vx0.10_Vy0.00_Wz0.00_H0.25_S0.029_P1.0.csv"
    )
    parser.add_argument("csv_file", type=str, help="Path to the 12-column hardware CSV file.")
    args = parser.parse_args()

    csv_path = args.csv_file
    if not os.path.exists(csv_path):
        print(f"Error: File not found: {csv_path}")
        sys.exit(1)

    csv_name = os.path.basename(csv_path).replace('.csv', '')

    print("=========================================")
    print(" CorgiRobot CSV Trajectory Viewer        ")
    print("=========================================")
    print(f"  File: {csv_path}")

    # --- Load and downsample ---
    raw_data = np.loadtxt(csv_path, delimiter=",")
    n_raw = raw_data.shape[0]
    data = raw_data[::DOWNSAMPLE]  # Downsample 1ms → 10ms
    n_frames = data.shape[0]

    print(f"  Raw frames:  {n_raw} ({n_raw * DT_INPUT:.1f}s @ {1/DT_INPUT:.0f}Hz)")
    print(f"  Render frames: {n_frames} ({n_frames * DT_RENDER:.1f}s @ {1/DT_RENDER:.0f}Hz)")

    # --- Kinematics ---
    legs = [CorgiLegKinematics(i) for i in range(4)]

    # --- Pre-compute foot traces for all render frames ---
    print("Pre-computing foot traces ...")
    foot_traces = [[] for _ in range(4)]
    for fidx in range(n_frames):
        q_all = hw_to_kin(data[fidx])
        for i in range(4):
            q = q_all[i]
            alpha, w_contact = legs[i].foot_rim_contact_fk(*q)
            p = legs[i].forward_kinematics(*q, alpha=alpha, w=w_contact)
            foot_traces[i].append(p)
    foot_traces = [np.array(ft) for ft in foot_traces]

    z_ground = min(ft[:, 2].min() for ft in foot_traces)
    print(f"  Ground Z: {z_ground:.4f} m")

    # --- Figure layout ---
    fig = plt.figure(figsize=(16, 9))
    fig.suptitle(f"CSV Viewer: {csv_name}", fontsize=13, fontweight='bold')

    # Main 3D view
    ax_iso = fig.add_axes([0.02, 0.18, 0.46, 0.75], projection='3d')
    ax_side = fig.add_axes([0.50, 0.18, 0.24, 0.75], projection='3d')
    ax_front = fig.add_axes([0.75, 0.18, 0.24, 0.75], projection='3d')
    view_axes = [ax_iso, ax_side, ax_front]
    view_names = ['Isometric', 'Side (XZ)', 'Front (YZ)']
    view_angles = [(25, -50), (0, -90), (0, 0)]

    # Slider & Button
    ax_slider = fig.add_axes([0.15, 0.05, 0.60, 0.03])
    ax_btn = fig.add_axes([0.80, 0.04, 0.08, 0.04])

    slider = Slider(ax_slider, 'Frame', 0, n_frames - 1, valinit=0, valstep=1, valfmt='%d')
    btn = Button(ax_btn, '▶ Play')

    # --- State ---
    state = {'playing': False, 'anim': None}

    lim = 0.45

    def render_frame(fidx):
        """Draw a single frame on all 3 views."""
        fidx = int(fidx)
        q_all = hw_to_kin(data[fidx])
        t_sec = fidx * DT_RENDER

        for vi, ax in enumerate(view_axes):
            ax.clear()
            draw_ground_plane(ax, z_ground, extent=lim)
            draw_body_frame(ax, legs)

            for i in range(4):
                q = q_all[i]
                legs[i].plot_leg_3d(q[0], q[1], q[2], ax)

                # Full foot trace (faint)
                ax.plot(foot_traces[i][:, 0], foot_traces[i][:, 1], foot_traces[i][:, 2],
                        color=LEG_COLORS[i], linewidth=1, alpha=0.2)

                # Current foot point
                p_foot = foot_traces[i][fidx]
                ax.scatter(p_foot[0], p_foot[1], p_foot[2],
                           color=LEG_COLORS[i], s=50, marker='o', zorder=10,
                           label=LEG_LABELS[i])

            ax.set_box_aspect([1, 1, 1])
            ax.view_init(elev=view_angles[vi][0], azim=view_angles[vi][1])
            ax.set_axis_off()

            z_mid = (z_ground - 0.05 + 0.3) / 2
            fixed_span = 0.9
            ax.set_xlim(-fixed_span / 2, fixed_span / 2)
            ax.set_ylim(-fixed_span / 2, fixed_span / 2)
            ax.set_zlim(z_mid - fixed_span / 2, z_mid + fixed_span / 2)

            ax.set_title(f"{view_names[vi]}  [F{fidx}/{n_frames}]  t={t_sec:.2f}s")

            if vi == 0:
                ax.legend(loc='upper right', fontsize=7, ncol=2)

    def on_slider_change(val):
        render_frame(int(val))
        fig.canvas.draw_idle()

    def anim_update(frame_num):
        current = int(slider.val)
        next_frame = (current + 1) % n_frames
        slider.set_val(next_frame)

    def on_play_pause(event):
        if state['playing']:
            # Pause
            if state['anim'] is not None:
                state['anim'].pause()
            btn.label.set_text('▶ Play')
            state['playing'] = False
        else:
            # Play
            if state['anim'] is None:
                state['anim'] = FuncAnimation(fig, anim_update, frames=n_frames,
                                              interval=DT_RENDER * 1000, repeat=True)
            else:
                state['anim'].resume()
            btn.label.set_text('⏸ Pause')
            state['playing'] = True
        fig.canvas.draw_idle()

    slider.on_changed(on_slider_change)
    btn.on_clicked(on_play_pause)

    # Initial render
    render_frame(0)
    print("Ready. Use the slider or click Play.")
    plt.show()


if __name__ == "__main__":
    main()
