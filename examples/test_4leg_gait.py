"""
Full 4-Leg Gait 3D Animation.

Animates all 4 legs of the Corgi robot executing a coordinated gait,
using GaitGenerator3D with twist-based velocity mapping.

Usage:
    /home/starlee/envs/legwheel/bin/python examples/test_4leg_gait.py
"""

from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.planners.gait_generator_3d import GaitGenerator3D
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

LEG_COLORS = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red']
LEG_LABELS = ['FL', 'FR', 'RR', 'RL']


def draw_ground_plane(ax, z_ground, center_xy=(0, 0), extent=0.5):
    cx, cy = center_xy
    xx, yy = np.meshgrid(
        np.linspace(cx - extent, cx + extent, 2),
        np.linspace(cy - extent, cy + extent, 2)
    )
    zz = np.full_like(xx, z_ground)
    ax.plot_surface(xx, yy, zz, alpha=0.1, color='sienna', zorder=0)


def draw_body_frame(ax, legs):
    """Draw a simple body rectangle connecting the 4 hip points."""
    hips = np.array([leg.p_Mi_in_B for leg in legs])
    # FL→FR→RR→RL→FL loop
    order = [0, 1, 2, 3, 0]
    body_pts = hips[order]
    ax.plot(body_pts[:, 0], body_pts[:, 1], body_pts[:, 2],
            'k-', linewidth=2, alpha=0.6)


def test_4leg_gait():
    print("=========================================")
    print(" 4-Leg Gait Animation — Corgi Robot      ")
    print("=========================================")

    # --- Configuration ---
    TWIST = [0.0, 0.10, 0.05]   # [omega_z, vx, vy]
    GAIT_TYPE = "Walk"
    STAND_HEIGHT = 0.25
    STEP_HEIGHT = 0.04
    PERIOD = 1.0
    DT = 0.005
    N_CYCLES = 2
    DT_ANIM = 0.02

    # --- Generate Gait ---
    gait = GaitGenerator3D(
        stand_height=STAND_HEIGHT,
        twist=TWIST,
        step_height=STEP_HEIGHT,
        period=PERIOD,
        gait_type=GAIT_TYPE,
        dt=DT
    )
    gait.print_summary()

    print("\nGenerating 4-leg gait trajectory ...")
    cmds = gait.generate_full_gait(n_cycles=N_CYCLES)
    print(f"  Generated: {cmds.shape} (frames x joints)")

    n_total = cmds.shape[0]
    legs = [CorgiLegKinematics(i) for i in range(4)]

    # --- Pre-compute foot positions for traces ---
    # For each leg, compute contact point trace
    foot_traces = [[] for _ in range(4)]
    for frame_idx in range(n_total):
        for i in range(4):
            q = cmds[frame_idx, i * 3: i * 3 + 3]
            contact = legs[i].foot_rim_contact_fk(*q)
            p = legs[i].forward_kinematics(*q, alpha=contact[0], w=0.0)
            foot_traces[i].append(p)
    foot_traces = [np.array(ft) for ft in foot_traces]

    # Ground level
    z_ground = min(ft[:, 2].min() for ft in foot_traces)
    print(f"  Ground Z: {z_ground:.4f} m")

    # --- Determine stance/swing phase per frame per leg ---
    one_cycle = int(PERIOD / DT)
    stance_frames = int(one_cycle * gait.stance_duty)

    def is_stance(frame_idx, leg_idx):
        """Check if leg is in stance phase at this frame."""
        shift = int(gait.phase_offsets[leg_idx] * one_cycle)
        local_frame = (frame_idx + shift) % one_cycle
        return local_frame < stance_frames

    # --- Subsample for animation ---
    step = max(1, int(DT_ANIM / DT))
    anim_indices = list(range(0, n_total, step))

    # --- Figure ---
    fig = plt.figure(figsize=(18, 6))
    fig.suptitle(f"4-Leg {GAIT_TYPE} Gait  |  twist=[{TWIST[0]:.2f}, {TWIST[1]:.2f}, {TWIST[2]:.2f}]",
                 fontsize=14, fontweight='bold')

    axes = [
        fig.add_subplot(131, projection='3d'),
        fig.add_subplot(132, projection='3d'),
        fig.add_subplot(133, projection='3d'),
    ]
    titles = ['Isometric View', 'Side View (XZ)', 'Front View (YZ)']
    views = [(25, -50), (0, -90), (0, 0)]
    lim = 0.45

    def update(frame_count):
        idx = anim_indices[frame_count]

        for vi, ax in enumerate(axes):
            ax.clear()
            draw_ground_plane(ax, z_ground, extent=lim)
            draw_body_frame(ax, legs)

            for i in range(4):
                q = cmds[idx, i * 3: i * 3 + 3]
                stance = is_stance(idx, i)

                # Draw mechanism
                legs[i].plot_leg_3d(q[0], q[1], q[2], ax)

                # Draw foot trace (full cycle)
                ax.plot(foot_traces[i][:, 0], foot_traces[i][:, 1], foot_traces[i][:, 2],
                        color=LEG_COLORS[i], linewidth=1, alpha=0.3)

                # Current foot point
                p_foot = foot_traces[i][idx]
                marker = 'o' if stance else '^'
                ax.scatter(p_foot[0], p_foot[1], p_foot[2],
                           color=LEG_COLORS[i], s=60, marker=marker, zorder=10,
                           label=f"{LEG_LABELS[i]} {'ST' if stance else 'SW'}")

            ax.set_box_aspect([1, 1, 1])
            ax.view_init(elev=views[vi][0], azim=views[vi][1])

            # Equalize axes
            z_mid = (z_ground - 0.05 + 0.3) / 2
            fixed_span = 0.9  # Match x/y span (0.45 * 2)
            ax.set_xlim(-fixed_span/2, fixed_span/2)
            ax.set_ylim(-fixed_span/2, fixed_span/2)
            ax.set_zlim(z_mid - fixed_span/2, z_mid + fixed_span/2)

            progress = idx / n_total * 100
            ax.set_title(f"{titles[vi]}  [{idx}/{n_total}] {progress:.0f}%")
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')

            if vi == 0:
                ax.legend(loc='upper right', fontsize=7, ncol=2)

    print(f"Rendering {len(anim_indices)} frames ...")
    ani = FuncAnimation(fig, update, frames=len(anim_indices),
                        interval=DT_ANIM * 1000, repeat=True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    test_4leg_gait()
