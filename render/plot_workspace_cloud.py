"""
render/plot_workspace_cloud.py

Visualises the reachable workspace of one or all Corgi legs as a 3D scatter
point-cloud (sphere-marker style).  Each point represents a foot/contact
position computed by forward kinematics for a sampled (θ, β, γ) combination.

Usage:
    # Single leg (FL), default resolution, isometric view
    uv run python render/plot_workspace_cloud.py

    # All 4 legs, higher resolution, save PNG
    uv run python render/plot_workspace_cloud.py --all-legs --res 30 --save-dir output --no-show

    # Wheel mode only (θ ≈ 17°)
    uv run python render/plot_workspace_cloud.py --theta-min 15 --theta-max 20

    # Show chassis wireframe as reference
    uv run python render/plot_workspace_cloud.py --all-legs --chassis
"""

import argparse
import os
import sys

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from legwheel.config import RobotParams
from legwheel.models.corgi_leg import CorgiLegKinematics
from render.plot_corgi_robot import draw_chassis

# ── Joint limits ──────────────────────────────────────────────────────────────
THETA_MIN = RobotParams.MIN_THETA_DEG   # 17°
THETA_MAX = RobotParams.MAX_THETA_DEG   # 160°
BETA_MAX  = 40.0                         # ±40°
GAMMA_MAX = RobotParams.GAMMA_MAX_DEG   # ±30°

LEG_COLORS = ["#E63946", "#457B9D", "#2A9D8F", "#E9C46A"]  # FL FR RR RL
LEG_LABELS = ["FL", "FR", "RR", "RL"]


# ── Sample workspace for one leg ──────────────────────────────────────────────

def sample_workspace(leg_index: int,
                     theta_min: float, theta_max: float,
                     beta_min: float, beta_max: float,
                     gamma_min: float, gamma_max: float,
                     n_theta: int = 20, n_beta: int = 15, n_gamma: int = 7,
                     alpha: float = 0.0) -> np.ndarray:
    """
    Sweep (θ, β, γ) and collect foot positions in Body Frame {B}.

    Returns:
        pts: (N, 3) array of foot positions.
    """
    kin = CorgiLegKinematics(leg_index)
    thetas = np.linspace(np.deg2rad(theta_min), np.deg2rad(theta_max), n_theta)
    betas  = np.linspace(np.deg2rad(beta_min),  np.deg2rad(beta_max),  n_beta)
    gammas = np.linspace(np.deg2rad(gamma_min), np.deg2rad(gamma_max), n_gamma)

    pts = []
    for t in thetas:
        for b in betas:
            for g in gammas:
                try:
                    p = kin.forward_kinematics(t, b, g, alpha=alpha, w=0.0)
                    pts.append(p)
                except Exception:
                    pass
    return np.array(pts)  # shape (N, 3)


# ── Plot ──────────────────────────────────────────────────────────────────────

def plot_workspace_cloud(leg_indices=None,
                         theta_min: float = THETA_MIN,
                         theta_max: float = THETA_MAX,
                         beta_max: float = BETA_MAX,
                         gamma_max: float = GAMMA_MAX,
                         n_theta: int = 20,
                         n_beta: int = 15,
                         n_gamma: int = 7,
                         show_chassis: bool = True,
                         marker_size: float = 4.0,
                         alpha_pts: float = 0.25,
                         save_dir: str = "output",
                         show: bool = True,
                         timeout: float = 30.0):
    if leg_indices is None:
        leg_indices = [0]

    plt.style.use("default")
    fig = plt.figure(figsize=(13, 10), facecolor="white")
    ax  = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("#f8f8f8")
    ax.xaxis.pane.fill = True; ax.xaxis.pane.set_facecolor("#f0f0f0")
    ax.yaxis.pane.fill = True; ax.yaxis.pane.set_facecolor("#f0f0f0")
    ax.zaxis.pane.fill = True; ax.zaxis.pane.set_facecolor("#f0f0f0")
    ax.grid(True, color="#dddddd", linewidth=0.5)

    total_pts = 0
    for leg_idx in leg_indices:
        print(f"  Sampling leg {LEG_LABELS[leg_idx]} (idx={leg_idx}) ...")
        pts = sample_workspace(
            leg_idx,
            theta_min=theta_min, theta_max=theta_max,
            beta_min=-beta_max, beta_max=beta_max,
            gamma_min=-gamma_max, gamma_max=gamma_max,
            n_theta=n_theta, n_beta=n_beta, n_gamma=n_gamma,
        )
        total_pts += len(pts)
        color = LEG_COLORS[leg_idx % len(LEG_COLORS)]
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
                   c=color, s=marker_size, alpha=alpha_pts,
                   marker="o", label=LEG_LABELS[leg_idx], depthshade=True)

    if show_chassis:
        draw_chassis(ax, linewidth=1.5, color="#222222", alpha=0.7)

    ax.set_xlabel("X (Front) [m]", fontsize=9)
    ax.set_ylabel("Y (Left) [m]",  fontsize=9)
    ax.set_zlabel("Z (Up) [m]",    fontsize=9)
    ax.set_xlim(-0.50, 0.50)
    ax.set_ylim(-0.50, 0.50)
    ax.set_zlim(-0.55, 0.20)
    ax.set_box_aspect([1, 1, 0.75])
    ax.view_init(elev=22, azim=-55)

    legs_str = "+".join([LEG_LABELS[i] for i in leg_indices])
    fig.suptitle(
        f"Corgi — Reachable Workspace (Body Frame {{B}})\n"
        f"Legs: {legs_str}   "
        f"θ∈[{theta_min:.0f}°,{theta_max:.0f}°]  "
        f"β∈[±{beta_max:.0f}°]  γ∈[±{gamma_max:.0f}°]\n"
        f"Total sample points: {total_pts:,}  "
        f"(res {n_theta}×{n_beta}×{n_gamma})",
        fontsize=10, color="#111111",
    )
    ax.legend(fontsize=9, loc="upper right", framealpha=0.9)
    plt.tight_layout()

    os.makedirs(save_dir, exist_ok=True)
    fname = os.path.join(save_dir,
                         f"workspace_cloud_{legs_str}_theta{int(theta_min)}-{int(theta_max)}.png")
    plt.savefig(fname, dpi=180, bbox_inches="tight", facecolor="white")
    print(f"Saved → {fname}")

    if show:
        plt.show(block=False)
        plt.pause(timeout)
    plt.close("all")


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--legs",       nargs="+", type=int, default=[0],
                   help="Leg indices to render (0=FL 1=FR 2=RR 3=RL). Default: 0")
    p.add_argument("--all-legs",   action="store_true",
                   help="Render all 4 legs (overrides --legs)")
    p.add_argument("--theta-min",  type=float, default=THETA_MIN)
    p.add_argument("--theta-max",  type=float, default=THETA_MAX)
    p.add_argument("--beta-max",   type=float, default=BETA_MAX)
    p.add_argument("--gamma-max",  type=float, default=GAMMA_MAX)
    p.add_argument("--res",        type=int, default=20,
                   help="θ resolution (n_theta). β=0.75×res, γ=0.35×res")
    p.add_argument("--no-chassis", action="store_true")
    p.add_argument("--marker-size", type=float, default=4.0)
    p.add_argument("--alpha",      type=float, default=0.25)
    p.add_argument("--save-dir",   default="output")
    p.add_argument("--no-show",    action="store_true")
    p.add_argument("--timeout",    type=float, default=30.0)
    return p.parse_args()


def main():
    args = parse_args()
    leg_indices = list(range(4)) if args.all_legs else args.legs
    n_theta = args.res
    n_beta  = max(5, int(args.res * 0.75))
    n_gamma = max(3, int(args.res * 0.35))
    print(f"Workspace cloud: legs={[LEG_LABELS[i] for i in leg_indices]}, "
          f"resolution θ={n_theta} β={n_beta} γ={n_gamma}")
    plot_workspace_cloud(
        leg_indices=leg_indices,
        theta_min=args.theta_min,
        theta_max=args.theta_max,
        beta_max=args.beta_max,
        gamma_max=args.gamma_max,
        n_theta=n_theta,
        n_beta=n_beta,
        n_gamma=n_gamma,
        show_chassis=not args.no_chassis,
        marker_size=args.marker_size,
        alpha_pts=args.alpha,
        save_dir=args.save_dir,
        show=not args.no_show,
        timeout=args.timeout,
    )


if __name__ == "__main__":
    main()
