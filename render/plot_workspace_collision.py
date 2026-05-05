"""
render/plot_workspace_collision.py

Reachable workspace of the Corgi legs with collision-filtered colouring.

Sweep strategy
--------------
For each (θ, β, γ) sample of target leg i (all 4 legs assumed at the same pose):
  1. Foot position  = FK(θ, β, γ)
  2. Rim point cloud of leg i (N_rim points around the wheel, both ±w faces)
  3. Linkage joint positions  (O, A, B, C, D, E, F, G)
  4. Torso hit  → any rim / joint point inside the chassis bounding box
  5. Leg-leg hit → min pairwise distance between leg i rim cloud and any other
                   leg j rim cloud (same pose) < WHEEL_THICKNESS

Colour coding
-------------
  GREEN  (#2A9D8F) – valid (no collision)
  RED    (#E63946) – torso collision (leg ↔ chassis)
  ORANGE (#F4A261) – leg-leg overlap (rim-to-rim distance < wheel thickness)
     (if both, torso takes priority)

Usage
-----
    # Single leg FL (default), all theta
    uv run python render/plot_workspace_collision.py

    # All 4 legs, higher resolution
    uv run python render/plot_workspace_collision.py --all-legs --res 25 --no-show

    # Foot mode only (θ ≈ 60-90°)
    uv run python render/plot_workspace_collision.py --theta-min 60 --theta-max 90 --res 30
"""
import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from legwheel.config import RobotParams
from legwheel.models.corgi_leg import CorgiLegKinematics
from render.plot_corgi_robot import draw_chassis

# ── Constants ─────────────────────────────────────────────────────────────────
THETA_MIN = RobotParams.MIN_THETA_DEG   # 17°
THETA_MAX = RobotParams.MAX_THETA_DEG   # 160°
BETA_MAX  = 180.0                        # full 360° sweep (±180°)
GAMMA_MAX = 90.0                         # swept wide; collision box determines real limit

# Chassis AABB (Body Frame {B})
_CL  = RobotParams.CHASSIS_LENGTH   # 0.694
_CW  = RobotParams.CHASSIS_WIDTH    # 0.352
_CH  = RobotParams.CHASSIS_HEIGHT   # 0.138
_CZ  = RobotParams.ABAD_AXIS_OFFSET # 0.0572
_CC  = 0.04                          # chamfer size

# 8 half-space inequalities for the octagonal cross-section (Y-Z plane).
# Each row is [n_y, n_z, b] meaning  n_y*y + n_z*z <= b
_CZ_LO = _CZ - _CH / 2
_CZ_HI = _CZ + _CH / 2
_CY_HI =  _CW / 2
_CY_LO = -_CW / 2
_d     = _CC / np.sqrt(2)  # projected diagonal offset
_OCTAGON_HALFSPACES = np.array([
    [ 1,  0,  _CY_HI],          # y  <=  W/2
    [-1,  0, -_CY_LO],          # y  >= -W/2
    [ 0,  1,  _CZ_HI],          # z  <=  z0+H/2
    [ 0, -1, -_CZ_LO],          # z  >= -(z0-H/2)  (sign: flip so <=)
    [ 1,  1,  _CY_HI + _CZ_HI - _CC],   # +y+z <= corner
    [-1,  1, -_CY_LO + _CZ_HI - _CC],   # -y+z <= corner
    [ 1, -1,  _CY_HI - _CZ_LO - _CC],   # +y-z <= corner
    [-1, -1, -_CY_LO - _CZ_LO - _CC],   # -y-z <= corner
])

LEG_LABELS = ["FL", "FR", "RR", "RL"]
WHEEL_THICK = RobotParams.WHEEL_THICKNESS          # 0.04 m
_R_OUTER    = RobotParams.WHEEL_RADIUS_OUTER        # 0.135 m
# Bounding-sphere fast-reject threshold for leg-leg check
_LEG_LEG_FAST_THRESH = 2 * _R_OUTER * 2.5          # ≈ 0.675 m


# ── Geometry helpers ──────────────────────────────────────────────────────────

def _foot_pos(leg, theta, beta, gamma):
    """FK with fast exception guard; returns None on failure."""
    try:
        return leg.forward_kinematics(theta, beta, gamma, alpha=0.0, w=0.0)
    except Exception:
        return None


def get_rim_cloud(leg: CorgiLegKinematics,
                  theta: float, beta: float, gamma: float,
                  N: int = 18) -> np.ndarray:
    """
    Returns a (≤2*N, 3) array of rim points in Body Frame {B}.
    Samples N evenly-spaced alpha values, at both wheel faces ±w/2.
    """
    alphas = np.linspace(-180, 180, N, endpoint=False)
    half_w = leg.wheel_thickness / 2.0
    pts = []
    for a in alphas:
        for w in (half_w, -half_w):
            try:
                pts.append(leg.forward_kinematics(theta, beta, gamma,
                                                   alpha=float(a), w=w))
            except Exception:
                pass
    return np.array(pts) if pts else np.empty((0, 3))


def get_joint_cloud(leg: CorgiLegKinematics,
                    theta: float, beta: float, gamma: float) -> np.ndarray:
    """Returns the linkage joint positions in {B}."""
    try:
        jd = leg.get_joint_positions(theta, beta, gamma)
        return np.array(list(jd.values()))
    except Exception:
        return np.empty((0, 3))


# ── Collision tests ───────────────────────────────────────────────────────────

def _inside_chassis(pts: np.ndarray, margin: float = 0.003) -> bool:
    """
    Vectorised chassis interior check using AABB (X) + octagonal prism (Y-Z).
    Returns True if ANY point is inside the chassis volume.
    """
    if pts.shape[0] == 0:
        return False
    x_lo = -_CL / 2 - margin
    x_hi =  _CL / 2 + margin
    mask_x = (pts[:, 0] > x_lo) & (pts[:, 0] < x_hi)
    if not mask_x.any():
        return False
    cands = pts[mask_x]  # shape (K, 3)
    # Vectorised octagon test:  (K,8) = (K,1) * (8,)
    y = cands[:, 1:2]  # (K,1)
    z = cands[:, 2:3]  # (K,1)
    hs = _OCTAGON_HALFSPACES          # (8,3): [n_y, n_z, b]
    vals = y * hs[:, 0] + z * hs[:, 1]   # (K,8)
    inside = np.all(vals <= hs[:, 2] + margin, axis=1)   # (K,)
    return bool(inside.any())


def _leg_leg_min_dist(rim_a: np.ndarray, rim_b: np.ndarray) -> float:
    """Min Euclidean distance between two rim clouds (vectorised)."""
    if rim_a.shape[0] == 0 or rim_b.shape[0] == 0:
        return float('inf')
    diff = rim_a[:, None, :] - rim_b[None, :, :]   # (Na, Nb, 3)
    return float(np.sqrt((diff ** 2).sum(axis=2)).min())


# ── Fast per-sample classification ────────────────────────────────────────────

def classify_pose_fast(legs: list,
                       leg_idx: int,
                       theta: float, beta: float, gamma: float) -> tuple:
    """
    Returns (foot_pos, status) where status is 'torso', 'leg_leg', or 'valid'.
    Uses bounding-sphere fast-reject to avoid computing remote legs' rim clouds.
    Returns (None, None) if FK fails for the target leg.
    """
    leg = legs[leg_idx]

    # 1. Target leg geometry
    rim    = get_rim_cloud(leg, theta, beta, gamma)
    joints = get_joint_cloud(leg, theta, beta, gamma)
    if rim.shape[0] == 0:
        return None, None
    foot = rim.mean(axis=0)   # approximate foot centre

    pts = np.vstack([rim, joints]) if joints.shape[0] else rim

    # 2. Torso check (highest priority)
    if _inside_chassis(pts):
        return foot, 'torso'

    # 3. Leg-leg check with fast bounding sphere pre-filter
    for j in range(4):
        if j == leg_idx:
            continue
        # Fast reject: compare foot positions first
        foot_j = _foot_pos(legs[j], theta, beta, gamma)
        if foot_j is None:
            continue
        if np.linalg.norm(foot - foot_j) > _LEG_LEG_FAST_THRESH:
            continue
        # Detailed rim-rim check
        rim_j = get_rim_cloud(legs[j], theta, beta, gamma)
        if _leg_leg_min_dist(rim, rim_j) < WHEEL_THICK:
            return foot, 'leg_leg'

    return foot, 'valid'


# ── Main plot ─────────────────────────────────────────────────────────────────

def plot_workspace_collision(leg_indices=None,
                             theta_min: float = THETA_MIN,
                             theta_max: float = THETA_MAX,
                             beta_max: float  = BETA_MAX,
                             gamma_max: float = GAMMA_MAX,
                             n_theta: int = 20,
                             n_beta:  int = 15,
                             n_gamma: int = 7,
                             show_chassis: bool = True,
                             marker_size: float = 5.0,
                             save_dir: str = "output",
                             show: bool = True,
                             timeout: float = 30.0):

    if leg_indices is None:
        leg_indices = [0]

    legs = [CorgiLegKinematics(i) for i in range(4)]

    thetas = np.linspace(np.deg2rad(theta_min), np.deg2rad(theta_max), n_theta)
    betas  = np.linspace(np.deg2rad(-beta_max),  np.deg2rad(beta_max),  n_beta)
    gammas = np.linspace(np.deg2rad(-gamma_max), np.deg2rad(gamma_max), n_gamma)

    COLOR = {'valid': '#2A9D8F', 'torso': '#E63946', 'leg_leg': '#F4A261'}
    LABEL = {'valid': 'Valid', 'torso': 'Torso collision', 'leg_leg': 'Leg-leg overlap'}
    ALPHA = {'valid': 0.15, 'torso': 0.55, 'leg_leg': 0.55}
    SIZE  = {'valid': marker_size, 'torso': marker_size * 1.5, 'leg_leg': marker_size * 1.5}

    plt.style.use("default")
    fig = plt.figure(figsize=(14, 11), facecolor="white")
    ax  = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("#f8f8f8")
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.fill = True
        pane.set_facecolor("#f0f0f0")
    ax.grid(True, color="#dddddd", linewidth=0.5)

    total = {k: 0 for k in COLOR}
    n_total_per_leg = n_theta * n_beta * n_gamma

    for leg_idx in leg_indices:
        leg_label = LEG_LABELS[leg_idx]
        buckets = {k: [] for k in COLOR}
        n_done = 0
        print(f"  Processing leg {leg_label} — {n_total_per_leg:,} samples ...", flush=True)

        for t in thetas:
            for b in betas:
                for g in gammas:
                    n_done += 1
                    foot, cls = classify_pose_fast(legs, leg_idx, t, b, g)
                    if foot is not None and cls is not None:
                        buckets[cls].append(foot)
            # progress every theta slice
            pct = int(100 * n_done / n_total_per_leg)
            print(f"\r    {leg_label}: {pct:3d}%  "
                  f"valid={len(buckets['valid'])}  "
                  f"torso={len(buckets['torso'])}  "
                  f"leg_leg={len(buckets['leg_leg'])}     ",
                  end='', flush=True)
        print()  # newline

        for cls, pts_list in buckets.items():
            if not pts_list:
                continue
            pts = np.array(pts_list)
            total[cls] += len(pts)
            ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
                       c=COLOR[cls], s=SIZE[cls], alpha=ALPHA[cls],
                       marker='o', depthshade=True)

        print(f"    {leg_label} done: valid={len(buckets['valid'])}  "
              f"torso={len(buckets['torso'])}  "
              f"leg_leg={len(buckets['leg_leg'])}")

    if show_chassis:
        draw_chassis(ax, linewidth=1.8, color='#222222', alpha=0.8)

    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor=COLOR[k], alpha=0.85, label=LABEL[k]) for k in COLOR]
    ax.legend(handles=legend_elements, fontsize=9, loc='upper right', framealpha=0.9)

    ax.set_xlabel('X (Front) [m]', fontsize=9)
    ax.set_ylabel('Y (Left) [m]',  fontsize=9)
    ax.set_zlabel('Z (Up) [m]',    fontsize=9)
    ax.set_xlim(-0.60, 0.60)
    ax.set_ylim(-0.60, 0.60)
    ax.set_zlim(-0.65, 0.30)
    ax.set_box_aspect([1, 1, 0.78])
    ax.view_init(elev=22, azim=-55)

    legs_str = "+".join([LEG_LABELS[i] for i in leg_indices])
    fig.suptitle(
        f"Corgi — Workspace with Collision Filtering  (Body Frame {{B}})\n"
        f"Legs: {legs_str}   "
        f"θ∈[{theta_min:.0f}°,{theta_max:.0f}°]  β∈[±{beta_max:.0f}° = full 360°]  "
        f"γ∈[±{gamma_max:.0f}°, collision-limited]\n"
        f"Green=Valid  Red=Torso collision  Orange=Leg-leg overlap  "
        f"(res {n_theta}×{n_beta}×{n_gamma})\n"
        f"Total — valid:{total['valid']:,}  torso:{total['torso']:,}  leg_leg:{total['leg_leg']:,}",
        fontsize=9.5, color="#111111",
    )
    plt.tight_layout()

    os.makedirs(save_dir, exist_ok=True)
    fname = os.path.join(save_dir,
                         f"workspace_collision_{legs_str}_theta{int(theta_min)}-{int(theta_max)}.png")
    plt.savefig(fname, dpi=180, bbox_inches='tight', facecolor='white')
    print(f"Saved → {fname}")

    if show:
        plt.show(block=False)
        plt.pause(timeout)
    plt.close('all')


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--legs',      nargs='+', type=int, default=[0],
                   help='Leg indices (0=FL 1=FR 2=RR 3=RL)')
    p.add_argument('--all-legs',  action='store_true', help='Render all 4 legs')
    p.add_argument('--theta-min', type=float, default=THETA_MIN)
    p.add_argument('--theta-max', type=float, default=THETA_MAX)
    p.add_argument('--beta-max',  type=float, default=BETA_MAX,
                   help='Half-range for beta sweep (180 = full 360°)')
    p.add_argument('--gamma-max', type=float, default=GAMMA_MAX,
                   help='Half-range for gamma sweep; collision box filters real limit')
    p.add_argument('--res',       type=int, default=20,
                   help='θ resolution; β=0.75×res, γ=0.4×res')
    p.add_argument('--no-chassis', action='store_true')
    p.add_argument('--marker-size', type=float, default=5.0)
    p.add_argument('--save-dir',  default='output')
    p.add_argument('--no-show',   action='store_true')
    p.add_argument('--timeout',   type=float, default=30.0)
    return p.parse_args()


def main():
    args = parse_args()
    leg_indices = list(range(4)) if args.all_legs else args.legs
    n_theta = args.res
    n_beta  = max(5, int(args.res * 0.75))
    n_gamma = max(3, int(args.res * 0.40))
    print(f"Workspace collision: legs={[LEG_LABELS[i] for i in leg_indices]}, "
          f"resolution θ={n_theta} β={n_beta} γ={n_gamma}")
    plot_workspace_collision(
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
        save_dir=args.save_dir,
        show=not args.no_show,
        timeout=args.timeout,
    )


if __name__ == '__main__':
    main()
