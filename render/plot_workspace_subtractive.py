"""
render/plot_workspace_subtractive.py

Subtractive workspace analysis (non-point-cloud view)
------------------------------------------------------
1) Use FK reference point with alpha=0, w=0 as the RRP endpoint.
2) Sweep (gamma, beta, theta) to get the ideal single-leg workspace.
3) Remove parameter samples that collide with the chassis.
4) Mirror the single-leg valid workspace to all four legs via symmetry.

Visualization uses voxel occupancy (volume blocks), not scatter clouds.

Usage
-----
    # Single-leg subtractive volume + mirrored 4-leg volume
    .venv/bin/python render/plot_workspace_subtractive.py --res 26 --save-dir output --no-show

    # Narrower gamma range
    .venv/bin/python render/plot_workspace_subtractive.py --gamma-max 60 --res 28
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.ndimage import gaussian_filter
from skimage.measure import marching_cubes

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from legwheel.config import RobotParams
from legwheel.models.corgi_leg import CorgiLegKinematics
from render.plot_corgi_robot import draw_chassis


THETA_MIN = RobotParams.MIN_THETA_DEG
THETA_MAX = RobotParams.MAX_THETA_DEG
BETA_MAX = 180.0
GAMMA_MAX = 90.0

# Chassis geometry in body frame {B}
_CL = RobotParams.CHASSIS_LENGTH
_CW = RobotParams.CHASSIS_WIDTH
_CH = RobotParams.CHASSIS_HEIGHT
_CZ = RobotParams.ABAD_AXIS_OFFSET
_CC = 0.04

_CZ_LO = _CZ - _CH / 2
_CZ_HI = _CZ + _CH / 2
_CY_HI = _CW / 2
_CY_LO = -_CW / 2

_OCTAGON_HALFSPACES = np.array([
    [1, 0, _CY_HI],
    [-1, 0, -_CY_LO],
    [0, 1, _CZ_HI],
    [0, -1, -_CZ_LO],
    [1, 1, _CY_HI + _CZ_HI - _CC],
    [-1, 1, -_CY_LO + _CZ_HI - _CC],
    [1, -1, _CY_HI - _CZ_LO - _CC],
    [-1, -1, -_CY_LO - _CZ_LO - _CC],
])


def _inside_chassis(pts: np.ndarray, margin: float = 0.003) -> bool:
    if pts.shape[0] == 0:
        return False

    x_lo = -_CL / 2 - margin
    x_hi = _CL / 2 + margin
    mask_x = (pts[:, 0] > x_lo) & (pts[:, 0] < x_hi)
    if not mask_x.any():
        return False

    cands = pts[mask_x]
    y = cands[:, 1:2]
    z = cands[:, 2:3]
    hs = _OCTAGON_HALFSPACES
    vals = y * hs[:, 0] + z * hs[:, 1]
    inside = np.all(vals <= hs[:, 2] + margin, axis=1)
    return bool(inside.any())


def _reference_fk(leg: CorgiLegKinematics, theta: float, beta: float, gamma: float):
    """Reference endpoint using alpha=0, w=0."""
    try:
        return leg.forward_kinematics(theta, beta, gamma, alpha=0.0, w=0.0)
    except Exception:
        return None


def _module_points_for_collision(leg: CorgiLegKinematics, theta: float, beta: float, gamma: float) -> np.ndarray:
    """Module approximation for torso interference: wheel rim + linkage joints."""
    alphas = np.linspace(-180, 180, 18, endpoint=False)
    half_w = leg.wheel_thickness / 2.0
    pts = []

    for a in alphas:
        for w in (half_w, -half_w):
            try:
                pts.append(leg.forward_kinematics(theta, beta, gamma, alpha=float(a), w=w))
            except Exception:
                pass

    try:
        jd = leg.get_joint_positions(theta, beta, gamma)
        pts.extend(list(jd.values()))
    except Exception:
        pass

    if not pts:
        return np.empty((0, 3))
    return np.array(pts)


def sample_subtractive_single_leg(theta_min: float,
                                  theta_max: float,
                                  beta_max: float,
                                  gamma_max: float,
                                  n_theta: int,
                                  n_beta: int,
                                  n_gamma: int,
                                  collision_mode: str = "module"):
    """
    Returns
    -------
    ideal_pts:   all valid FK reference points (no collision filtering)
    kept_pts:    ideal points after removing torso-colliding parameter samples
    removed_pts: points removed by collision subtraction
    stats:       summary dict
    """
    leg = CorgiLegKinematics(0)  # FL as symmetry prototype

    thetas = np.linspace(np.deg2rad(theta_min), np.deg2rad(theta_max), n_theta)
    betas = np.linspace(np.deg2rad(-beta_max), np.deg2rad(beta_max), n_beta)
    gammas = np.linspace(np.deg2rad(-gamma_max), np.deg2rad(gamma_max), n_gamma)

    ideal_pts = []
    kept_pts = []
    removed_pts = []

    total = n_theta * n_beta * n_gamma
    done = 0

    print(f"Sampling FL parameter space: {total:,} samples")
    for t in thetas:
        for b in betas:
            for g in gammas:
                done += 1

                ref = _reference_fk(leg, t, b, g)
                if ref is None:
                    continue

                ideal_pts.append(ref)

                if collision_mode == "reference":
                    collides = _inside_chassis(np.array([ref]))
                else:
                    module_pts = _module_points_for_collision(leg, t, b, g)
                    collides = _inside_chassis(module_pts)

                if collides:
                    removed_pts.append(ref)
                else:
                    kept_pts.append(ref)

        pct = int(100 * done / total)
        print(
            f"\r  progress {pct:3d}%  ideal={len(ideal_pts)}  kept={len(kept_pts)}  removed={len(removed_pts)}",
            end="",
            flush=True,
        )
    print()

    ideal_pts = np.array(ideal_pts) if ideal_pts else np.empty((0, 3))
    kept_pts = np.array(kept_pts) if kept_pts else np.empty((0, 3))
    removed_pts = np.array(removed_pts) if removed_pts else np.empty((0, 3))

    stats = {
        "total_samples": total,
        "ideal": int(len(ideal_pts)),
        "kept": int(len(kept_pts)),
        "removed": int(len(removed_pts)),
    }
    if stats["ideal"] > 0:
        stats["removed_ratio"] = stats["removed"] / stats["ideal"]
        stats["kept_ratio"] = stats["kept"] / stats["ideal"]
    else:
        stats["removed_ratio"] = 0.0
        stats["kept_ratio"] = 0.0

    return ideal_pts, kept_pts, removed_pts, stats


def mirror_from_fl(points_fl: np.ndarray, target_leg: int) -> np.ndarray:
    """
    Mirror FL points to target leg by body symmetry.
    target_leg: 0=FL, 1=FR, 2=RR, 3=RL
    """
    if points_fl.shape[0] == 0:
        return np.empty((0, 3))

    pts = points_fl.copy()
    # FL -> FR: mirror Y
    if target_leg in (1, 2):
        pts[:, 1] *= -1.0
    # FL -> RL: mirror X
    if target_leg in (2, 3):
        pts[:, 0] *= -1.0
    return pts


def _draw_smooth_shell(ax, points: np.ndarray, voxel: float,
                       color, alpha: float, sigma: float = 1.2):
    """
    Render a smooth isosurface (marching cubes) from a point cloud.
    Workflow:
      1. Rasterise points into occupancy grid.
      2. Gaussian-blur the binary field to get a density volume.
      3. Extract isosurface at level=0.25 (between 0 and 1 after blur).
      4. Render with Poly3DCollection.
    """
    if points.shape[0] == 0:
        return

    pad = voxel * 4
    pmin = points.min(axis=0) - pad
    pmax = points.max(axis=0) + pad

    nx = int(np.ceil((pmax[0] - pmin[0]) / voxel)) + 1
    ny = int(np.ceil((pmax[1] - pmin[1]) / voxel)) + 1
    nz = int(np.ceil((pmax[2] - pmin[2]) / voxel)) + 1

    occ = np.zeros((nx, ny, nz), dtype=np.float32)
    idx = np.floor((points - pmin) / voxel).astype(int)
    idx = np.clip(idx, 0, np.array([nx - 1, ny - 1, nz - 1]))
    occ[idx[:, 0], idx[:, 1], idx[:, 2]] = 1.0

    # Gaussian blur → smooth density field
    density = gaussian_filter(occ, sigma=sigma)

    # Marching cubes at 25% of peak density
    level = density.max() * 0.25
    if density.max() <= 0 or level <= 0:
        return

    verts, faces, _, _ = marching_cubes(density, level=level)

    # Convert voxel indices back to world coordinates
    verts_world = verts * voxel + pmin

    mesh = Poly3DCollection(
        verts_world[faces],
        facecolor=color,
        edgecolor="none",
        alpha=alpha,
    )
    ax.add_collection3d(mesh)


def plot_subtractive_workspace(theta_min: float,
                               theta_max: float,
                               beta_max: float,
                               gamma_max: float,
                               n_theta: int,
                               n_beta: int,
                               n_gamma: int,
                               collision_mode: str,
                               voxel_size: float,
                               save_dir: str,
                               show: bool,
                               timeout: float):
    ideal, kept, removed, stats = sample_subtractive_single_leg(
        theta_min=theta_min,
        theta_max=theta_max,
        beta_max=beta_max,
        gamma_max=gamma_max,
        n_theta=n_theta,
        n_beta=n_beta,
        n_gamma=n_gamma,
        collision_mode=collision_mode,
    )

    # Mirror kept workspace from FL to all legs by symmetry
    kept_all = np.vstack([
        mirror_from_fl(kept, 0),
        mirror_from_fl(kept, 1),
        mirror_from_fl(kept, 2),
        mirror_from_fl(kept, 3),
    ]) if kept.shape[0] else np.empty((0, 3))

    plt.style.use("default")
    fig = plt.figure(figsize=(16, 8), facecolor="white")

    # Left: single leg ideal minus removed (subtractive view)
    ax1 = fig.add_subplot(121, projection="3d")
    ax1.set_facecolor("#f8f8f8")
    # Draw ideal volume as a thin translucent outer shell, then overlay collision
    # region in red and valid region in green on top.
    _draw_smooth_shell(ax1, ideal,   voxel_size, "#A8DADC", 0.08)  # ideal outer shell
    _draw_smooth_shell(ax1, removed, voxel_size, "#E63946", 0.45)  # torso-collision subtracted
    _draw_smooth_shell(ax1, kept,    voxel_size, "#2A9D8F", 0.65)  # valid single-leg
    draw_chassis(ax1, linewidth=1.4, color="#222222", alpha=0.80)
    ax1.set_title("Single leg (FL): ideal  −  torso collision", fontsize=10)

    # Right: mirrored 4-leg feasible workspace
    ax2 = fig.add_subplot(122, projection="3d")
    ax2.set_facecolor("#f8f8f8")
    _draw_smooth_shell(ax2, kept_all, voxel_size, "#2A9D8F", 0.55)
    draw_chassis(ax2, linewidth=1.4, color="#222222", alpha=0.80)
    ax2.set_title("Mirrored 4-leg feasible workspace", fontsize=10)

    for ax in (ax1, ax2):
        ax.set_xlabel("X (Front) [m]", fontsize=9)
        ax.set_ylabel("Y (Left) [m]", fontsize=9)
        ax.set_zlabel("Z (Up) [m]", fontsize=9)
        ax.set_xlim(-0.60, 0.60)
        ax.set_ylim(-0.60, 0.60)
        ax.set_zlim(-0.65, 0.30)
        ax.set_box_aspect([1, 1, 0.78])
        ax.view_init(elev=22, azim=-55)
        ax.grid(True, color="#dddddd", linewidth=0.4)

    fig.suptitle(
        "Subtractive Workspace (RRP reference: alpha=0, w=0)\n"
        f"theta=[{theta_min:.0f},{theta_max:.0f}] deg, beta=[±{beta_max:.0f}] deg, gamma=[±{gamma_max:.0f}] deg, "
        f"res={n_theta}x{n_beta}x{n_gamma}, collision={collision_mode}\n"
        f"FL ideal={stats['ideal']:,}, removed={stats['removed']:,} ({stats['removed_ratio']:.1%}), "
        f"kept={stats['kept']:,} ({stats['kept_ratio']:.1%})",
        fontsize=10,
        color="#111111",
    )
    plt.tight_layout()

    os.makedirs(save_dir, exist_ok=True)
    fname = os.path.join(save_dir, "workspace_subtractive_fl_and_mirrored.png")
    plt.savefig(fname, dpi=180, bbox_inches="tight", facecolor="white")
    print(f"Saved -> {fname}")

    if show:
        plt.show(block=False)
        plt.pause(timeout)
    plt.close("all")



def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--theta-min", type=float, default=THETA_MIN)
    p.add_argument("--theta-max", type=float, default=THETA_MAX)
    p.add_argument("--beta-max", type=float, default=BETA_MAX)
    p.add_argument("--gamma-max", type=float, default=GAMMA_MAX)
    p.add_argument("--res", type=int, default=24,
                   help="theta resolution; beta=0.75*res, gamma=0.4*res")
    p.add_argument("--collision-mode", choices=["module", "reference"], default="module",
                   help="module: wheel+linkage vs chassis; reference: only alpha=0,w=0 point")
    p.add_argument("--voxel-size", type=float, default=0.025,
                   help="voxel edge length in meters")
    p.add_argument("--sigma", type=float, default=1.2,
                   help="Gaussian blur sigma for density field smoothing")
    p.add_argument("--save-dir", default="output")
    p.add_argument("--no-show", action="store_true")
    p.add_argument("--timeout", type=float, default=30.0)
    return p.parse_args()



def main():
    args = parse_args()
    n_theta = args.res
    n_beta = max(5, int(args.res * 0.75))
    n_gamma = max(3, int(args.res * 0.40))

    print(
        f"Subtractive workspace: theta={n_theta}, beta={n_beta}, gamma={n_gamma}, "
        f"collision={args.collision_mode}, voxel={args.voxel_size}"
    )

    plot_subtractive_workspace(
        theta_min=args.theta_min,
        theta_max=args.theta_max,
        beta_max=args.beta_max,
        gamma_max=args.gamma_max,
        n_theta=n_theta,
        n_beta=n_beta,
        n_gamma=n_gamma,
        collision_mode=args.collision_mode,
        voxel_size=args.voxel_size,
        save_dir=args.save_dir,
        show=not args.no_show,
        timeout=args.timeout,
    )

# sigma is not wired through the function yet; the default 1.2 covers most use cases.


if __name__ == "__main__":
    main()
