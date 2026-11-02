"""
render/plot_workspace_parametric.py

Parametric (vector-geometry) workspace envelope for the Corgi leg.

Method
------
Treat the leg as RRP: (γ, β, θ) = ABAD-roll, swing, extension.
Reference point: alpha=0, w=0  (wheel-axis centre).

For each (β, γ) grid cell
  outer boundary: scan θ from θ_max downward → first non-torso-colliding θ.
  inner boundary: FK at θ_min (the hollow core of the workspace).

The workspace surface is built as a proper 2D parametric mesh and rendered
with ax.plot_surface — no point cloud, no voxels.

Outer surface colour encodes collision impact:
  Green  → full θ range available  (theta_frac ≈ 1.0)
  Yellow → partially clipped
  Red    → heavily clipped / nearly fully blocked

The FL canonical workspace is mirrored to FR / RR / RL by body symmetry.

Usage
-----
    .venv/bin/python render/plot_workspace_parametric.py --res 60 --save-dir output --no-show
"""
import argparse
import os
import sys

import matplotlib.cm as cm
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from legwheel.config import RobotParams
from legwheel.models.corgi_leg import CorgiLegKinematics
from render.plot_corgi_robot import draw_chassis

# ── Constants ─────────────────────────────────────────────────────────────────
THETA_MIN = RobotParams.MIN_THETA_DEG
THETA_MAX = RobotParams.MAX_THETA_DEG
BETA_MAX  = 180.0
GAMMA_MAX = 90.0

_CL = RobotParams.CHASSIS_LENGTH
_CW = RobotParams.CHASSIS_WIDTH
_CH = RobotParams.CHASSIS_HEIGHT
_CZ = RobotParams.ABAD_AXIS_OFFSET
_CC = 0.04

_CZ_LO = _CZ - _CH / 2
_CZ_HI = _CZ + _CH / 2
_CY_HI =  _CW / 2
_CY_LO = -_CW / 2

_OCTAGON_HALFSPACES = np.array([
    [ 1,  0,  _CY_HI],
    [-1,  0, -_CY_LO],
    [ 0,  1,  _CZ_HI],
    [ 0, -1, -_CZ_LO],
    [ 1,  1,  _CY_HI + _CZ_HI - _CC],
    [-1,  1, -_CY_LO + _CZ_HI - _CC],
    [ 1, -1,  _CY_HI - _CZ_LO - _CC],
    [-1, -1, -_CY_LO - _CZ_LO - _CC],
])


# ── Collision helpers ─────────────────────────────────────────────────────────

def _inside_chassis(pts: np.ndarray, margin: float = 0.003) -> bool:
    if pts.shape[0] == 0:
        return False
    mask_x = (pts[:, 0] > -_CL / 2 - margin) & (pts[:, 0] < _CL / 2 + margin)
    if not mask_x.any():
        return False
    cands = pts[mask_x]
    y, z = cands[:, 1:2], cands[:, 2:3]
    vals = y * _OCTAGON_HALFSPACES[:, 0] + z * _OCTAGON_HALFSPACES[:, 1]
    return bool(np.all(vals <= _OCTAGON_HALFSPACES[:, 2] + margin, axis=1).any())


def _module_pts(leg: CorgiLegKinematics, t: float, b: float, g: float) -> np.ndarray:
    """Wheel rim + linkage joints for collision check."""
    alphas = np.linspace(-180, 180, 18, endpoint=False)
    half_w = leg.wheel_thickness / 2.0
    # Sample w=0 (torus crown, r_eff is maximum) and ±half_w (hard rim face)
    # so the bounding volume covers both the widest and outermost contact points.
    w_samples = (0.0, half_w, -half_w)
    pts = []
    for a in alphas:
        for w in w_samples:
            try:
                pts.append(leg.forward_kinematics(t, b, g, alpha=float(a), w=w))
            except Exception:
                pass
    try:
        pts.extend(list(leg.get_joint_positions(t, b, g).values()))
    except Exception:
        pass
    return np.array(pts) if pts else np.empty((0, 3))


def _ref_fk(leg: CorgiLegKinematics, t: float, b: float, g: float):
    try:
        return leg.forward_kinematics(t, b, g, alpha=0.0, w=0.0)
    except Exception:
        return None


# ── Core computation ──────────────────────────────────────────────────────────

def compute_workspace_surface(
    theta_min: float,
    theta_max: float,
    beta_max: float,
    gamma_max: float,
    n_beta: int,
    n_gamma: int,
    n_theta_scan: int,
    collision_margin: float = 0.003,
):
    """
    Compute the parametric workspace boundary surface for the FL leg.

    Returns
    -------
    outer_pts  : (n_beta+1, n_gamma, 3)  — outer boundary FK mesh (NaN where fully blocked)
    inner_pts  : (n_beta+1, n_gamma, 3)  — inner boundary FK mesh (θ_min)
    theta_frac : (n_beta+1, n_gamma)     — fraction of θ range kept (1=no cut, 0=fully blocked)
    """
    leg = CorgiLegKinematics(0)

    betas  = np.linspace(-np.deg2rad(beta_max),  np.deg2rad(beta_max),  n_beta, endpoint=False)
    gammas = np.linspace(-np.deg2rad(gamma_max), np.deg2rad(gamma_max), n_gamma)
    thetas = np.linspace(np.deg2rad(theta_min),  np.deg2rad(theta_max), n_theta_scan)

    t_min_rad = np.deg2rad(theta_min)
    t_max_rad = np.deg2rad(theta_max)
    t_range   = t_max_rad - t_min_rad

    outer_raw = np.full((n_beta, n_gamma, 3), np.nan)
    inner_raw = np.full((n_beta, n_gamma, 3), np.nan)
    frac_raw  = np.full((n_beta, n_gamma),    np.nan)

    total = n_beta * n_gamma
    done  = 0

    for j, g in enumerate(gammas):
        for i, b in enumerate(betas):
            done += 1

            # ── outer: scan theta from max → first non-colliding ──
            for t in reversed(thetas):
                ref = _ref_fk(leg, t, b, g)
                if ref is None:
                    continue
                if not _inside_chassis(_module_pts(leg, t, b, g), collision_margin):
                    outer_raw[i, j] = ref
                    frac_raw[i, j]  = (t - t_min_rad) / t_range
                    break

            # ── inner: theta_min ──
            ref_inner = _ref_fk(leg, t_min_rad, b, g)
            if ref_inner is not None:
                inner_raw[i, j] = ref_inner

        pct = int(100 * done / total)
        n_valid = int(np.sum(~np.isnan(outer_raw[:, :j+1, 0])))
        print(f"\r  FL surface: {pct:3d}%  valid cells = {n_valid}", end="", flush=True)
    print()

    # Close beta (periodic): append first row so surface wraps cleanly
    outer_pts  = np.concatenate([outer_raw,  outer_raw[[0]]],  axis=0)
    inner_pts  = np.concatenate([inner_raw,  inner_raw[[0]]],  axis=0)
    theta_frac = np.concatenate([frac_raw,   frac_raw[[0]]],   axis=0)

    return outer_pts, inner_pts, theta_frac


# ── Mirror ────────────────────────────────────────────────────────────────────

def mirror_surface(pts: np.ndarray, target_leg: int) -> np.ndarray:
    """
    Mirror an (N, M, 3) surface array from FL canonical to target leg.
    0=FL (identity), 1=FR (flip Y), 2=RR (flip X+Y), 3=RL (flip X).
    """
    out = pts.copy()
    if target_leg in (1, 2):
        out[..., 1] *= -1.0
    if target_leg in (2, 3):
        out[..., 0] *= -1.0
    return out


# ── Drawing helpers ───────────────────────────────────────────────────────────

def _draw_gamma_caps(ax, outer_pts: np.ndarray, inner_pts: np.ndarray,
                     color, alpha: float):
    """
    Connect outer and inner surfaces at each gamma extreme with a quad strip.
    Fills the 'edge ring' so the workspace looks like a closed solid shell.
    """
    n_gamma = outer_pts.shape[1]
    for j in (0, n_gamma - 1):
        o = outer_pts[:, j, :]
        n = inner_pts[:, j, :]
        polys = []
        for i in range(o.shape[0] - 1):
            verts = [o[i], o[i + 1], n[i + 1], n[i]]
            if not any(np.any(np.isnan(v)) for v in verts):
                polys.append(verts)
        if polys:
            coll = Poly3DCollection(polys, facecolor=color, edgecolor="none", alpha=alpha)
            ax.add_collection3d(coll)


def draw_workspace_shell(
    ax,
    outer_pts: np.ndarray,
    inner_pts: np.ndarray,
    theta_frac: np.ndarray,
    alpha_outer: float = 0.75,
    alpha_inner: float = 0.20,
    alpha_caps:  float = 0.25,
    colormap: str = "RdYlGn",
):
    """
    Render the full workspace shell:
      outer surface  — coloured by theta_frac (green=full, red=collision-clipped)
      inner surface  — hollow core, translucent blue-grey
      gamma end caps — connecting outer to inner at gamma extremes
    """
    # ── outer surface ──
    cmap = plt.colormaps[colormap]
    norm = Normalize(vmin=0.0, vmax=1.0)

    frac_fill = np.where(np.isnan(theta_frac), 0.0, theta_frac)
    fc = cmap(norm(frac_fill))                            # (N, M, 4)
    fc[..., 3] = np.where(np.isnan(theta_frac), 0.0, alpha_outer)   # transparent where no valid theta

    X = np.ma.masked_invalid(outer_pts[:, :, 0])
    Y = np.ma.masked_invalid(outer_pts[:, :, 1])
    Z = np.ma.masked_invalid(outer_pts[:, :, 2])

    ax.plot_surface(X, Y, Z, facecolors=fc, shade=True,
                    linewidth=0, antialiased=True, zorder=2)

    # ── inner surface ──
    Xi = np.ma.masked_invalid(inner_pts[:, :, 0])
    Yi = np.ma.masked_invalid(inner_pts[:, :, 1])
    Zi = np.ma.masked_invalid(inner_pts[:, :, 2])

    ax.plot_surface(Xi, Yi, Zi, color="#A8DADC", shade=True, alpha=alpha_inner,
                    linewidth=0, antialiased=True, zorder=1)

    # ── gamma-end caps ──
    _draw_gamma_caps(ax, outer_pts, inner_pts, color="#888888", alpha=alpha_caps)


# ── Colourbar ─────────────────────────────────────────────────────────────────

def _add_colorbar(fig, ax, colormap="RdYlGn"):
    mappable = cm.ScalarMappable(cmap=plt.colormaps[colormap], norm=Normalize(0, 1))
    mappable.set_array([])
    cbar = fig.colorbar(mappable, ax=ax, shrink=0.55, pad=0.08, orientation="vertical")
    cbar.set_label("θ range preserved\n(1=full, 0=fully blocked)", fontsize=8)
    cbar.set_ticks([0, 0.5, 1.0])
    cbar.set_ticklabels(["blocked", "half", "full"])


# ── Axis style ────────────────────────────────────────────────────────────────

def _style_ax(ax, title):
    ax.set_facecolor("#f8f8f8")
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.fill = True
        pane.set_facecolor("#f0f0f0")
    ax.grid(True, color="#dddddd", linewidth=0.4)
    ax.set_xlabel("X (Front) [m]", fontsize=8)
    ax.set_ylabel("Y (Left)  [m]", fontsize=8)
    ax.set_zlabel("Z (Up)    [m]", fontsize=8)
    ax.set_xlim(-0.65, 0.65)
    ax.set_ylim(-0.65, 0.65)
    ax.set_zlim(-0.70, 0.30)
    ax.set_box_aspect([1, 1, 0.75])
    ax.view_init(elev=22, azim=-55)
    ax.set_title(title, fontsize=10, pad=6)


# ── Main ──────────────────────────────────────────────────────────────────────

def plot_parametric_workspace(
    theta_min: float = THETA_MIN,
    theta_max: float = THETA_MAX,
    beta_max:  float = BETA_MAX,
    gamma_max: float = GAMMA_MAX,
    n_beta:    int   = 72,
    n_gamma:   int   = 36,
    n_theta_scan: int = 40,
    save_dir:  str   = "output",
    show:      bool  = True,
    timeout:   float = 30.0,
):
    # 1. Compute FL canonical workspace surface
    outer_fl, inner_fl, frac_fl = compute_workspace_surface(
        theta_min, theta_max, beta_max, gamma_max,
        n_beta, n_gamma, n_theta_scan,
    )

    n_valid  = int(np.sum(~np.isnan(frac_fl)))
    n_cut    = int(np.sum((~np.isnan(frac_fl)) & (frac_fl < 1.0 - 1e-6)))
    n_blocked = int(np.sum(np.isnan(frac_fl)))
    total_cells = (n_beta + 1) * n_gamma
    print(f"\n  FL cells: total={total_cells}  valid={n_valid}  "
          f"cut={n_cut}  blocked={n_blocked}")
    print(f"  Mean theta_frac (valid cells): "
          f"{np.nanmean(frac_fl):.3f}")

    # 2. Figure: left=FL only, right=4-leg mirror
    plt.style.use("default")
    fig = plt.figure(figsize=(16, 8), facecolor="white")

    # ── Left panel: FL single leg ──────────────────────────────────────────────
    ax1 = fig.add_subplot(121, projection="3d")
    draw_workspace_shell(ax1, outer_fl, inner_fl, frac_fl,
                         alpha_outer=0.80, alpha_inner=0.20)
    draw_chassis(ax1, linewidth=1.5, color="#222222", alpha=0.85)
    _style_ax(ax1, "FL leg — outer surface coloured by collision impact\n"
              "(green=full θ-range  red=clipped by torso collision)")
    _add_colorbar(fig, ax1)

    # ── Right panel: 4-leg mirror ──────────────────────────────────────────────
    ax2 = fig.add_subplot(122, projection="3d")
    LEG_NAMES = ["FL", "FR", "RR", "RL"]
    for leg_idx in range(4):
        outer_i = mirror_surface(outer_fl, leg_idx)
        inner_i = mirror_surface(inner_fl, leg_idx)
        draw_workspace_shell(ax2, outer_i, inner_i, frac_fl,
                             alpha_outer=0.45, alpha_inner=0.10, alpha_caps=0.12)
    draw_chassis(ax2, linewidth=1.5, color="#222222", alpha=0.85)
    _style_ax(ax2, "4-leg mirrored feasible workspace\n"
              "(FL canonical + symmetric mirrors)")

    # ── Suptitle ──────────────────────────────────────────────────────────────
    fig.suptitle(
        f"Corgi — Parametric Workspace Envelope  (RRP model, alpha=0, w=0)\n"
        f"θ∈[{theta_min:.0f}°,{theta_max:.0f}°]  β∈[±{beta_max:.0f}°]  "
        f"γ∈[±{gamma_max:.0f}°]   mesh {n_beta}×{n_gamma}, θ-scan={n_theta_scan}\n"
        f"FL: {n_valid}/{total_cells} cells valid, "
        f"{n_cut} partially clipped, {n_blocked} fully blocked",
        fontsize=10, color="#111111", y=1.01,
    )
    plt.tight_layout()

    os.makedirs(save_dir, exist_ok=True)
    fname = os.path.join(save_dir, "workspace_parametric_fl_and_4leg.png")
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
    p.add_argument("--theta-min",    type=float, default=THETA_MIN)
    p.add_argument("--theta-max",    type=float, default=THETA_MAX)
    p.add_argument("--beta-max",     type=float, default=BETA_MAX)
    p.add_argument("--gamma-max",    type=float, default=GAMMA_MAX)
    p.add_argument("--res",          type=int,   default=60,
                   help="beta resolution; gamma=res//2, theta_scan=res*2//3")
    p.add_argument("--save-dir",     default="output")
    p.add_argument("--no-show",      action="store_true")
    p.add_argument("--timeout",      type=float, default=30.0)
    return p.parse_args()


def main():
    args = parse_args()
    n_beta       = args.res
    n_gamma      = max(8, args.res // 2)
    n_theta_scan = max(20, args.res * 2 // 3)

    print(f"Parametric workspace: beta_grid={n_beta}, gamma_grid={n_gamma}, "
          f"theta_scan={n_theta_scan}")
    print(f"  theta=[{args.theta_min:.0f},{args.theta_max:.0f}]  "
          f"beta=[±{args.beta_max:.0f}]  gamma=[±{args.gamma_max:.0f}]")

    plot_parametric_workspace(
        theta_min=args.theta_min,
        theta_max=args.theta_max,
        beta_max=args.beta_max,
        gamma_max=args.gamma_max,
        n_beta=n_beta,
        n_gamma=n_gamma,
        n_theta_scan=n_theta_scan,
        save_dir=args.save_dir,
        show=not args.no_show,
        timeout=args.timeout,
    )


if __name__ == "__main__":
    main()
