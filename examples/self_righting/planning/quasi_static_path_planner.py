"""
examples/self_righting/quasi_static_path_planner.py

Diagnostic quasi-static stability scan (legacy planner):
  - Fixed theta=75, beta=0 (best stable config from cone analysis)
  - Sweep gamma_lower x gamma_upper x roll
  - Generates stability map + optimal path for scoring manual FSM keyframes

Current planning note (2026-05-25): self-righting is manual-keyframe/FSM first.
This script is retained as a diagnostic stability scan, not the primary trajectory planner.
"""
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

from legwheel.models.collision_model import CorgiCollisionModel
from legwheel.models.corgi_robot import CorgiRobot


def stability_margin(roll_deg, q_list, tol=4e-3):
    robot = CorgiRobot()
    robot.base_ori = np.array([np.deg2rad(roll_deg), 0, 0])
    robot.base_pos = np.array([0, 0, 0.5])
    col = CorgiCollisionModel(robot)
    pts = col.get_all_collision_points(q_list)
    c = np.vstack([pts["chassis"], pts["m6_studs"], pts["wheels"]])
    robot.base_pos[2] -= c[:, 2].min()
    pts2 = col.get_all_collision_points(q_list)
    c2 = np.vstack([pts2["chassis"], pts2["m6_studs"], pts2["wheels"]])
    mask = c2[:, 2] <= c2[:, 2].min() + tol
    xy = c2[mask, :2]
    if len(xy) < 3:
        return -1.0
    try:
        hull = ConvexHull(xy)
        com = robot.body_to_world(np.zeros(3))[:2]
        vals = hull.equations @ np.append(com, 1.0)
        d = -vals.max()
        hp = xy[hull.vertices]
        n = len(hp)
        perim = sum(np.linalg.norm(hp[(i + 1) % n] - hp[i]) for i in range(n))
        return float(d / (2 * hull.volume / perim))
    except Exception:
        return -1.0


def make_q_list(roll_deg, theta, beta, gamma_lower, gamma_upper):
    q_l = [np.deg2rad(theta), np.deg2rad(beta), np.deg2rad(gamma_lower)]
    q_u = [np.deg2rad(theta), np.deg2rad(beta), np.deg2rad(gamma_upper)]
    if roll_deg >= 90:
        return [q_u, q_l, q_l, q_u]
    else:
        return [q_l, q_u, q_u, q_l]


if __name__ == "__main__":
    THETA = 75
    BETA = 0
    rolls = np.arange(180, -5, -10)  # every 10°: 19 values
    gammas = np.arange(-30, 35, 10)  # -30,-20,...,30: 7 values
    thetas = [45, 60, 75, 90]  # 4 theta values

    # Build (roll x gamma_lower) stability map
    # gamma_upper = -gamma_lower (symmetric) for each evaluation
    # Total: 19 x 7 x 4 = 532 evaluations
    N_roll = len(rolls)
    N_gamma = len(gammas)
    # Store best margin and theta at each (roll, gamma_lower)
    smap = np.zeros((N_gamma, N_roll))
    theta_map = np.zeros((N_gamma, N_roll), dtype=int)

    print(f"Sweeping {N_roll} roll x {N_gamma} gamma x {len(thetas)} theta ...")
    for ri, roll in enumerate(rolls):
        for gi, gl in enumerate(gammas):
            best_m = -99.0
            best_t = thetas[0]
            for theta in thetas:
                # symmetric: upper = -lower
                q_list = make_q_list(roll, theta, BETA, gl, -gl)
                m = stability_margin(roll, q_list)
                if m > best_m:
                    best_m = m
                    best_t = theta
            smap[gi, ri] = best_m
            theta_map[gi, ri] = best_t
        best_gi = np.argmax(smap[:, ri])
        print(
            f"  Roll={roll:3.0f}°  best_margin={smap[:, ri].max():+.3f}  "
            f"best_γ={gammas[best_gi]:+d}°  best_θ={theta_map[best_gi, ri]}°"
        )

    # Optimal gamma at each roll
    best_gi = np.argmax(smap, axis=0)
    best_gamma = gammas[best_gi]
    best_margin = smap[best_gi, np.arange(N_roll)]
    best_theta = np.array([theta_map[best_gi[ri], ri] for ri in range(N_roll)])

    # ── Plot ────────────────────────────────────────────────────────────────
    plt.style.use("default")
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle(
        "Diagnostic Quasi-Static Stability Scan  (θ=75°, β=0°)\n"
        "Manual-FSM keyframe scoring reference over Roll × γ_lower",
        fontsize=12,
    )

    # Panel 1: stability heatmap
    ax = axes[0]
    cmap = plt.cm.RdYlGn
    im = ax.pcolormesh(rolls, gammas, smap, cmap=cmap, vmin=-1.0, vmax=1.0, shading="auto")
    ax.contour(rolls, gammas, smap, levels=[0], colors="black", linewidths=1.5, linestyles="--")
    # Optimal path line
    ax.plot(rolls, best_gamma, "k-o", ms=5, lw=2, label="Optimal γ path")
    ax.set_xlabel("Roll (deg)")
    ax.set_ylabel("γ_lower (deg)")
    ax.set_title("Stability Map  (black dashed = stability boundary)")
    ax.legend(fontsize=8)
    ax.invert_xaxis()
    plt.colorbar(im, ax=ax, label="Norm. CoM margin")

    # Panel 2: margin along optimal path
    ax = axes[1]
    stable_mask = best_margin > 0
    ax.fill_between(
        rolls, best_margin, 0, where=stable_mask, alpha=0.3, color="green", label="Stable"
    )
    ax.fill_between(
        rolls, best_margin, 0, where=~stable_mask, alpha=0.3, color="red", label="Unstable"
    )
    ax.plot(rolls, best_margin, "k-o", ms=5, lw=1.5)
    ax.axhline(0, color="black", lw=1, ls="--")
    ax.set_xlabel("Roll (deg)")
    ax.set_ylabel("Norm. CoM margin")
    ax.set_title("Stability Margin Along Optimal γ Path")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax.invert_xaxis()

    # Stable zone annotation
    for roll_m, lbl in [(180, "S4"), (0, "S0")]:
        for a in axes:
            a.axvline(roll_m, color="steelblue", lw=1, ls=":", alpha=0.7)
            a.text(
                roll_m,
                a.get_ylim()[1] if hasattr(a, "get_ylim") else 1,
                lbl,
                fontsize=7,
                color="steelblue",
                ha="center",
                va="top",
            )

    plt.tight_layout()
    os.makedirs("output", exist_ok=True)
    plt.savefig("output/quasi_static_path.png", dpi=180, bbox_inches="tight")
    print("\nSaved → output/quasi_static_path.png")
    plt.close("all")

    # Print readable waypoint table
    print("\n=== Diagnostic Quasi-Static Waypoints (Roll 180→0) ===")
    print(f"{'Roll':>5} | {'γ_lower':>8} | {'theta':>5} | {'margin':>8} | stable?")
    print("-" * 52)
    for ri, roll in enumerate(rolls):
        m = best_margin[ri]
        g = best_gamma[ri]
        t = best_theta[ri]
        flag = "✓" if m > 0 else "✗ DYNAMIC needed"
        print(f"  {roll:3.0f}°  |  {g:+4d}°   | {t:3d}°  | {m:+.4f} | {flag}")
