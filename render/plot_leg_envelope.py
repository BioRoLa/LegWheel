"""
render/plot_leg_envelope.py

Visualizes the leg mechanism as a CONE GEOMETRY:
  Apex  = M6 stud tip  (red dot, fixed in {Mi} frame)
  Base  = wheel rim points  (sampled by alpha)
  Surface = triangle fan from apex to rim → three incomplete cone sections

Three arc sections:
  Foot rim   alpha ∈ [−35°, +35°]   — contact zone, widest outward → large contact cone
  Side rim   alpha ∈ [±40°, ±80°]  — shoulder arcs → intermediate cone
  Upper rim  alpha ∈ [±85°, ±165°] — upper arcs → narrow back-cone
                                      ** gap at ±165°→±180° (linkage zone) **

The ABAD sweep (γ ±30°) is shown by overlaying the same cone at several γ positions.

Usage:
    uv run python render/plot_leg_envelope.py                        # θ=75°, 4-view
    uv run python render/plot_leg_envelope.py --theta 17             # wheel mode
    uv run python render/plot_leg_envelope.py --all-theta --no-show
"""
import argparse, os, sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.config import RobotParams
from render.plot_corgi_robot import draw_chassis

DISPLAY_SECONDS = 30

# ── Geometry constants ────────────────────────────────────────────────────────
D_WHEEL  = RobotParams.WHEEL_AXIAL_OFFSET   # 0.09168 m
M6_EXTRA = 0.0345                            # 34.5 mm
APEX_M   = np.array([D_WHEEL + M6_EXTRA, 0, 0])

# Alpha ranges (rim_point parametrisation)
SECTIONS = [
    ("Foot rim",  (-35, 35),   "#E63946"),   # red
    ("Side rim",  (40,  80),   "#457B9D"),   # blue-gray
    ("Upper rim", (85, 165),   "#F4A261"),   # orange
]

# ── Core: build cone surface mesh from apex to one rim section ────────────────

def cone_surface(leg: CorgiLegKinematics,
                 theta: float, beta: float, gamma: float,
                 alpha_lo: float, alpha_hi: float,
                 N_alpha: int = 40, N_t: int = 8,
                 mirror: bool = False) -> tuple:
    """
    Returns (X, Y, Z) mesh arrays, shape (N_t, N_alpha), in Body Frame {B}.
    t=0 row = apex repeated; t=1 row = rim points.
    mirror=True flips the alpha range for the symmetric side (alpha_lo→-alpha_hi, etc.)
    """
    if mirror:
        alphas = np.linspace(-alpha_hi, -alpha_lo, N_alpha)
    else:
        alphas = np.linspace(alpha_lo, alpha_hi, N_alpha)
    ts     = np.linspace(0, 1, N_t)

    # Apex in {B}
    apex_B = leg._M_to_B(APEX_M, gamma=gamma)   # shape (3,)

    # Rim points in {B}
    rim_B  = np.array([leg.forward_kinematics(theta, beta, gamma,
                                               alpha=float(a), w=0.0)
                       for a in alphas])         # shape (N_alpha, 3)

    # Interpolate apex → rim
    X = np.outer(1 - ts, np.full(N_alpha, apex_B[0])) + np.outer(ts, rim_B[:, 0])
    Y = np.outer(1 - ts, np.full(N_alpha, apex_B[1])) + np.outer(ts, rim_B[:, 1])
    Z = np.outer(1 - ts, np.full(N_alpha, apex_B[2])) + np.outer(ts, rim_B[:, 2])
    return X, Y, Z


# ── Draw one cone (one section, one gamma) ────────────────────────────────────

def draw_cone_section(ax, leg: CorgiLegKinematics,
                      theta: float, beta: float, gamma: float,
                      alpha_lo: float, alpha_hi: float,
                      color: str, alpha_surf: float = 0.30,
                      N_alpha: int = 40, N_t: int = 8,
                      label: str = ""):
    """Draws the cone surface (apex→rim) for one arc section."""
    for mirror in [False, True] if alpha_lo > 0 else [False]:
        X, Y, Z = cone_surface(leg, theta, beta, gamma,
                                alpha_lo, alpha_hi,
                                N_alpha=N_alpha, N_t=N_t, mirror=mirror)
        surf = ax.plot_surface(X, Y, Z,
                               color=color, alpha=alpha_surf,
                               linewidth=0, antialiased=True,
                               label=label if not mirror else "_")
        # Rim edge line
        ax.plot(X[-1, :], Y[-1, :], Z[-1, :],
                color=color, lw=1.0, alpha=min(alpha_surf + 0.35, 1.0))
        # Apex → rim boundary lines (generating lines at edges of arc)
        for ai in [0, -1]:
            ax.plot([X[0, ai], X[-1, ai]],
                    [Y[0, ai], Y[-1, ai]],
                    [Z[0, ai], Z[-1, ai]],
                    color=color, lw=0.8, alpha=0.5)


# ── Draw apex (M6 stud) marker ────────────────────────────────────────────────

def draw_apex(ax, leg: CorgiLegKinematics, gamma: float, color: str = "#E63946"):
    apex_B = leg._M_to_B(APEX_M, gamma=gamma)
    ax.scatter(*apex_B, color=color, s=60, zorder=10, depthshade=False)


# ── Full leg envelope: multiple gamma slices ─────────────────────────────────

def draw_leg_envelope(ax,
                      leg_index: int,
                      theta: float, beta: float,
                      gamma_max_deg: float,
                      n_gamma_slices: int = 4,
                      N_alpha: int = 40,
                      N_t: int = 8):
    """
    Draws the cone geometry for one leg at n_gamma_slices gamma positions.
    Each slice is shown with decreasing transparency away from gamma=0.
    """
    leg     = CorgiLegKinematics(leg_index)
    gammas  = np.linspace(-np.deg2rad(gamma_max_deg),
                           np.deg2rad(gamma_max_deg),
                           n_gamma_slices)

    for gi, g in enumerate(gammas):
        # Opacity: brightest at gamma=0, fading at extremes
        frac        = 1 - 0.55 * abs(g) / np.deg2rad(gamma_max_deg)
        alpha_surf  = 0.22 * frac
        alpha_edge  = 0.55 * frac

        _already_labeled = set()
        for sec_label, (a_lo, a_hi), color in SECTIONS:
            lbl = sec_label if (gi == 0 and sec_label not in _already_labeled) else "_"
            _already_labeled.add(sec_label)
            draw_cone_section(ax, leg, theta, beta, g,
                              a_lo, a_hi,
                              color=color,
                              alpha_surf=alpha_surf,
                              N_alpha=N_alpha, N_t=N_t,
                              label=lbl)

        # Apex marker at each gamma
        draw_apex(ax, leg, g, color="#E63946")

    # γ=0 rim outline (white reference line)
    alphas_full = np.linspace(-170, 170, 180)
    rim0 = np.array([leg.forward_kinematics(theta, beta, 0.0, alpha=float(a))
                     for a in alphas_full])
    ax.plot(rim0[:, 0], rim0[:, 1], rim0[:, 2],
            color='black', lw=1.5, alpha=0.7, ls='-', zorder=6)


# ── Full robot: 4 legs ────────────────────────────────────────────────────────

def draw_full_robot_cones(ax,
                          theta_deg: float = 75.0,
                          beta_deg: float  = 0.0,
                          gamma_max_deg: float = RobotParams.GAMMA_MAX_DEG,
                          n_slices: int = 4,
                          N_alpha: int = 36,
                          N_t: int = 8):
    theta = np.deg2rad(theta_deg)
    beta  = np.deg2rad(beta_deg)

    draw_chassis(ax, linewidth=1.5, color='#333333', alpha=0.7)

    for leg_idx in range(4):
        draw_leg_envelope(ax, leg_idx, theta, beta,
                          gamma_max_deg=gamma_max_deg,
                          n_gamma_slices=n_slices,
                          N_alpha=N_alpha,
                          N_t=N_t)


# ── Axis styling (light theme) ────────────────────────────────────────────────

def style_ax(ax, title: str = ""):
    ax.set_facecolor('white')
    ax.xaxis.pane.fill = True;  ax.xaxis.pane.set_facecolor('#f0f0f0')
    ax.yaxis.pane.fill = True;  ax.yaxis.pane.set_facecolor('#f0f0f0')
    ax.zaxis.pane.fill = True;  ax.zaxis.pane.set_facecolor('#f5f5f5')
    ax.xaxis.pane.set_edgecolor('#cccccc')
    ax.yaxis.pane.set_edgecolor('#cccccc')
    ax.zaxis.pane.set_edgecolor('#cccccc')
    ax.grid(True, color='#dddddd', linewidth=0.5)
    ax.set_xlabel('X (m)', fontsize=7, color='#444444')
    ax.set_ylabel('Y (m)', fontsize=7, color='#444444')
    ax.set_zlabel('Z (m)', fontsize=7, color='#444444')
    ax.tick_params(colors='#666666', labelsize=5.5)
    ax.set_title(title, fontsize=8.5, color='#222222', pad=6)
    ax.set_xlim(-0.45, 0.45)
    ax.set_ylim(-0.45, 0.45)
    ax.set_zlim(-0.40, 0.20)
    ax.set_box_aspect([1, 1, 0.67])


# ── Plot ──────────────────────────────────────────────────────────────────────

def plot_envelope(theta_deg: float = 75.0,
                  beta_deg: float  = 0.0,
                  save_dir: str    = 'output',
                  show: bool       = True,
                  timeout: float   = DISPLAY_SECONDS,
                  gamma_max_deg: float = RobotParams.GAMMA_MAX_DEG,
                  n_slices: int = 4):

    plt.style.use('default')

    views = [
        ("Front  (+X→−X)",  0,  -90),
        ("Side   (+Y→−Y)",  0,  180),
        ("Top    (+Z→−Z)", 90,    0),
        ("Isometric",       22,  -55),
    ]

    fig = plt.figure(figsize=(20, 5.5), facecolor='white')

    labeled = False
    for idx, (vtitle, elev, azim) in enumerate(views):
        ax = fig.add_subplot(1, 4, idx + 1, projection='3d')

        draw_full_robot_cones(ax,
                              theta_deg=theta_deg,
                              beta_deg=beta_deg,
                              gamma_max_deg=gamma_max_deg,
                              n_slices=n_slices)

        ax.view_init(elev=elev, azim=azim)
        mode = "Wheel mode" if theta_deg <= RobotParams.MIN_THETA_DEG + 2 else "Foot mode"
        style_ax(ax, title=f"{vtitle}")

        if idx == 3:
            # Only add legend on last panel
            from matplotlib.patches import Patch
            from matplotlib.lines import Line2D
            legend_elements = [
                Patch(facecolor=c, alpha=0.7, edgecolor=c, label=lbl)
                for lbl, _, c in SECTIONS
            ] + [
                Line2D([0], [0], color='black', lw=1.5, label='γ=0° rim outline'),
                Line2D([0], [0], marker='o', color='#E63946', lw=0,
                       markersize=6, label='M6 apex'),
            ]
            ax.legend(handles=legend_elements, fontsize=7, loc='lower left',
                      framealpha=0.85, edgecolor='#cccccc')

    mode = "Wheel mode" if theta_deg <= RobotParams.MIN_THETA_DEG + 2 else "Foot mode"
    fig.suptitle(
        f'Corgi — Leg Cone Geometry  ({mode})   '
        f'θ={theta_deg:.0f}°  β={beta_deg:.0f}°  γ_max=±{gamma_max_deg:.0f}°  '
        f'({n_slices} γ-slices overlaid)\n'
        f'Cone: Apex = M6 stud  →  Base = wheel rim   |   '
        f'Red=Foot  Blue=Side  Orange=Upper  (gap at upper ±165°→±180°)',
        fontsize=9, color='#111111', y=1.02
    )

    plt.tight_layout()
    os.makedirs(save_dir, exist_ok=True)
    fname = os.path.join(save_dir, f'leg_envelope_theta{int(theta_deg):03d}.png')
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
    p.add_argument('--theta',     type=float, default=75.0)
    p.add_argument('--beta',      type=float, default=0.0)
    p.add_argument('--gamma-max', type=float, default=RobotParams.GAMMA_MAX_DEG)
    p.add_argument('--n-slices',  type=int,   default=4,
                   help='Number of γ positions to overlay (default 4)')
    p.add_argument('--save-dir',  default='output')
    p.add_argument('--no-show',   action='store_true')
    p.add_argument('--timeout',   type=float, default=DISPLAY_SECONDS)
    p.add_argument('--all-theta', action='store_true',
                   help='Generate θ = 17, 45, 75, 110, 160')
    return p.parse_args()


def main():
    args = parse_args()
    thetas = [17, 45, 75, 110, 160] if args.all_theta else [args.theta]
    for t in thetas:
        print(f"Rendering θ={t}° ...")
        plot_envelope(theta_deg=t,
                      beta_deg=args.beta,
                      gamma_max_deg=args.gamma_max,
                      n_slices=args.n_slices,
                      save_dir=args.save_dir,
                      show=not args.no_show,
                      timeout=args.timeout)


if __name__ == '__main__':
    main()
