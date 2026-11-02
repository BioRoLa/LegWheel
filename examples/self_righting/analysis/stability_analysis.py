"""
stability_analysis.py  (v3)

Generates a 3-panel academic-quality figure:
  Panel 1: Stability margin + stability window labels (Roll 0→180°)
  Panel 2: CoM height profile = potential energy landscape
  Panel 3: Energy barrier and estimated minimum angular impulse for self-righting

All plots auto-close after DISPLAY_SECONDS.

Usage:
    uv run python examples/stability_analysis.py
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch
from scipy.spatial import ConvexHull
import os, sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

DISPLAY_SECONDS = 30   # auto-close after this many seconds

# ---------------------------------------------------------------------------
# Core helpers
# ---------------------------------------------------------------------------

def get_state(roll_deg, q_list, tol=3e-3):
    """Returns (com_height_m, norm_stability_margin, n_chassis, n_stud, n_wheel)."""
    from legwheel.models.corgi_robot import CorgiRobot
    from legwheel.models.collision_model import CorgiCollisionModel

    robot = CorgiRobot()
    robot.base_ori = np.array([np.deg2rad(roll_deg), 0, 0])
    robot.base_pos = np.array([0, 0, 0.5])
    col = CorgiCollisionModel(robot)

    pts = col.get_all_collision_points(q_list)
    c = np.vstack([pts["chassis"], pts["m6_studs"], pts["wheels"]])
    robot.base_pos[2] -= c[:, 2].min()

    com_h = float(robot.body_to_world(np.zeros(3))[2])

    pts2 = col.get_all_collision_points(q_list)
    c2 = np.vstack([pts2["chassis"], pts2["m6_studs"], pts2["wheels"]])
    min_z2 = c2[:, 2].min()
    mask = c2[:, 2] <= min_z2 + tol
    contact_xy = c2[mask, :2]
    n_ch = int(mask[:16].sum())
    n_st = int(mask[16:20].sum())
    n_wh = int(mask[20:].sum())

    margin = -1.0
    if len(contact_xy) >= 3:
        try:
            hull = ConvexHull(contact_xy)
            com_xy = robot.body_to_world(np.zeros(3))[:2]
            vals = hull.equations @ np.append(com_xy, 1.0)
            min_dist = -vals.max()
            hp = contact_xy[hull.vertices]
            n = len(hp)
            perim = sum(np.linalg.norm(hp[(i+1)%n]-hp[i]) for i in range(n))
            inscribed_r = 2 * hull.volume / perim if perim > 1e-6 else 1e-6
            margin = float(min_dist / inscribed_r)
        except Exception:
            pass
    return com_h, margin, n_ch, n_st, n_wh


# ---------------------------------------------------------------------------
# Stability window definitions (pre-computed, labeled)
# ---------------------------------------------------------------------------

# Canonical stability windows from analysis
STABILITY_WINDOWS = [
    # (name_short, name_full, roll_center_deg, roll_range, gamma_label, color)
    ("S0",  "S0 · Upright",        0,   (0,   0),   "γ=0°",  "#2196F3"),
    ("S1",  "S1 · SFL-α\n(Side-Fall Leading)",
                                   84.5,(84,  85),  "γ=0°",  "#FF9800"),
    ("S2",  "S2 · SFT-α\n(Side-Fall Trailing)",
                                  114.5,(114,115),  "γ=30°", "#9C27B0"),
    ("S3",  "S3 · SFT-β\n(Side-Fall Trailing)",
                                  125.5,(125,126),  "γ=30°", "#E91E63"),
    ("S4",  "S4 · Upside-Down",   180,  (180,180),  "γ=0°",  "#4CAF50"),
]


# ---------------------------------------------------------------------------
# Sweep
# ---------------------------------------------------------------------------

def run_sweep(rolls, q0_list, qg_list):
    """Returns (heights_g0, margins_g0, heights_gx, margins_gx)."""
    h0, m0, hg, mg = [], [], [], []
    for r in rolls:
        h, m, *_ = get_state(r, q0_list)
        h0.append(h); m0.append(m)
        h, m, *_ = get_state(r, qg_list)
        hg.append(h); mg.append(m)
    return np.array(h0), np.array(m0), np.array(hg), np.array(mg)


# ---------------------------------------------------------------------------
# Panels
# ---------------------------------------------------------------------------

def panel_stability_margin(ax, rolls, m0, mg):
    """Panel 1: Stability margin vs Roll."""
    ax.axhline(0, color='black', lw=1.0, ls='--', zorder=1)
    ax.plot(rolls, m0, color='steelblue', lw=1.8, label='γ=0° all legs  (β=0°, θ=75°)', zorder=2)
    ax.plot(rolls, mg, color='darkorange', lw=1.8, ls='--', label='γ=30° lower legs  (β=0°, θ=75°)', zorder=2)

    ax.fill_between(rolls, m0, 0, where=np.array(m0) > 0,
                    color='steelblue', alpha=0.20, zorder=1)
    ax.fill_between(rolls, mg, 0, where=np.array(mg) > 0,
                    color='darkorange', alpha=0.20, zorder=1)

    # Label stability windows
    label_y = 1.15
    for sname, sfull, rc, rrange, glabel, color in STABILITY_WINDOWS:
        rl, rr = rrange
        ax.axvspan(rl - 0.5, rr + 0.5, alpha=0.15, color=color, zorder=0)
        ax.annotate(sname, xy=(rc, 0.0), xytext=(rc, label_y - 0.55 * (rc / 180)),
                    ha='center', fontsize=8, color=color, fontweight='bold',
                    arrowprops=dict(arrowstyle='->', color=color, lw=1.0),
                    bbox=dict(boxstyle='round,pad=0.2', fc='white', ec=color, lw=1.0))

    ax.set_xlim(0, 180)
    ax.set_ylim(-1.3, 1.3)
    ax.set_xlabel("Roll (deg)")
    ax.set_ylabel("Normalized CoM margin")
    ax.set_title("Panel 1 — Stability Margin\ngreen=stable (>0), red=unstable (<0)")
    ax.legend(fontsize=8, loc='lower right')
    ax.grid(True, alpha=0.3)


def panel_com_height(ax, rolls, h0, hg):
    """Panel 2: CoM height = potential energy landscape."""
    h0_mm = np.array(h0) * 1000
    hg_mm = np.array(hg) * 1000

    ax.plot(rolls, h0_mm, color='steelblue', lw=1.8, label='γ=0° all legs  (β=0°, θ=75°)')
    ax.plot(rolls, hg_mm, color='darkorange', lw=1.8, ls='--', label='γ=30° lower legs  (β=0°, θ=75°)')

    # Mark stable window CoM heights
    window_props = {
        "S0":  (0,   h0_mm[0],    'steelblue'),
        "S1":  (84,  h0_mm[84],   'steelblue'),
        "S2":  (114, hg_mm[114],  'darkorange'),
        "S3":  (125, hg_mm[125],  'darkorange'),
        "S4":  (180, h0_mm[180],  'steelblue'),
    }
    for sname, (rx, hy, color) in window_props.items():
        ax.scatter(rx, hy, color=color, s=80, zorder=5)
        ax.annotate(f"{sname}\n{hy:.0f}mm", xy=(rx, hy),
                    xytext=(rx + 8, hy + 10),
                    fontsize=7.5, color=color, fontweight='bold',
                    arrowprops=dict(arrowstyle='->', color=color, lw=0.8))

    # Energy barrier annotation (S4 → S1 path)
    h_start = h0_mm[180]
    h_peak  = h0_mm[np.argmax(h0_mm[90:150]) + 90]
    peak_roll = np.argmax(h0_mm[90:150]) + 90
    dh = h_peak - h_start
    ax.annotate('', xy=(peak_roll, h_peak), xytext=(180, h_start),
                arrowprops=dict(arrowstyle='<->', color='red', lw=1.5))
    ax.text(peak_roll - 15, (h_peak + h_start)/2,
            f"ΔPE/m·g\n≈{dh:.0f}mm",
            fontsize=8, color='red', ha='right')

    ax.set_xlim(0, 180)
    ax.set_xlabel("Roll (deg)")
    ax.set_ylabel("CoM height (mm)")
    ax.set_title("Panel 2 — CoM Height Profile\n(Potential Energy Landscape)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)


def panel_energy_barrier(ax, rolls, h0, hg):
    """
    Panel 3: Energy barrier and minimum angular impulse estimate.
    Assumes robot mass M (kg) and approximates moment arm as CoM-to-contact-line distance.
    """
    g = 9.81  # m/s²

    # The energy barrier (per unit mass): ΔPE/m = g * Δh_com
    # For path S4→S0 (via S1): barrier = h_peak - h_start(S4)
    h0_m = np.array(h0)
    hg_m = np.array(hg)

    h_s4 = h0_m[180]
    h_s0 = h0_m[0]

    # Peak CoM height in each path segment
    h_peak_g0 = h0_m[90:160].max()
    roll_peak_g0 = np.argmax(h0_m[90:160]) + 90

    delta_h_g0  = h_peak_g0 - h_s4   # energy barrier height, g0 path
    delta_h_end = h_s0 - h_s4         # final vs. start height

    mass_range = np.linspace(5, 20, 100)  # kg
    # Moment of inertia approximation: I ≈ M * r²
    # r = moment arm ≈ CoM height at tipping point (distance from contact edge to CoM)
    r_arm = h_peak_g0  # meters
    # Minimum KE = M * g * delta_h  →  ½ I ω² = ½ M r² ω² = M g Δh
    # ω_min = sqrt(2 g Δh) / r_arm
    omega_min_g0 = np.sqrt(2 * g * delta_h_g0) / r_arm if r_arm > 0 else 0
    # Angular impulse L = I * ω = M * r² * ω
    L_per_kg = r_arm**2 * omega_min_g0  # L/M  [m²/s]

    print(f"\n=== Energy Barrier (S4 → S0 via S1) ===")
    print(f"  h_start (S4) = {h_s4*1000:.1f} mm")
    print(f"  h_peak        = {h_peak_g0*1000:.1f} mm  (at Roll={roll_peak_g0}°)")
    print(f"  ΔPE/m/g       = {delta_h_g0*1000:.1f} mm  ({delta_h_g0*1000:.1f} mm × m × g)")
    print(f"  ω_min         ≈ {np.rad2deg(omega_min_g0):.1f} deg/s  (at r_arm={r_arm*1000:.0f}mm)")
    print(f"  L/M           ≈ {L_per_kg:.4f} m²/s")
    print(f"  For M=8kg:  L ≈ {L_per_kg*8:.3f} kg·m²/s")
    print(f"  For M=12kg: L ≈ {L_per_kg*12:.3f} kg·m²/s")

    # Plot: L vs M
    L_vals = mass_range * L_per_kg
    ax.plot(mass_range, L_vals, color='red', lw=2.0, label=f'ΔPE path: S4→S1→S0\n(r_arm≈{r_arm*1000:.0f}mm)')
    ax.fill_between(mass_range, L_vals, 0, alpha=0.15, color='red')

    ax.axvline(10, color='gray', lw=1, ls=':')
    ax.text(10.3, L_vals[np.argmin(np.abs(mass_range - 10))] * 1.05,
            f'M=10kg\nL≈{L_per_kg*10:.2f}', fontsize=8, color='gray')

    ax2 = ax.twinx()
    omega_vals = np.full_like(mass_range, np.rad2deg(omega_min_g0))
    ax2.plot(mass_range, omega_vals, color='navy', lw=1.5, ls='--', label='ω_min (rad/s)')
    ax2.set_ylabel("Min angular velocity (deg/s)", color='navy')
    ax2.tick_params(axis='y', labelcolor='navy')

    ax.set_xlabel("Robot mass (kg)")
    ax.set_ylabel("Min angular impulse L = Iω  (kg·m²/s)")
    ax.set_title(f"Panel 3 — Min Angular Impulse to Self-Right\n"
                 f"(ΔPE barrier at Roll≈{roll_peak_g0}°, Δh={delta_h_g0*1000:.0f}mm)")
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, fontsize=8, loc='upper left')
    ax.grid(True, alpha=0.3)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Updated to beta=0 (realistic walking / sagittal stance) and theta=75 (optimal)
    q_nom = [np.deg2rad(75), np.deg2rad(0), 0.0]
    q_g30 = [np.deg2rad(75), np.deg2rad(0), np.deg2rad(30)]
    q_up  = q_nom

    print("Running Roll 0→180° sweep (1° resolution, beta=0, theta=75)...")
    rolls = np.arange(0, 181, 1)
    h0, m0, hg, mg = run_sweep(rolls, [q_nom]*4, [q_up, q_g30, q_g30, q_up])

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle("Corgi Self-Righting — Stability Windows & Energy Analysis  (β=0°, θ=75°)", fontsize=13, y=1.01)

    panel_stability_margin(ax1, rolls, m0, mg)
    panel_com_height(ax2, rolls, h0, hg)
    panel_energy_barrier(ax3, rolls, h0, hg)

    plt.tight_layout()
    os.makedirs("output", exist_ok=True)
    plt.savefig("output/stability_map.png", dpi=200, bbox_inches='tight')
    print("Saved: LegWheel/output/stability_map.png")

    plt.show(block=False)
    plt.pause(DISPLAY_SECONDS)
    plt.close('all')
