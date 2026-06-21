#!/usr/bin/env python3
"""
G_offset_comparison.py

Overlay comparison: original vs G_OFFSET=20mm modification.
G moves 20mm toward origin; O_r is compensated so wheel center is unchanged.
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from legwheel.visualization.plot_leg import PlotLeg
from legwheel.models.leg_model import LegModel

G_OFFSET = 0.020  # 20mm toward origin

BLUE = {
    'Actuating_Link': '#1565C0', 'Driven_Link': '#1E88E5',
    'Upper_Rim': '#1976D2', 'Lower_Rim': '#0288D1',
    'Foot_Rim': '#0277BD', 'Upper_Tyre': '#4FC3F7',
    'Construction_Line': '#B3E5FC', 'Axis': '#01579B',
    'trajectory': '#2196F3', 'Joint': '#0D47A1',
}
RED = {
    'Actuating_Link': '#B71C1C', 'Driven_Link': '#E53935',
    'Upper_Rim': '#C62828', 'Lower_Rim': '#D81B60',
    'Foot_Rim': '#AD1457', 'Upper_Tyre': '#F48FB1',
    'Construction_Line': '#FCE4EC', 'Axis': '#880E4F',
    'trajectory': '#F44336', 'Joint': '#7F0000',
}


def _make_leg(g_offset, theme):
    pl = PlotLeg(g_offset=g_offset)
    pl.leg_shape.color_label = theme.copy()
    return pl


def plot_single(theta_deg, g_offset, theme, ax, label_prefix):
    """Plot one version (original or modified) at a given theta."""
    O = [0, 0]
    pl = _make_leg(g_offset, theme)
    pl.plot_by_angle(np.deg2rad(theta_deg), O=O, ax=ax)

    pl.forward(np.deg2rad(theta_deg), 0.0, vector=False)
    G  = np.array([pl.G.real,   pl.G.imag])
    Or = np.array([pl.O_r.real, pl.O_r.imag])

    ax.plot(*G,  'o', color=theme['Foot_Rim'], ms=7, zorder=12)
    ax.plot(*Or, 'k+', ms=9, markeredgewidth=1.5, zorder=15)
    ax.annotate(f'G ({G[1]*1000:.0f} mm)',
                G, xytext=(6, 5), textcoords='offset points',
                fontsize=8, color=theme['Foot_Rim'], fontweight='bold')
    ax.annotate(f'O_r ({Or[1]*1000:.0f} mm)',
                Or, xytext=(6, -13), textcoords='offset points', fontsize=7.5)

    variant = 'Original' if g_offset == 0.0 else f'Modified (+{g_offset*1000:.0f} mm)'
    ax.set_title(f'θ = {theta_deg}°  —  {variant}', fontsize=10)
    ax.set_xlabel('x (m)', fontsize=9)
    ax.set_ylabel('y (m)', fontsize=9)
    ax.grid(True, alpha=0.25)
    ax.set_aspect('equal')


def standing_height_plot(ax):
    thetas = np.arange(17, 161, 0.5)
    h0, h1 = [], []

    lm0 = LegModel(g_offset=0.0)
    lm1 = LegModel(g_offset=G_OFFSET)

    for t in thetas:
        lm0.forward(np.deg2rad(t), 0.0, vector=False)
        lm1.forward(np.deg2rad(t), 0.0, vector=False)
        # After beta0=90° rotation, O_r is purely imaginary → distance = |imag|
        h0.append(abs(lm0.O_r.imag) * 1000)
        h1.append(abs(lm1.O_r.imag) * 1000)

    ax.plot(thetas, h0, color=BLUE['Upper_Rim'], lw=2.5, label='Original (g_offset=0)')
    ax.plot(thetas, h1, color=RED['Upper_Rim'],  lw=2.0, ls='--',
            label=f'Modified (g_offset={G_OFFSET*1000:.0f}mm)')
    ax.set_xlabel('θ (deg)', fontsize=9)
    ax.set_ylabel('Hip → wheel center (mm)', fontsize=9)
    ax.set_title('Standing height vs θ', fontsize=11)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.25)

    # Print comparison table
    print('\n── Standing height comparison (mm) ──')
    print(f'{"θ":>6}  {"h_orig":>8}  {"h_mod":>8}  {"Δh":>8}')
    for td in [17, 45, 90, 120, 160]:
        idx = int((td - 17) / 0.5)
        print(f'{td:>6}°  {h0[idx]:>8.2f}  {h1[idx]:>8.2f}  {h1[idx]-h0[idx]:>+8.4f}')


def G_position_plot(ax):
    """G position (y, after rotation) vs theta for both versions."""
    thetas = np.arange(17, 161, 0.5)
    g0_y, g1_y = [], []

    lm0 = LegModel(g_offset=0.0)
    lm1 = LegModel(g_offset=G_OFFSET)

    for t in thetas:
        lm0.forward(np.deg2rad(t), 0.0, vector=False)
        lm1.forward(np.deg2rad(t), 0.0, vector=False)
        g0_y.append(lm0.G.imag * 1000)
        g1_y.append(lm1.G.imag * 1000)

    ax.plot(thetas, g0_y, color=BLUE['Lower_Rim'], lw=2.5, label='G original')
    ax.plot(thetas, g1_y, color=RED['Lower_Rim'],  lw=2.0, ls='--',
            label=f'G modified (+{G_OFFSET*1000:.0f}mm)')
    ax.fill_between(thetas, g0_y, g1_y, alpha=0.15, color='purple',
                    label=f'Δ = {G_OFFSET*1000:.0f}mm')
    ax.set_xlabel('θ (deg)', fontsize=9)
    ax.set_ylabel('G.y position (mm)', fontsize=9)
    ax.set_title('G point position vs θ', fontsize=11)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.25)


# ── Main ──────────────────────────────────────────────────────────────────────

THETAS = [17, 90, 160]

fig = plt.figure(figsize=(18, 14))
fig.suptitle(
    f'G-point offset experiment  |  G_OFFSET = {G_OFFSET*1000:.0f} mm toward origin\n'
    f'O_r = G + (R − offset)  →  wheel center unchanged',
    fontsize=13, fontweight='bold'
)

gs = gridspec.GridSpec(3, len(THETAS), figure=fig,
                       hspace=0.45, wspace=0.35,
                       top=0.90, bottom=0.05)

# Row 0: Original at each theta
for col, theta in enumerate(THETAS):
    ax = fig.add_subplot(gs[0, col])
    plot_single(theta, 0.0, BLUE, ax, 'Orig')

# Row 1: Modified at each theta
for col, theta in enumerate(THETAS):
    ax = fig.add_subplot(gs[1, col])
    plot_single(theta, G_OFFSET, RED, ax, 'Mod')

# Row 2 left: standing height curve
ax_h = fig.add_subplot(gs[2, :2])
standing_height_plot(ax_h)

# Row 2 right: G position curve
ax_g = fig.add_subplot(gs[2, 2])
G_position_plot(ax_g)

out = 'examples/analysis/G_offset_comparison.png'
plt.savefig(out, dpi=150, bbox_inches='tight')
print(f'\nSaved → {out}')
plt.show()
