"""
examples/self_righting/plot_recovery_strategy.py

Visualises the quasi-static self-righting strategy based on scan results.
Shows the recovery path with stable zones, dynamic gaps, and joint schedules.
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch
import os

# ── Scan results (from independent_gamma_scan.py, 10° resolution) ────────────
# (roll, best_margin, theta, gamma_lower, gamma_upper)
SCAN = [
    (180, +0.829, 60,  +30, +30),
    (170, +0.871, 45,  -20, +30),
    (160, -0.980, 90,  -10,   0),
    (150, -0.578, 90,  -20,   0),
    (140, -1.000, 45,  -30,   0),
    (130, +0.549, 60,  +30,   0),
    (120, +0.528, 75,  +20,   0),
    (110, +0.485, 90,    0,   0),
    (100, +0.185, 90,  -10,   0),
    ( 90, -0.337, 90,  -20,   0),
    ( 80, -0.817, 90,  -30, -30),
    ( 70, -0.919, 60,  -30, -30),
    ( 60, -1.000, 45,  -30,   0),
    ( 50, -1.000, 45,  -30,   0),
    ( 40, -1.000, 45,  -30,   0),
    ( 30, -1.000, 45,  -30,   0),
    ( 20, -1.000, 45,  -30,   0),
    ( 10, +0.875, 75,  -10, +30),
    (  0, +0.967, 45,  +30, +30),
]

rolls   = np.array([r[0] for r in SCAN])
margins = np.array([r[1] for r in SCAN])
thetas  = np.array([r[2] for r in SCAN])
gamma_l = np.array([r[3] for r in SCAN])
gamma_u = np.array([r[4] for r in SCAN])
stable  = margins > 0

# Phase definitions
PHASES = [
    {"name": "Phase 0\nS4 Stable",   "rolls": (180, 170), "color": "#2ecc71", "type": "static"},
    {"name": "Phase 1\nDynamic\n(~30°)", "rolls": (160, 140), "color": "#e74c3c", "type": "dynamic"},
    {"name": "Phase 2\nQuasi-static\n(~40°)", "rolls": (130, 100), "color": "#3498db", "type": "quasi"},
    {"name": "Phase 3\nDynamic\n(~90°)", "rolls": (90, 20),  "color": "#e74c3c", "type": "dynamic"},
    {"name": "Phase 4\nS0 Stable",   "rolls": (10, 0),   "color": "#2ecc71", "type": "static"},
]

plt.style.use("default")
fig, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=True)
fig.suptitle(
    "Corgi Self-Righting — Quasi-Static Path Strategy\n"
    "(Scan: θ∈{45,60,75,90}°, γ_lower∈[−30,30]°, γ_upper∈{0,±30}°, β=0°, Δroll=10°)",
    fontsize=12)

# ── Panel 1: Stability margin + phase bands ───────────────────────────────────
ax = axes[0]
for ph in PHASES:
    r0, r1 = max(ph["rolls"]), min(ph["rolls"])
    c = ph["color"]
    ax.axvspan(r1, r0, alpha=0.15, color=c, zorder=0)
    ax.text((r0 + r1) / 2, 1.05, ph["name"],
            ha="center", va="bottom", fontsize=7, color=c,
            fontweight="bold")

ax.fill_between(rolls, margins, 0, where=stable,  alpha=0.4, color="#2ecc71")
ax.fill_between(rolls, margins, 0, where=~stable, alpha=0.4, color="#e74c3c")
ax.plot(rolls, margins, "ko-", ms=5, lw=1.5)
ax.axhline(0, color="black", lw=1.2, ls="--")
ax.set_ylabel("Norm. CoM margin")
ax.set_title("Stability Margin (best joint config per Roll angle)")
ax.set_ylim(-1.15, 1.35)
ax.grid(alpha=0.3)

# Stable window labels
for roll, m in zip(rolls, margins):
    if m > 0:
        ax.annotate(f"+{m:.2f}", (roll, m),
                    textcoords="offset points", xytext=(0, 6),
                    ha="center", fontsize=6.5, color="#27ae60")

# ── Panel 2: Optimal gamma schedule ──────────────────────────────────────────
ax = axes[1]
for ph in PHASES:
    r0, r1 = max(ph["rolls"]), min(ph["rolls"])
    ax.axvspan(r1, r0, alpha=0.10, color=ph["color"], zorder=0)

ax.plot(rolls, gamma_l, "b-o", ms=5, lw=1.5, label="γ_lower (ground-side legs)")
ax.plot(rolls, gamma_u, "r--s", ms=5, lw=1.5, label="γ_upper (air-side legs)")
ax.axhline(0,   color="gray", lw=0.8, ls=":")
ax.axhline(30,  color="gray", lw=0.5, ls=":")
ax.axhline(-30, color="gray", lw=0.5, ls=":")
ax.set_ylabel("ABAD γ (deg)")
ax.set_title("Optimal ABAD Schedule  (key insight: γ_upper=+30° unlocks S4-side and S0-side)")
ax.legend(fontsize=8)
ax.grid(alpha=0.3)
ax.set_ylim(-38, 42)

# ── Panel 3: Optimal theta ────────────────────────────────────────────────────
ax = axes[2]
for ph in PHASES:
    r0, r1 = max(ph["rolls"]), min(ph["rolls"])
    ax.axvspan(r1, r0, alpha=0.10, color=ph["color"], zorder=0)

ax.plot(rolls, thetas, "g-^", ms=5, lw=1.5, label="θ (best extension angle)")
ax.axhline(75, color="orange", lw=1, ls="--", alpha=0.6, label="θ=75° (cone analysis optimum)")
ax.set_xlabel("Roll (deg)")
ax.set_ylabel("θ (deg)")
ax.set_title("Optimal Extension Angle θ Along Path")
ax.legend(fontsize=8)
ax.grid(alpha=0.3)

for ax in axes:
    ax.invert_xaxis()

# Phase legend
legend_handles = [
    mpatches.Patch(color="#2ecc71", alpha=0.6, label="Static / Quasi-static (stable)"),
    mpatches.Patch(color="#e74c3c", alpha=0.6, label="Dynamic gap (needs momentum)"),
    mpatches.Patch(color="#3498db", alpha=0.6, label="Quasi-static descent"),
]
axes[0].legend(handles=legend_handles, fontsize=8, loc="lower right")

plt.tight_layout()
os.makedirs("output", exist_ok=True)
plt.savefig("output/recovery_strategy.png", dpi=180, bbox_inches="tight")
print("Saved → output/recovery_strategy.png")
plt.close("all")

# Print strategy summary
print("\n" + "="*60)
print("RECOVERY STRATEGY SUMMARY")
print("="*60)
for ph in PHASES:
    r0, r1 = max(ph["rolls"]), min(ph["rolls"])
    span = r0 - r1
    ptype = "STATIC/QUASI" if ph["type"] != "dynamic" else "*** DYNAMIC ***"
    print(f"  {ph['name'].replace(chr(10),' '):30s}  Roll {r0}°→{r1}°  ({span}°)  [{ptype}]")
