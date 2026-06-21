# G-point Offset Experiment — Branch Notes

**Branch**: `experiment/G-point-offset-20mm`  
**Date**: 2026-06-21  
**Goal**: Evaluate the effect of moving G point 20 mm toward origin on mechanism properties.

---

## 1. What Was Changed

### `g_offset` parameter (`leg_model.py`, `plot_leg.py`)

A new scalar parameter `g_offset` (default `0.0`) was added to both `LegModel.__init__` and `PlotLeg.__init__`.

In `LegModel.calculate()`:

```python
G_mech = F_l.real - l8 * cos(ang_OGF)   # linkage-determined G, no offset
self.G  = G_mech + g_offset              # G shifted toward origin
self.L_l = F_l + (G_mech - F_l) * exp(i*ang_LFG) * (R / l8)  # uses G_mech
self.O_r = self.G.real + (R - g_offset)  # compensated → O_r = G_mech + R (unchanged)
```

Key design decisions:
- **L_l / L_r computed from `G_mech`** (not the shifted G), so arc curvature (radius = R) is independent of g_offset.
- **O_r compensated** by subtracting g_offset from the ring arm length, so wheel center stays at exactly the same position at every theta.

---

## 2. Mathematical Analysis

### Standing height — unchanged

After `beta0 = 90°` rotation, `O_r.imag = G_mech.imag + R.imag = const` regardless of g_offset.  
Numerical verification (Δh at all theta): **0.0000 mm**.

### Extension workspace — unchanged

`ΔG(17° → 160°)` = 242.86 mm for both versions.  
g_offset shifts the entire G(θ) curve by a constant, so total extension stroke is identical.

### G position curve

G(θ) shifts uniformly by +20 mm toward origin across the full range.  
This only affects clearance geometry, not kinematics.

---

## 3. Visualization Adjustments (`plot_leg.py`)

### foot_joint circle enlarged

Original tangency condition at any theta:

```
|O_r - G| = R = 100 mm = foot_rim_outer (134.5 mm) − foot_joint_r (34.5 mm)
```

With g_offset = 20 mm:

```
|O_r - G| = R − g_offset = 80 mm = 134.5 − 54.5 mm
```

So `foot_joint` radius becomes `foot_offset + tyre_thickness + g_offset` = 54.5 mm.  
This holds at **all theta** because `|O_r - G| = R - g_offset` is constant.

### lower_rim arc radius fix

`lower_rim_r` is `_make_arc(G_shifted, F_r, L_r, ...)`.  
Default radius would be `|G_shifted - L_r|` which varies from 80–90 mm (wrong, ≠ R).

Fix: pass explicit `radius = |F_r - L_r| = R` via new `radius=` kwarg in `_make_arc`.  
Both lower rim arcs now use radius = 100 mm — symmetric and geometrically correct.

---

## 4. Output

`examples/analysis/G_offset_comparison.png` — 3×3 grid:
- Row 0: Original (blue) at θ = 17°, 90°, 160°
- Row 1: Modified +20 mm (red) at θ = 17°, 90°, 160°
- Row 2L: Standing height vs θ (curves overlap exactly)
- Row 2R: G position vs θ (constant 20 mm offset band)

---

## 5. Conclusion

Moving G 20 mm toward origin:

| Property | Effect |
|---|---|
| Standing height | Unchanged (Δh = 0) |
| Extension stroke ΔG | Unchanged |
| Foot tangency circle | +20 mm radius (54.5 mm) |
| Lower rim arc geometry | Radius = R = 100 mm (symmetric) |
| Mechanism kinematics | Identical |

G point in this mechanism serves only as the pivot anchor for lower linkage symmetry.  
Its absolute position does **not** affect extension workspace or standing height.  
`g_offset` can be used to adjust clearance geometry without any kinematic side-effects.
