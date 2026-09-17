"""Support margin of the CoM (body centre) over every swing of an exported plan, from the
hardware CSV (theta/beta per leg) + phase CSV (1 = airborne), using the posture's rim geometry."""
import sys, numpy as np
sys.path.insert(0, '.')
import matplotlib; matplotlib.use('Agg')
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import hybrid_posture_2d
tag = sys.argv[1]; base = sys.argv[2] if len(sys.argv) > 2 else 'hybrid_note/notes/day14'
p = hybrid_posture_2d(); hold = float(p.hold_hip_z_m)
hw = np.loadtxt(f'{base}/{tag}_hardware.csv', delimiter=',')
ph = np.loadtxt(f'{base}/{tag}_hardware_phase.csv', delimiter=',', skiprows=1)
legs = ['LF', 'RF', 'RH', 'LH']; hx = {'LF': 255, 'RF': 255, 'RH': -255, 'LH': -255}; hy = {'LF': 211.7, 'RF': -211.7, 'RH': -211.7, 'LH': 211.7}
cache = {}
SIGN = -1.0  # the Walk-contract CSV beta is mirrored w.r.t. the planner beta (checked: descent landing +145 ahead)
def dx(theta, beta):
    k = (round(theta, 3), round(beta, 3))
    if k not in cache:
        sc = p.scene(float(beta), 0.0, hold, theta_rad=float(theta))
        pts = np.asarray(sc.geometry.points_world_xz_m, dtype=float)
        cache[k] = float(pts[int(np.argmin(pts[:, 1])), 0]) * 1e3 * SIGN
    return cache[k]
def margin(pts):  # signed distance of (0,0) to the triangle: + inside
    P = np.array(pts); c = P.mean(axis=0); best = None
    for i in range(3):
        a, b = P[i], P[(i + 1) % 3]; e = b - a; n = np.array([-e[1], e[0]]); n /= np.linalg.norm(n)
        if np.dot(n, c - a) < 0: n = -n            # inward normal
        d = float(np.dot(n, -a))                    # (0,0) - a
        best = d if best is None else min(best, d)
    return best
def contacts(i):
    return {l: (hx[l] + dx(hw[i, 2 * j], hw[i, 2 * j + 1]), hy[l]) for j, l in enumerate(legs)}
# swings from the phase file
for j, l in enumerate(legs):
    col = ph[:, j]; start = None; runs = []
    for i, v in enumerate(col):
        if v and start is None: start = i
        if not v and start is not None: runs.append((start, i)); start = None
    for a, b in runs:
        if a / 1000 < 5.2 or a / 1000 > 22.5: continue
        ms = []
        for i in range(a, b, 10):
            c = contacts(i); sup = [c[k] for k in legs if k != l]
            ms.append(margin(sup))
        c0 = contacts(a); off = {k: round(c0[k][0] - hx[k]) for k in legs if k != l}
        print(f"{l} swing csv {a/1000:6.2f}-{b/1000:6.2f} s (plan {a/1000-5.24:5.2f}) margin at liftoff {ms[0]:+6.1f} min {min(ms):+6.1f} mm  stance contact-hip: {off}")
