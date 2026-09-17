"""Align one hardware trial (Orin + Vicon) to the rl50 plan and print what happened."""
import sys, csv
sys.path.insert(0, "/home/chang/corgi_ws/icra hybrid/LegWheel")
import matplotlib; matplotlib.use("Agg")
import numpy as np
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import hybrid_posture_2d

EXP = "/home/chang/corgi_ws/icra hybrid/corgi-research/05_Experiments/0912exp"
NOTES = "/home/chang/corgi_ws/icra hybrid/LegWheel/hybrid_note/notes/day14"
trial = sys.argv[1]; t_end = float(sys.argv[2]) if len(sys.argv) > 2 else 9.0
p = hybrid_posture_2d(); hold = float(p.hold_hip_z_m)
LEAD = 5.24   # csv row 5240 = plan t 0
hw = np.array([[float(v) for v in r] for r in csv.reader(open(f"{NOTES}/day14_step3_100mm_roll_v7nf_rl50_hardware.csv"))])
ev = [r for r in csv.DictReader(open(f"{NOTES}/day14_step3_100mm_roll_v7nf_events.csv")) if r["row_kind"] == "swing"]
pts = sorted({(float(r["start_s"]), float(r["body_x_start_mm"])) for r in ev} | {(float(r["end_s"]), float(r["body_x_end_mm"])) for r in ev})
pt = np.array([q[0] for q in pts]); px = np.array([q[1] for q in pts])
plan_x = lambda tp: np.interp(tp, pt, px)
front_kn = [(830, 0), (837, 0), (1036, 50), (1212, 50), (1362, 100), (1600, 100), (1700, 0)]
def front_z(hx): return np.interp(hx, [k[0] for k in front_kn], [k[1] for k in front_kn])
def rear_z(tp):   # rl50 window in plan time (script base 3.6/4.3/5.28/6.10 + 0.08)
    return np.interp(tp, [3.68, 4.38, 5.36, 6.18], [0, 50, 50, 0]) if 3.68 <= tp <= 6.18 else (np.interp(plan_x(tp) - 255, [830, 837, 1036, 1136, 1600, 1700], [0, 0, 50, 100, 100, 0]) if tp > 6.18 else 0.0)

# ---- Orin
rows = list(csv.DictReader(open(f"{EXP}/orin/0912_100mm_{trial}.csv")))
to = np.array([float(r["Time"]) for r in rows]); to -= to[0]
keep = to <= 40.0; rows = [r for r, k in zip(rows, keep) if k]; to = to[keep]
cth = {l: np.array([float(r[f"cmd_theta_{l}"]) for r in rows]) for l in "abcd"}
cbe = {l: np.array([float(r[f"cmd_beta_{l}"]) for r in rows]) for l in "abcd"}
sth = {l: np.array([float(r[f"state_theta_{l}"]) for r in rows]) for l in "abcd"}
sbe = {l: np.array([float(r[f"state_beta_{l}"]) for r in rows]) for l in "abcd"}
q = {k: np.array([float(r["imu_orien_" + k]) for r in rows]) for k in "xyzw"}
pitch = np.degrees(np.arcsin(np.clip(2 * (q["w"] * q["y"] - q["z"] * q["x"]), -1, 1)))
roll = np.degrees(np.arctan2(2 * (q["w"] * q["x"] + q["y"] * q["z"]), 1 - 2 * (q["x"] ** 2 + q["y"] ** 2)))
# csv row of orin row 0: match the 8 command values
v0 = np.array([cth["a"][0], cbe["a"][0], cth["b"][0], cbe["b"][0], cth["c"][0], cbe["c"][0], cth["d"][0], cbe["d"][0]])
d = np.abs(hw[:, :8] - v0).max(axis=1); row0 = int(np.argmin(d))
plan_t0 = row0 / 1000.0 - LEAD
tpo = to + plan_t0          # plan time of each orin row (assumes 1 kHz playback)
# verify with LF cmd folds
def folds(a, t):
    out, inside = [], False
    for i in range(len(a)):
        if a[i] < np.deg2rad(20) and not inside: out.append(t[i]); inside = True
        elif a[i] >= np.deg2rad(20): inside = False
    return out
csv_f = [v / 1000.0 - LEAD for v in [i for i in range(len(hw))] if False]
hw_fold = folds(hw[:, 0], np.arange(len(hw)) / 1000.0 - LEAD); or_fold = folds(cth["a"], tpo)
print(f"trial {trial}: orin row0 = csv row {row0} (plan t {plan_t0:+.3f}); LF cmd folds csv {[round(v,2) for v in hw_fold[1:5]]} vs orin {[round(v,2) for v in or_fold[:4]]}")

# ---- Vicon
lines = open(f"{EXP}/vicon/0912_100mm_{trial}.csv", encoding="utf-8-sig").read().splitlines()
names = [n.split(":")[-1] for n in lines[2].split(",")[2::3]]
a = np.array([[float(x) if x != "" else np.nan for x in ln.split(",")] for ln in lines[5:] if len(ln.split(",")) > 3])
tv = (a[:, 0] - a[0, 0]) / 500.0
P = {n: a[:, 2 + 3 * i:5 + 3 * i] for i, n in enumerate(names) if n}
bx = np.nanmean(np.stack([P[f"B{i}"][:, 0] for i in "1234"]), axis=0)
# vicon <-> orin, two ways.  (1) event: LF's first swing -- the orin cmd theta
# reaches 17 deg (rotation starts) vs the marker angle's first fast rotation.
# (2) cross-correlation of the LF leg-angle rate with the state beta rate.
dv = P["G11"] - P["O11"]; ang = np.unwrap(np.arctan2(dv[:, 2], dv[:, 0]))
ang = np.where(np.isnan(ang), np.nanmean(ang), ang)
rate_v = np.gradient(ang, tv)
rate_o = np.gradient(np.unwrap(sbe["a"]), to)
i_fold = next(i for i in range(len(cth["a"])) if cth["a"][i] < np.deg2rad(17.05))
t_rot_orin = to[i_fold]
on = np.abs(rate_v) > 3.0
k = 0
while k < len(on):
    if on[k]:
        j = k
        while j < len(on) and on[j]: j += 1
        if tv[j - 1] - tv[k] > 0.08: break
        k = j
    else: k += 1
t_rot_vicon = tv[k]
shift_ev = t_rot_vicon - t_rot_orin
cands = []
for shift in np.arange(-2.0, 30.0, 0.004):
    ro = np.interp(tv - shift, to, rate_o, left=0, right=0)
    cands.append((abs(np.dot(ro, rate_v)), shift))
cands.sort(reverse=True)
peaks = []
for c, sh in cands:
    if all(abs(sh - q) > 0.5 for _, q in peaks): peaks.append((c, sh))
    if len(peaks) == 3: break
shift = shift_ev
print(f"   vicon<->orin: event shift {shift_ev:.3f} s; correlation peaks {[(round(sh,3), round(c/peaks[0][0],2)) for c, sh in peaks]}; using event")
tpv = tv - shift + plan_t0   # plan time of each vicon row
print(f"   vicon t = plan t + {shift - plan_t0:.3f} s")
# calibrations during flat walking (plan 0.5..2.0)
sel = (tpv > 0.5) & (tpv < 2.0)
zoff = {n: float(np.nanmean(P[n][sel, 2])) - hold * 1e3 for n in ("O11", "O21", "O31", "O41")}
i_still = int(np.argmin(abs(tpv - (-0.30))))
B0 = float(np.nanmean(bx[max(0, i_still - 100):i_still + 1]))
x_off = plan_x(-0.24) - B0     # plan x = vicon x + x_off, pinned where the body is still
print(f"   body centre at rest (vicon) {B0:.1f} mm -> nominal face (887) would be at vicon {B0 + 886.8:.1f}; hip z offsets {[round(v,1) for v in zoff.values()]}")

def contact(theta, beta, hip_x, hip_z):
    sc = p.scene(float(beta), 0.0, hold, theta_rad=float(theta))
    ptsg = np.asarray(sc.geometry.points_world_xz_m, dtype=float)
    k = int(np.argmin(ptsg[:, 1]))
    return hip_x + float(ptsg[k, 0]) * 1e3, hip_z + (float(ptsg[k, 1]) - hold) * 1e3

def at(tp):
    i = int(np.argmin(abs(tpv - tp))); j = int(np.argmin(abs(tpo - tp)))
    return i, j

# LF contact trace -> face
lf = []
for tp in np.arange(2.5, 5.5, 0.05):
    i, j = at(tp)
    hx = float(P["O11"][i, 0]); hz = float(P["O11"][i, 2]) - zoff["O11"]
    cx, cz = contact(sth["a"][j], sbe["a"][j], hx, hz)
    lf.append((tp, hx, cx, cz))
ground = [c for c in lf if c[0] < 4.1 and c[3] < 12]
top = [c for c in lf if c[0] > 4.55 and c[3] > 75]
last_ground = max(ground, key=lambda c: c[2]) if ground else None
first_top = min(top, key=lambda c: c[2]) if top else None
face_lo = last_ground[2] if last_ground else np.nan; face_hi = first_top[2] if first_top else np.nan
owner = float(sys.argv[3]) if len(sys.argv) > 3 else 887.0
face = B0 + owner if (np.isnan(face_lo) or np.isnan(face_hi)) else 0.5 * (face_lo + face_hi)
print(f"   LF last ground contact x {face_lo:.0f} ({'-' if not last_ground else format(last_ground[0], '.2f')}), first top contact x {face_hi:.0f} "
      f"({'-' if not first_top else format(first_top[0], '.2f')}) -> face {face_lo - B0:.0f}..{face_hi - B0:.0f} mm from rest (owner said {owner:.0f}); table uses face at {face - B0:.0f}")
# flat advance per LF cycle
i0, _ = at(0.0); i1, _ = at(2.40)
print(f"   flat: body advance plan t 0->2.40: vicon {bx[i1] - bx[i0]:.0f} mm vs plan {plan_x(2.40) - plan_x(0.0):.0f} mm")

# per-stance body advance on flat ground (landing -> next liftoff), vicon vs plan
print("   stance advances (vicon / plan mm): " + ", ".join(
    f"{leg} {bx[at(t1)[0]] - bx[at(t0)[0]]:.0f}/{plan_x(t1) - plan_x(t0):.0f}"
    for leg, t0, t1 in (("LF", 0.36, 2.40), ("RH", 0.96, 2.76), ("RF", 1.56, 3.11), ("LH", 2.16, 4.49)) if t1 <= tpo[-1]))
# swinging rear wheels: hip (cmd vs measured) and wheel-bottom height over the ground
for leg, mk, col, t0, t1 in (("LH", "O41", "d", 4.45, 4.90), ("RH", "O31", "c", 5.35, 6.20), ("LH climb", "O41", "d", 7.15, 8.25)):
    if t1 > tpo[-1] or t1 > tpv[-1]: continue
    out = []
    for tp in np.arange(t0, t1 + 1e-9, 0.05):
        i, j = at(tp); hz = float(P[mk][i, 2]) - zoff[mk]
        sc = p.scene(float(sbe[col][j]), 0.0, hold, theta_rad=float(sth[col][j]))
        low = float(np.asarray(sc.geometry.points_world_xz_m, dtype=float)[:, 1].min()) - hold
        out.append(f"{tp:.2f}:{hz - hold*1e3:+.0f}/{hz + low*1e3:.0f}")
    print(f"   {leg} swing (t: hip-hold / wheel bottom above ground, mm; cmd rear lift {rear_z(t0):+.0f}): " + " ".join(out))
print("   plan t | plan x | vicon x |  lag | pitch roll | hip z-hold LF RF RH LH | plan F/R | LF contact x,z | RF contact x,z | cmd/state th RF")
fine = list(np.arange(3.0, 4.6, 0.1)) + list(np.arange(4.4, 5.0, 0.1)) + list(np.arange(7.1, 8.4, 0.1))
grid = sorted(set([round(v, 2) for v in np.arange(-0.25, t_end + 0.01, 0.25)] + [round(v, 2) for v in fine if v <= t_end]))
for tp in grid:
    if tp > tpo[-1] or tp > tpv[-1]: break
    i, j = at(tp)
    hz = [float(P[n][i, 2]) - zoff[n] - hold * 1e3 for n in ("O11", "O21", "O31", "O41")]
    cxa, cza = contact(sth["a"][j], sbe["a"][j], float(P["O11"][i, 0]), float(P["O11"][i, 2]) - zoff["O11"])
    cxb, czb = contact(sth["b"][j], sbe["b"][j], float(P["O21"][i, 0]), float(P["O21"][i, 2]) - zoff["O21"])
    print(f"   {tp:5.2f} | {plan_x(tp):6.0f} | {bx[i] + x_off:6.0f} | {plan_x(tp) - bx[i] - x_off:4.0f} | {pitch[j]:5.1f} {roll[j]:5.1f} | "
          + " ".join(f"{v:+4.0f}" for v in hz) + f" | {front_z(plan_x(tp) + 255):+4.0f}/{rear_z(tp):+4.0f} | "
          f"{cxa - face:+5.0f} {cza:4.0f} | {cxb - face:+5.0f} {czb:4.0f} | {np.degrees(cth['b'][j]):4.0f}/{np.degrees(sth['b'][j]):4.0f}")
