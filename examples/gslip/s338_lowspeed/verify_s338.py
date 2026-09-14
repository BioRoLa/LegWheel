"""s338 re-derivation: scans + cache of record. Read-only.

Reads the scan_<mode>.jsonl files next to this file and the paper repo's cache of
record, figures/stage2a_grid.npz, from the sibling checkout ../corgi-abad-icra2027
(beside LegWheel). From the LegWheel root:

    .venv/bin/python examples/gslip/s338_lowspeed/verify_s338.py

Moved 2026-09-14 from the session scratchpad; the only code changes are S and PAPER.
"""
import json
from pathlib import Path
import numpy as np

S = Path(__file__).resolve().parent
PAPER = S.parents[2].parent / "corgi-abad-icra2027"  # sibling of the LegWheel checkout


def ok(x):
    return x["duty"] <= 0.55 and x["apex_mm"] >= 10.0


def in_prod(x):
    return x["beta"] <= 86.0 + 1e-9 and x["alpha"] <= 45.0 + 1e-9


def on_grid(x):
    return abs(x["beta"] * 2 - round(x["beta"] * 2)) < 1e-9


def rng(sel, k, f="{:.3f}"):
    v = [s[k] for s in sel]
    return (f + "-" + f).format(min(v), max(v))


print("=== SCANS")
allpass = []
for mode in ("prod", "wide", "band", "floor", "krel"):
    rows = [json.loads(l) for l in (S / f"scan_{mode}.jsonl").read_text().splitlines() if l.strip()]
    rows.sort(key=lambda r: (r["k_rel"], r["v"]))
    for r in rows:
        fps = r["fps"]
        p = [x for x in fps if ok(x)]
        for x in p:
            allpass.append(dict(x, v=r["v"], k_rel=r["k_rel"], mode=mode))
        line = f"{mode:5s} k{r['k_rel']:.0f} v_td {r['v']:.2f} win {r['win']} : {len(fps)} FP, {len(p)} pass"
        if p:
            line += (f" | vfwd {rng(p,'vfwd')} beta {rng(p,'beta','{:.2f}')} alpha {rng(p,'alpha','{:.2f}')}"
                     f" duty {rng(p,'duty')} apex {rng(p,'apex_mm','{:.1f}')} slope {rng(p,'slope','{:+.3f}')}")
        print(line)

print("\n=== k_rel 18 passing, grouped by v_td (all modes pooled)")
k18 = [x for x in allpass if x["k_rel"] == 18.0]
for v in sorted(set(x["v"] for x in k18)):
    sel = [x for x in k18 if x["v"] == v]
    outp = [x for x in sel if not in_prod(x)]
    inp = [x for x in sel if in_prod(x)]
    print(f"v_td {v:.2f}: {len(sel)} pass; outside prod window {len(outp)}; inside prod window {len(inp)}")
    if outp:
        print(f"   outside: vfwd {rng(outp,'vfwd')} beta {rng(outp,'beta','{:.2f}')} alpha {rng(outp,'alpha','{:.2f}')} "
              f"duty {rng(outp,'duty')} apex {rng(outp,'apex_mm','{:.1f}')} slope {rng(outp,'slope','{:+.3f}')}")
    for x in sorted(inp, key=lambda x: x["beta"]):
        print(f"   INSIDE prod: mode {x['mode']} beta {x['beta']:.2f} on0.5grid {on_grid(x)} alpha {x['alpha']:.2f} "
              f"duty {x['duty']:.4f} apex {x['apex_mm']:.1f} vfwd {x['vfwd']:.3f} slope {x['slope']:+.3f}")
    # continuity in beta: sorted unique betas of passing points
    bs = sorted(set(round(x["beta"], 2) for x in sel))
    gaps = [(a, b) for a, b in zip(bs, bs[1:]) if b - a > 0.51]
    print(f"   passing betas {bs[0]}..{bs[-1]} n={len(bs)} gaps>0.5deg: {gaps}")

print("\n=== pooled outside-window k18 passing (v_td <= 0.75)")
o = [x for x in k18 if not in_prod(x)]
print(f"n {len(o)} v_td {sorted(set(x['v'] for x in o))}")
print(f"vfwd {rng(o,'vfwd')} duty {rng(o,'duty')} apex {rng(o,'apex_mm','{:.1f}')} slope {rng(o,'slope','{:+.3f}')} beta {rng(o,'beta','{:.2f}')} alpha {rng(o,'alpha','{:.2f}')}")
print(f"pooled all k18 passing: vfwd {rng(k18,'vfwd')} duty {rng(k18,'duty')} apex {rng(k18,'apex_mm','{:.1f}')} slope {rng(k18,'slope','{:+.3f}')}")
print("v_td<=0.70 k18 passing vfwd max", max(x["vfwd"] for x in k18 if x["v"] <= 0.70))

print("\n=== v_td 0.70 neighbours of beta 84.25 in scan_band (all FPs, unfiltered)")
rows = [json.loads(l) for l in (S / "scan_band.jsonl").read_text().splitlines() if l.strip()]
for r in rows:
    if abs(r["v"] - 0.70) < 1e-9:
        for x in sorted(r["fps"], key=lambda x: (x["beta"], x["alpha"])):
            if 83.9 <= x["beta"] <= 84.6:
                print(f"   beta {x['beta']:.2f} alpha {x['alpha']:.2f} duty {x['duty']:.4f} apex {x['apex_mm']:.1f} vfwd {x['vfwd']:.3f} slope {x['slope']:+.3f}")

print("\n=== CACHE figures/stage2a_grid.npz")
z = np.load(PAPER / "figures" / "stage2a_grid.npz", allow_pickle=True)
print("files", z.files)
cells = list(z["cells"])
print("n cells", len(cells), "keys", sorted(cells[0].keys()))
vs = sorted(set(round(float(c["v_td"]), 3) for c in cells))
lams = sorted(set(float(c["lam_deg"]) for c in cells))
laws = sorted(set(c["law"] for c in cells))
print("n v_td", len(vs), vs[0], vs[-1], "n lam", len(lams), "laws", laws)
for v in vs:
    sel = [c for c in cells if round(float(c["v_td"]), 3) == v]
    ex = [c for c in sel if c["exists"]]
    per = {law: sum(1 for c in ex if c["law"] == law) for law in laws}
    s = f"v_td {v:.2f}: {len(ex)}/{len(sel)} exist {per}"
    if ex:
        vf = [float(c["v_fwd"]) for c in ex]
        s += f" v_fwd {min(vf):.3f}-{max(vf):.3f}"
    print(s)
