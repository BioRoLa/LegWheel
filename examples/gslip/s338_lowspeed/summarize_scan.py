"""Summarise the saved s338 low-speed scans (scan_<mode>.jsonl next to this file).

C1 filter as in Stage 2a (duty <= 0.55, apex >= 10 mm). From the LegWheel root:

    .venv/bin/python examples/gslip/s338_lowspeed/summarize_scan.py prod wide band floor krel

Moved 2026-09-14 from the session scratchpad; code unchanged, docstring added.
"""
import json
import sys
from pathlib import Path

S = Path(__file__).parent
for mode in sys.argv[1:]:
    f = S / f"scan_{mode}.jsonl"
    if not f.exists():
        continue
    rows = [json.loads(l) for l in f.read_text().splitlines() if l.strip()]
    rows.sort(key=lambda r: (r["k_rel"], r["v"]))
    print(f"===== {mode}")
    for r in rows:
        fps = r["fps"]
        ok = [x for x in fps if x["duty"] <= 0.55 and x["apex_mm"] >= 10.0]
        print(f"k_rel {r['k_rel']:.0f} v_td {r['v']:.2f} win {r['win']} "
              f"{r['secs']:.0f}s: {len(fps)} FPs, {len(ok)} pass filters")
        # summarise the range of each quantity over all FPs and over passing FPs
        for label, sel in (("all", fps), ("pass", ok)):
            if not sel:
                print(f"   {label}: none")
                continue
            best = min(sel, key=lambda x: abs(x["slope"]))
            fwd = [x["vfwd"] for x in sel]
            print(f"   {label}: vfwd {min(fwd):.3f}-{max(fwd):.3f}; "
                  f"beta {min(x['beta'] for x in sel):.1f}-{max(x['beta'] for x in sel):.1f}; "
                  f"alpha {min(x['alpha'] for x in sel):.1f}-{max(x['alpha'] for x in sel):.1f}; "
                  f"apex {min(x['apex_mm'] for x in sel):.1f}-{max(x['apex_mm'] for x in sel):.1f} mm; "
                  f"duty {min(x['duty'] for x in sel):.3f}-{max(x['duty'] for x in sel):.3f}")
            print(f"     min|slope|: beta {best['beta']:.1f} alpha {best['alpha']:.2f} "
                  f"slope {best['slope']:+.3f} duty {best['duty']:.3f} apex {best['apex_mm']:.1f} "
                  f"vfwd {best['vfwd']:.3f} stance {best['stance']:.3f} flight {best['flight']:.3f} "
                  f"comp {best['comp_mm']:.1f} grf {best['grf_bw']:.2f}")
        # passing FPs with vfwd in plant band
        band = [x for x in ok if 0.16 <= x["vfwd"] <= 0.40]
        if band:
            b = min(band, key=lambda x: abs(x["slope"]))
            print(f"   PLANT BAND (0.16-0.40) passing: {len(band)}; e.g. beta {b['beta']:.1f} "
                  f"alpha {b['alpha']:.1f} vfwd {b['vfwd']:.3f} apex {b['apex_mm']:.1f} "
                  f"duty {b['duty']:.3f} slope {b['slope']:+.3f}")
