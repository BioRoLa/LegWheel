import csv, sys, re, numpy as np, subprocess, os
S=os.path.dirname(os.path.abspath(__file__))
t=sys.argv[1]; base='hybrid_note/notes/day14'
rows=list(csv.DictReader(open(f'{base}/day14_step3_100mm_roll_{t}_events.csv')))
ax={'front':[], 'rear':[]}
for r in rows:
    if r['row_kind']=='axle_profile': ax[r['axle']].append((float(r['hip_x_mm']), float(r['hip_z_mm'])-219.4478))
def area(kn):
    xs=np.array([k[0] for k in kn]); zs=np.array([k[1] for k in kn]); return float(np.trapezoid(zs, xs))/1e3, float(zs.max())
sw=[r for r in rows if r['row_kind']=='swing']; dw=sum(float(r['dwell_s'] or 0) for r in rows if r['row_kind']=='slowdown')
fa,fp=area(ax['front']); ra,rp=area(ax['rear']); end=max(float(r['end_s']) for r in sw)
mf=f'{S}/margins_{t}.txt'
if not os.path.exists(mf):
    out=subprocess.run(['python3','hybrid_note/scripts/experiments/day14_probes/plan_support_margins.py',f'day14_step3_100mm_roll_{t}',base],capture_output=True,text=True).stdout
    open(mf,'w').write(out)
mrows=[]
for line in open(mf):
    m=re.match(r"(\w+) swing csv\s+([\d.]+)-\s*([\d.]+) s \(plan\s+([\d.-]+)\) margin at liftoff\s+([-+\d.]+) min\s+([-+\d.]+)", line)
    if not m: continue
    leg,a,b,tp,mn=m.group(1),float(m.group(2)),float(m.group(3)),float(m.group(4)),float(m.group(6))
    kind=next((r['kind'] for r in sw if r['leg']==leg and abs(float(r['start_s'])-tp)<0.05),'?')
    mrows.append((leg,kind,b-a,mn))
terr=[r for r in mrows if r[1]!='RECOVERY_SWING' and 'DOWN' not in r[1]]
air=sum(d for _,_,d,_ in terr); worst=min(terr,key=lambda r:r[3]) if terr else None
kinds={}
for r in sw:
    if r['kind']!='RECOVERY_SWING': kinds[r['kind']]=kinds.get(r['kind'],0)+1
print(f"{t}: start {sw[0]['body_x_start_mm'][:5]} peak {fp:.0f}/{rp:.0f} area F {fa:.1f} R {ra:.1f} mean {(fa+ra)/2:.1f} | dwell {dw:.2f}s end {end:.1f}s swings {len(sw)} | terrain airborne {air:.2f}s worst {worst} | {kinds}")
