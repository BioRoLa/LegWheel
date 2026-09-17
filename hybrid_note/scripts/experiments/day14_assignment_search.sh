#!/bin/bash
# Finite four-leg ascent assignment search at 100 mm x 875 mm (2026-09-17): every u in {R,S}^4,
# one composer, one clock, the composer's own origin candidates.  Front modes SS/RS/SR/RR = (LF,RF);
# rear modes = (RH,LH).  RR = both roll, the later leg in gait order bound to the earlier.
C=${DAY14_CACHE:-/home/chang/corgi_ws/day14_cache}; mkdir -p $C/logs/assign
run() { tag=$1; shift; DAY14_TRACE=1 DAY14_STROKE_CACHE=$C/stroke_cache4 setsid python3 -u hybrid_note/scripts/experiments/day14_step3_driver.py --height-mm 100 --samples 2401 --cycles-after 3 --passes 4 --hop-early --early-top-landing --late-rise-mm 0 --late-rise-rear-mm 50 --top-length-mm 875 "$@" --tag $tag > $C/logs/assign/step3_100${tag}.log 2>&1 < /dev/null & disown; }
declare -A FR=( [SS]="" [RS]="LF" [SR]="RF" [RR]="LF,RF" ); declare -A FB=( [SS]="" [RS]="" [SR]="" [RR]="RF" )
declare -A RR=( [SS]="" [RS]="RH" [SR]="LH" [RR]="RH,LH" ); declare -A RB=( [SS]="" [RS]="" [SR]="" [RR]="LH" )
for f in SS RS SR RR; do for r in SS RS SR RR; do
  legs=$(echo "${FR[$f]},${RR[$r]}" | sed 's/^,//;s/,$//'); bound=$(echo "${FB[$f]},${RB[$r]}" | sed 's/^,//;s/,$//')
  run _A${f}_${r}o --rolling-legs "$legs" --bound-legs "$bound"
done; done
# rank survivors: python3 hybrid_note/scripts/experiments/day14_probes/assignment_metrics.py A<front>_<rear>o
