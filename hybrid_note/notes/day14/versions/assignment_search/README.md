# Four-leg ascent assignment search, 100 mm x 875 mm top (2026-09-17)

Every assignment u in {R,S}^4 composed by the same gait-clock composer (`day14_step3_driver.py`,
flags in `day14_assignment_search.sh`), each with the composer's own origin candidates.
Front (LF,RF) x rear (RH,LH); RR = both roll (later leg bound).  Logs in `logs/`.

| front | rear | rolling legs | verdict | reason / metrics |
|---|---|---|---|---|
| SS | SS | none (all-step) | refused (4 origins) | rear step loses rotation clearance / front pair stance conflict |
| SS | RS | RH | refused | RF cannot lift inside LF's step |
| SS | SR | LH | refused | RF cannot stay in stance through LF's step |
| SS | RR | RH+LH | refused | same front conflict |
| RS | SS | LF | refused | LH cannot stay in stance through RH's step |
| SR | SS | RF | refused | RH cannot stay in stance through LH's step |
| RR | SS | LF+RF | refused | same rear conflict |
| RS | SR | LF+LH | refused (2 origins) | second roller's approach-window landing behind the window |
| SR | RS | RF+RH | refused (2 origins) | same |
| RR | RS | LF+RF+RH | refused (3 origins) | LH cannot stay in stance through RF's recovery |
| RR | SR | LF+RF+LH | refused (3 origins) | same |
| RR | RR | all roll | refused (4 origins) | same |
| **RS** | **RS** | LF+RH (2R+2S) | passes, 98.4% | peak 100, raised area 93.1 mm·m, terrain airborne 5.65 s, dwell 2.87 s |
| **RS** | **RR** | LF+RH+LH (3R+1S, hardware) | passes, 97.8% | peak 100, area 93.1, airborne **4.98 s**, dwell 3.47 s |
| SR | RR | RF+RH+LH (mirror) | passes, 97.7% | peak 100, area 93.2, airborne 5.00 s, dwell 3.46 s |
| SR | SR | RF+LH (mirror) | passes, 98.0% | peak 100, area 93.1, airborne 5.66 s, dwell 2.87 s |

Ranking keys: peak body rise (tie, 100), raised-height integral over travel (tie, 93.1),
airborne time of terrain transitions during the crossing (3R+1S 4.98 s < 2R+2S 5.65 s).
Dwell time and duration favour 2R+2S (2.87 vs 3.47 s; 23.4 vs 24.4 s).  The all-step
baseline used in hardware needs an explicit pre-lift profile (`--passes 1`), see `ablation_allstep/`.
Per-swing support margins: `support_margins.txt` in each survivor's folder (2R+2S: LH fold-climb
0.96 s airborne at -15.7 mm; 3R+1S: no rear ascent swing).
