"""Append the Step 5 section to the Day 12 dashboard notebook.

Idempotent: any previously appended Step 5 cells (marked by MARKER) are dropped
before the new ones go on, so re-running replaces rather than duplicates.
"""
from pathlib import Path
import nbformat as nbf

NB = Path("/home/chang/corgi_ws/icra hybrid/LegWheel/hybrid_note/notes/"
          "hybrid_gait_day12_whole_body_dashboard.ipynb")
MARKER = "day12-step5-section"

md = lambda s: nbf.v4.new_markdown_cell(s.strip("\n"),
                                        metadata={"tags": [MARKER]})
code = lambda s: nbf.v4.new_code_cell(s.strip("\n"),
                                      metadata={"tags": [MARKER]})

nb = nbf.read(NB, as_version=4)
nb.cells = [c for c in nb.cells
            if MARKER not in c.get("metadata", {}).get("tags", [])]

# The header table's Step 5 row, and the trailing regression cell, live in the
# already-built cells; update them in place rather than adding a second copy.
for cell in nb.cells:
    if cell.cell_type == "markdown" and "| 5 | body requirement" in cell.source:
        cell.source = cell.source.replace(
            "| 5 | body requirement → whole-body trajectory | ⬜ |",
            "| **5** | body requirement → whole-body trajectory "
            "| ✅ 完成（結論是 INFEASIBLE） |")

cells = []

cells.append(md(r"""
---

# Step 5 — Body Requirement → Whole-Body Trajectory

規格 §12。Day 10–11 說了每隻腳需要 body 怎麼配合，Step 4 把這些需求放上同一個時鐘。
Step 5 把它們合成**一條** `body_x(t)` / `body_z(t)`——
**deterministic、沒有 optimizer**（要求 4），而且**衝突時拒絕，不平均**（要求 5）。

## 5.1 合併規則只有一個函式

```text
hard（TRACK / PINNED）有兩個以上而且不合  ->  INFEASIBLE（記下誰跟誰差多少）
hard 只有一個（或彼此相容）                ->  body_z = 那個值
沒有 hard                                ->  body_z = max(active lower bounds, nominal)
```

`hip_z -> body_z` 用的是 Step 2 已經定好的關係
（`body_z = hip_z - ABAD_AXIS_OFFSET`），不是新數字。

**CoM 的說法**：這個 2D pipeline 沒有 whole-robot 質量模型，
所以每一列都掛著 `quasi-static body-frame approximation`——
標籤在**資料上**，這樣它不會在往 paper 的路上被弄丟。
"""))

cells.append(code(r"""
from hybrid_note.scripts.experiments.day10_11_composer_2d import ComposedSequence2D
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (
    BODY_BASIS, HIP_TO_BODY_Z_M, HARD_AGREEMENT_M,
    BodyDriver, LegDemand2D, body_trajectory_2d, body_rows, merge_demands,
)
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import BodyRequirementKind
from legwheel.config import RobotParams

print(f"basis                {BODY_BASIS}")
print(f"hip -> body offset   {HIP_TO_BODY_Z_M*1e3:.3f} mm"
      f"   == RobotParams.ABAD_AXIS_OFFSET: "
      f"{HIP_TO_BODY_Z_M == RobotParams.ABAD_AXIS_OFFSET}")

NOM = 0.162283
def demand(leg, kind, z):
    return LegDemand2D(leg=leg, segment_index=0, segment_kind="FOOT_RIM_ROLL",
                       phase=None, kind=kind, body_z_m=z, mode=LegMode.STANCE)

cases = {
    "沒有讓步": [],
    "一個下界": [demand(LegId.LF, BodyRequirementKind.LOWER_BOUND, NOM + 0.020)],
    "多個相容下界": [demand(LegId.LF, BodyRequirementKind.LOWER_BOUND, NOM + 0.010),
                     demand(LegId.RF, BodyRequirementKind.LOWER_BOUND, NOM + 0.025)],
    "互斥的 hard": [demand(LegId.LF, BodyRequirementKind.TRACK, 0.20),
                    demand(LegId.RH, BodyRequirementKind.TRACK, 0.24)],
}
for name, ds in cases.items():
    z, driver, winner, conflicts = merge_demands(ds, NOM, 0.0)
    shown = "nan" if np.isnan(z) else f"{z*1e3:.3f} mm"
    print(f"  {name:14s} body_z = {shown:12s}"
          f"   driver = {driver.value:11s} conflicts = {len(conflicts)}")
print("\n-> 互斥的那一列【不會】產生 0.22 m 這個中點。規格要求 5。")
"""))

cells.append(md(r"""
## 5.2 Step 5 最重要的結果：**平地四腳 nominal cycle 是 INFEASIBLE**

這不是 merge 寫壞，是 merge 照規格做事之後**問出來的真相**。

foot-rim 滾動的 hip 高度是一段**弧**（Step 1 量的）：
兩端 202.161 mm、中間（α = 0）219.448 mm，一個 stroke 起伏 **17.287 mm**。

Walk 的 phase offset 把三隻站立腳放在這條弧的**三個不同位置**，
於是在同一個瞬間，三隻腳各自要求一個不同的 body 高度——
而它們全都是 `TRACK`，**TRACK 是 hard**。
"""))

cells.append(code(r"""
probe = cycle_segments_2d(run_nominal_cycles_2d(1)[0], source_id="nb_probe")[0]
prof = probe.body_requirement.hip_z_profile_m
print(f"一個 stroke 的 hip_z：兩端 {prof[0]*1e3:.3f} mm，"
      f"最高 {prof.max()*1e3:.3f} mm，起伏 {(prof.max()-prof.min())*1e3:.3f} mm")

fig, ax = plt.subplots(figsize=(7.5, 2.6))
ax.plot(np.linspace(0, 1, len(prof)), prof * 1e3, color="#1a1a1a", lw=1.6)
for frac, colour in ((0.0, "#c5221f"), (1/3, "#b06000"), (2/3, "#2a6f4e")):
    i = int(frac * (len(prof) - 1))
    ax.scatter([frac], [prof[i] * 1e3], s=45, color=colour, zorder=3)
    ax.annotate(f"{prof[i]*1e3:.1f}", (frac, prof[i] * 1e3),
                textcoords="offset points", xytext=(4, 6), fontsize=8,
                color=colour)
ax.set_xlabel("phase within the stroke"); ax.set_ylabel("hip_z [mm]")
ax.set_title("three stance legs sit at three points on one arc",
             fontsize=9, loc="right")
ax.grid(alpha=0.25); fig.tight_layout(); plt.show()
"""))

cells.append(code(r"""
composed_flat = ComposedSequence2D(
    strategy=StrategyId.SWING_SWING, height_m=0.0, top_length_m=0.0,
    sequence=None, refusal="flat run, no crossing")
plans_flat = {leg: build_leg_plan_2d(leg, composed_flat) for leg in LEG_ORDER}
four_flat = plan_four_legs_2d(plans_flat, walk_timing_2d())

state_nom = initialize_four_leg_state_2d(SharedTerrainSpec2D(
    height_m=0.04, top_length_m=0.40, x_start_m=1.00,
    obstacle_id="day12_platform"))
traj = body_trajectory_2d(
    four_flat, nominal_body_z_m=float(state_nom.body_position_world_m[2]))

summary = traj.as_dict()
print(f"取樣                {summary['samples']}")
print(f"可行的取樣          {summary['feasible_samples']}")
print(f"conflicts           {summary['conflict_count']}"
      f"   feasible = {summary['is_feasible']}")
print(f"最嚴重的分歧        {summary['max_disagreement_mm']:.3f} mm"
      f"   （上界就是那 17.287 mm）")
display(pd.DataFrame([c.as_dict() for c in traj.conflicts[:5]]))
"""))

cells.append(md(r"""
所以 Step 5 **不會**交給你一條平滑的 `body_z` 曲線。那條曲線要存在，
得先承認下面三條路之一：

```text
(a) body 允許 heave / pitch      -> 但規格 §12 要求 6 在對稱測試裡把 rpy 釘在 0
(b) 每隻腳用 theta 補償弧的起伏   -> 那是 joint trajectory（Step 8）的事
(c) 換一個 hip 高度不隨滾動起伏的 nominal cycle
```

**三條都不是 Step 5 可以自己決定的**，所以 Step 5 的正確輸出就是
「不可行，而且這是誰跟誰差多少」。

> 這條和 Step 4 的 airborne overrun 是**兩件獨立的事**：
> 一個是**時間**放不下（0.6 s 的窗口要裝 1.2 s），
> 一個是**高度**對不起來（15.3 mm）。兩個都在，而且互不掩蓋。
"""))

cells.append(code(r"""
display(Image(filename=str(DAY12 / "day12_step5_body_trajectory.png")))
"""))

cells.append(md(r"""
## 5.3 Step 5 驗收（規格 §12 的九項）

| # | 要求 | 結果 |
|---|---|---|
| 1 | 吃 Step 4 同步過的 requirement timeline | ✅ 直接讀 `FourLegPlan2D` |
| 2 | HARD（TRACK/PINNED）必須滿足 | ✅ hard 一律壓過 lower bound |
| 3 | LOWER_BOUND 取滿足所有 active 腳的最小 | ✅ 取最大下界，且**不會**把 body 往下拉 |
| 4 | 不加 optimizer | ✅ `merge_demands()` 是純規則 |
| 5 | 互斥 hard → infeasible，不平均 | ✅ 回 NaN ＋ `BodyConflict2D`；有測試擋住中點 |
| 6 | body_x/z 連續、y 與 rpy 維持 nominal | ✅ body_x 單調連續；y/rpy 掛在資料上可驗 |
| 7 | 與四腳同步的取樣 | ✅ 在 `covered_interval_s` 上取樣 |
| 8 | 記錄垂直變化 | ✅ `body_z_travel_m`，跳過 NaN，不被 infeasible 抹掉 |
| 9 | 四個測試情境 | ✅ 無讓步 / 一個下界 / 多個相容下界 / 互斥 hard |

**Step 5 交給 Step 6 的東西**

```text
BodyTrajectory2D   .samples（每個都帶 driver_leg / driver_segment）
                   .conflicts / .is_feasible / .concession_intervals()
                   .body_z_travel_m / .max_body_z_step_m
merge_demands()    規則本身，可單獨測試與重用
BODY_BASIS         「這是 body frame 近似，不是 whole-robot CoM」
【要帶著走的結論】  平地四腳 nominal cycle 不可行，最大分歧 15.329 mm
```

**Step 6 要做的**：三腳 support triangle ＋ CoM stability margin（規格 §13）。
開始之前要先知道：Step 5 已經說了 body 高度本身就對不起來，
所以 Step 6 的穩定性結論必須說清楚它是在**哪一個** body 假設下算的。
"""))

cells.append(code(r"""
import subprocess
proc = subprocess.run(
    ["python3", "-m", "pytest",
     "tests/test_day12_segment_contract_2d.py",
     "tests/test_day12_nominal_cycle_2d.py",
     "tests/test_day12_four_leg_state_2d.py",
     "tests/test_day12_timing_skeleton_2d.py",
     "tests/test_day12_transition_mapping_2d.py",
     "tests/test_day12_body_trajectory_2d.py", "-q"],
    cwd=ROOT, capture_output=True, text=True,
    env={**os.environ, "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1"},
)
print(proc.stdout.strip().splitlines()[-1])
"""))

nb.cells.extend(cells)
nbf.write(nb, NB)
print(f"wrote {NB}  ({len(nb.cells)} cells, {len(cells)} appended)")
