"""Append the whole-body animation appendix to the Day 12 dashboard notebook.

Idempotent: previously appended cells (tagged ``MARKER``) are dropped before
the new ones go on.  Kept in the repository, not in a scratch directory
(trap 31).

    python3 LegWheel/hybrid_note/scripts/experiments/day12_animation_notebook_section.py
"""

from pathlib import Path

import nbformat as nbf

NB = (Path(__file__).resolve().parents[2] / "notes"
      / "hybrid_gait_day12_whole_body_dashboard.ipynb")
MARKER = "day12-animation-section"

md = lambda s: nbf.v4.new_markdown_cell(s.strip("\n"),
                                        metadata={"tags": [MARKER]})
code = lambda s: nbf.v4.new_code_cell(s.strip("\n"),
                                      metadata={"tags": [MARKER]})

nb = nbf.read(NB, as_version=4)
nb.cells = [c for c in nb.cells
            if MARKER not in c.get("metadata", {}).get("tags", [])]

cells = []

cells.append(md(r"""
---

# 附錄 A — 全機動畫：把 Step 8 的軌跡畫出來

這一節**不是新的一步**。它不呼叫任何 planner、不產生任何運動：畫面上每一個
姿態都直接來自 Step 8 的 `WholeBodyTrajectory2D`。會加它，是因為到 Step 11
為止，全機規劃只以**表格與指標**的形式存在，而「四隻腳合起來到底長什麼樣」
是表格看不出來的。

## A.1 畫圖需要一個 body 高度，而 Step 5 拒絕給

Step 8 的 241 個取樣裡，`body_z` **241 個都是 `nan`**
（Step 5：451 個 conflict，最差 15.329 mm）。畫圖非有一個高度不可，所以這裡
挑一個，並且**把挑的規則與它的代價一起畫出來**：

> 取那個瞬間**所有 hard demand 的最大值**——也就是
> 「**沒有任何一隻站立腳被壓進地面**」的最低 body 高度。

其他站立腳於是會**浮在地面上方**，浮的量 = 那隻腳自己的需求與被選中的高度之差。
圖上的紅色豎線就是**Step 5 的矛盾本身**，不是繪圖誤差。

反過來取**最小值**也畫得出來，畫面還更「好看」——每隻腳都貼著地——
但那是把同一個矛盾埋到地面**以下**，看起來像接觸。所以取最大值。

而且這個高度**不回寫**：`viewing_heights_2d` 跑完之後，Step 5 的 `body_z`
仍然是 `nan`，`VIEWING_BASIS` 這句話跟著每一列資料走
（`tests/test_day12_whole_body_animation_2d.py` 有一條測試專門守這件事）。
"""))

cells.append(code(r"""
from hybrid_note.scripts.experiments.day12_whole_body_animation_2d import (
    VIEWING_BASIS, viewing_heights_2d, viewing_height_rows,
)

# A.2：畫給人看的必須用 generator frames，理由見下一段。
whole_view = assemble_whole_body_2d(four_flat, traj, stability,
                                    reposition_unresolved=2,
                                    use_generator_frames=True)
heights = viewing_heights_2d(four_flat, whole_view,
                             nominal_body_z_m=traj.nominal_body_z_m)

planned_finite = sum(1 for s in whole_view.samples
                     if np.isfinite(s.body_position_world_m[2]))
print(f"Step 5 給的 body_z，可用的取樣  {planned_finite} / {len(whole_view.samples)}")
print(f"畫圖用的高度範圍               "
      f"{min(h.body_z_m for h in heights) * 1e3:.3f} .. "
      f"{max(h.body_z_m for h in heights) * 1e3:.3f} mm")
print(f"最差的 hard demand 分歧        "
      f"{max(h.spread_m for h in heights) * 1e3:.3f} mm"
      f"   <- 與 Step 5 的 {max(c.disagreement_m for c in traj.conflicts) * 1e3:.3f} mm 同一個數")
print(f"最差被留在空中的腳             "
      f"{max(h.worst_gap_m for h in heights) * 1e3:.3f} mm")
print(f"有腳浮在空中的取樣             "
      f"{sum(1 for h in heights if h.legs_off_the_ground)} / {len(heights)}"
      f"   <- 每一個瞬間都有，這正是 INFEASIBLE 的長相")
print(f"\n{VIEWING_BASIS}")

display(pd.DataFrame(viewing_height_rows(heights))[
    ["time_s", "viewing_body_z_mm", "demand_spread_mm", "worst_float_gap_mm",
     "legs_off_the_ground", "LF_float_gap_mm", "RF_float_gap_mm",
     "LH_float_gap_mm", "RH_float_gap_mm"]].iloc[::40].round(3))
"""))

cells.append(md(r"""
## A.2 姿態要用 generator frames 讀，凍結的數字則不是

`assemble_whole_body_2d(..., use_generator_frames=True)`。預設的 `False` 是在
segment 端點之間內插，而 `RECOVERY_SWING` 的頭尾 `theta` **都是 60°**——
於是整條 swing 的 `theta` 都會是 60°，這條 swing 賴以成立的**縮腿**根本不會出現。
Step 9 的 `peak theta rate` 讀到 0，講的是同一件事。

所以「要看的東西」必須用 generator frames；但 **Day 12 凍結的每一個數字是用內插版
量的**（Step 8 起就是），兩者不要混著引用。下面把兩個讀法的 `theta` 並排一次，
差別就是這一段話的證據。
"""))

cells.append(code(r"""
theta_interp = np.rad2deg([s.legs[LegId.LF].theta_rad for s in whole.samples])
theta_frames = np.rad2deg([s.legs[LegId.LF].theta_rad for s in whole_view.samples])
print(f"LF theta，內插版          {theta_interp.min():.2f} .. {theta_interp.max():.2f} deg"
      f"   <- 整條 swing 都是 60 度")
print(f"LF theta，generator frames {theta_frames.min():.2f} .. {theta_frames.max():.2f} deg"
      f"   <- 17 度是 RecoveryConfig2D.theta_compact")

steps = np.abs(np.diff([s.legs[LegId.LF].contact_world_xy_m[0]
                        for s in whole_view.samples]))
print(f"\n取樣之間最大的接觸點移動   {steps.max() * 1e3:.1f} mm")
print("-> 297 mm 的 chain break【看不到】，而且那是對的：Step 8 的接觸點是相對 hip 放的。")
print("   那 297 mm 活在 handoff 檢查表裡（8.1 節），不在取樣裡。")
"""))

cells.append(md(r"""
## A.3 四個關鍵瞬間：每一次 swing 各取一張

左右兩張矢狀圖是**必要的**：LF 與 RF 的 `x` 完全相同（`±255 mm` 是前後，不是左右），
疊在同一張圖上會互相蓋掉。地面上的短刻度是**世界固定**的，所以看得出 body 真的在前進。

- 黑色 **X** 是 hip、粗黑線是 body；
- 橘色標成 `SWING` 的是當下唯一在空中的腳；
- 綠色 ▽ 是踩在地上的腳，紅色 ▽ 加紅豎線是**被這個高度留在空中的腳**；
- 右邊小圖是支撐三角形與 Step 6 的 margin（不是這裡算的）。
"""))

cells.append(code(r"""
display(Image(filename=str(DAY12 / "day12_whole_body_frames.png")))
"""))

cells.append(md(r"""
## A.4 整趟 3 秒

下面的時間軸：實心 = 站立、斜線 = 空中，紅線是 Step 6 的 stability margin，
虛線是 10 mm 的 floor。**四隻腳同時最多一隻在空中**（Step 3 的 duty 0.75 逼出來的）
在這裡是直接看得到的；margin 掉到 0 也是。

左下角固定印著這條軌跡**疊在什麼之上**——那三行是 `whole.assumptions` 算出來的，
不是圖說。
"""))

cells.append(code(r"""
display(Image(filename=str(DAY12 / "day12_whole_body_animation.gif")))
"""))

cells.append(md(r"""
## A.5 這張圖**不是**什麼

```text
不是可行性的證據   四個未解結論原封不動，就印在動畫左下角
不是實機時間       時間軸是 Step 3 的建模決定（duty 0.75），不是量到的物理時間
不是 CoM           粗黑線是 body，不是質心；這條 pipeline 沒有質量模型
不是新的量測       margin 來自 Step 6、姿態來自 Step 8、高度是畫圖挑的
```

它**是**什麼：Day 12 第一次可以用眼睛檢查「四隻腳合起來的規劃」有沒有明顯錯位——
而它第一眼就會告訴你，**每一個瞬間都有一隻腳浮在地面上方**。
那不是動畫的瑕疵，那是 Step 5 的 INFEASIBLE 長成的樣子。

重畫（約 50 秒，含 GIF）：

```bash
python3 -u LegWheel/hybrid_note/scripts/experiments/day12_animation_driver.py
```
"""))

cells.append(code(r"""
import subprocess
proc = subprocess.run(
    ["python3", "-m", "pytest",
     "tests/test_day12_whole_body_trajectory_2d.py",
     "tests/test_day12_whole_body_animation_2d.py", "-q"],
    cwd=ROOT, capture_output=True, text=True,
    env={**os.environ, "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1"},
)
print(proc.stdout.strip().splitlines()[-1])
"""))

nb.cells.extend(cells)
nbf.write(nb, NB)
print(f"wrote {NB}  ({len(nb.cells)} cells, {len(cells)} appended)")
