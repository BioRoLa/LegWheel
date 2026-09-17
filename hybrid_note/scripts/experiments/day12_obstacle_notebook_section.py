"""Append the crossing-registration appendix to the Day 12 dashboard notebook.

Idempotent: previously appended cells (tagged ``MARKER``) are dropped before
the new ones go on.  Kept in the repository, not in a scratch directory
(trap 31).

    python3 LegWheel/hybrid_note/scripts/experiments/day12_obstacle_notebook_section.py
"""

from pathlib import Path

import nbformat as nbf

NB = (Path(__file__).resolve().parents[2] / "notes"
      / "hybrid_gait_day12_whole_body_dashboard.ipynb")
MARKER = "day12-obstacle-section"

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

# 附錄 B — 越障段的世界座標註冊

附錄 A 畫的是**平地**。要把越障也畫出來，第一件事不是畫地形，而是先問一個
到 Step 11 為止**沒有任何一步問過**的問題：

> 規劃出來的越障 swing，落在哪一個世界座標？

## B.1 為什麼這個問題到現在還沒有答案

Step 5 自己就寫著（`body_trajectory_2d` 的 docstring）：`body_x` 是從支撐腳的
**增量**積出來的，因為「Step 4 給每隻腳同一條 chain，它們的 `hip_x` 都從 0 開始，
**絕對值不共用原點**」。那是對的做法——但它同時把越障段自己的座標系丟掉了。

而地形只走到**決策**為止：`plan_terrain_2d` 把 terrain 餵給 `decide_2d` /
`compose_2d` 選策略，之後就沒有再往下傳；Step 8 又把接觸點放成**相對 hip**。

結果是組出來的軌跡**有 `SWING_UP` / `SWING_DOWN`，卻沒有障礙物**。
（附錄 A 提到「297 mm 的 chain break 看不到」是同一個機制的另一面。）

## B.2 註冊就是一個減法

一個越障 segment 知道自己座標系裡的 hip 起點與終點；軌跡知道那兩個瞬間那隻腳的
hip 實際在哪裡。差值就是位移，把 composition frame 的障礙物搬進世界：

```text
implied_x_start = world_hip_x - local_hip_x + COMPOSER_FRAME_X_START_M
```

`COMPOSER_FRAME_X_START_M` 現在是 `day10_11_composer_2d` 裡**具名的常數**，
不是散在三個函式裡的 `0.10`——要把越障放進世界的人需要的是那個數字本身。

在 segment 的**進入**與**離開**各算一次。如果這條越障放得進它被分到的排程窗口，
兩次會一樣，四隻腳也會彼此一樣，那就有**一個**平台。
"""))

cells.append(code(r"""
from hybrid_note.scripts.experiments.day12_obstacle_registration_2d import (
    REGISTRATION_TOLERANCE_M, registration_report_2d, registration_rows,
    stance_on_top_seconds_2d,
)

# 與附錄 B 的圖同一組取樣（driver 用 241），數字才對得起來。
crossing_terrain = SharedTerrainSpec2D(
    height_m=0.04, top_length_m=0.40, x_start_m=1.00,
    obstacle_id="day12_platform")
crossing_run = plan_terrain_2d(crossing_terrain, tables10, samples=241,
                               reposition_unresolved=2)
registration = registration_report_2d(crossing_run.plan, crossing_run.body,
                                      crossing_terrain)
rB = registration.as_dict()

print(f"越障 segment 數                {rB['crossing_segments']}")
print(f"一個障礙物能同時符合它們嗎      {rB['is_registrable']}"
      f"   （容忍值 {rB['tolerance_mm']:.1f} mm，就是這條 pipeline 的接觸容忍值）")
print(f"隱含位置的散佈                 {rB['implied_spread_mm']:.1f} mm")
print(f"同一隻腳自己前後矛盾            {rB['worst_within_leg_gap_mm']:.1f} mm"
      f"   （它的 ascent 對它自己的 descent）")
print(f"單一 segment 內就漂移           {rB['worst_drift_mm']:.1f} mm")
print(f"最差的 body-advance 比值        {rB['worst_advance_ratio']:.1f}x"
      f"   <- segment 自己的座標系要 body 前進這麼多倍")
print(f"這個 run 規劃時用的平台在        {rB['planned_terrain_x_start_mm']:.0f} mm，"
      f"最近的隱含障礙物差 {rB['distance_to_planned_terrain_mm']:.0f} mm")

display(pd.DataFrame(registration_rows(registration))
        .query("row_kind == 'segment'")[
            ["leg", "kind", "start_s", "end_s", "implied_x_start_entry_mm",
             "implied_x_start_exit_mm", "drift_mm", "demanded_advance_mm",
             "delivered_advance_mm", "advance_ratio"]]
        .round(1).reset_index(drop=True))
"""))

cells.append(md(r"""
## B.3 結果：放不進去，而且不是差一點

八個越障 segment 隱含的障礙物位置散佈**超過一公尺**，同一隻腳的 ascent 與 descent
差 **436 mm**，而且**單一個 segment 內部**就漂移 276 mm。

漂移的來源是一句可以直接檢查的話：**這條 swing 的座標系假設 body 在它進行中前進
293 mm，排程只給了 17 mm**（17.3×）。

這正是 **Step 4 那個 `2.000x` 的距離版本**。Step 4 量的是時間放不下，
這裡量的是同一件事在空間上的樣子——而且距離版比時間版嚴重一個數量級。
"""))

cells.append(code(r"""
display(Image(filename=str(DAY12 / "day12_obstacle_registration.png")))
"""))

cells.append(md(r"""
## B.4 註冊的過程順手抓到第二件事：**沒有任何一隻腳站上去過**

```text
在障礙物頂面【站立】的總時間 = 0.000 s
```

LF 的 `SWING_UP` 是 0.394–0.497 s，`SWING_DOWN` 是 **0.497**–0.600 s——
觸地與離地是**同一個瞬間**。兩者之間那 160 mm 的頂面移動
（composition frame 裡 260 mm 落地、420 mm 起跳），
就是 **Step 7 沒有解出來的 `TOP_REPOSITION`**。

所以這條軌跡「越過」障礙物的方式是：**跳上去、在同一瞬間跳下來**。
這不是動畫的取樣不夠密，是排程裡真的沒有那一段。
"""))

cells.append(code(r"""
print(f"stance on top   {stance_on_top_seconds_2d(crossing_run.plan):.3f} s")
print()
lf = LEG_ORDER[0]
for sch in crossing_run.plan.schedule.segments_of(lf):
    seg = crossing_run.plan.plans[lf].phased[sch.segment_index].segment
    print(f"  {lf.value} {seg.kind.value:16s} {sch.mode.value:8s} "
          f"t {sch.start_s:6.3f}-{sch.end_s:6.3f} s   "
          f"接觸高度 {seg.start_contact.point_world_xz_m[1]*1e3:5.1f} -> "
          f"{seg.end_contact.point_world_xz_m[1]*1e3:5.1f} mm")
print("\n-> SWING_UP 的終點與 SWING_DOWN 的起點都在 +40 mm，且時間相同。"
      "\n   中間應該有的那一段是 Step 7 的 TOP_REPOSITION。")
"""))

cells.append(md(r"""
## B.5 所以圖上**沒有平台**，而那是唯一誠實的畫法

- 畫在 1000 mm（這個 run 規劃時用的平台）：腳根本不會去那裡，差 349 mm；
- 畫在任何一個隱含位置：另外七個 segment 不同意，差最多 1045 mm。

所以畫出來的是**每一條 swing 自己宣稱的那個面**——紫色虛線的 `+40 mm`，
**下面什麼都沒有**。那條線是整張圖上唯一還記得有障礙物存在的東西。

其餘讀法與附錄 A 相同（body 高度是畫圖挑的、紅色是被那個高度留在空中的腳）。
"""))

cells.append(code(r"""
display(Image(filename=str(DAY12 / "day12_obstacle_frames.png")))
"""))

cells.append(code(r"""
display(Image(filename=str(DAY12 / "day12_obstacle_animation.gif")))
"""))

cells.append(md(r"""
## B.6 這一節交出去的三件事（都是 Day 13–14 的待辦，不是結論）

```text
1. 越障 swing 的 body-advance 需求要變成一個【排程約束】
   現在是 17.3x：段內要 293 mm，窗口給 17 mm。
   Step 4 的 2.000x 是同一個問題的時間側；兩個要一起解，不是各自放寬。

2. 需要頂面那一段（Step 7 的 TOP_REPOSITION）
   沒有它，「越障」是兩個【瞬間相接】的 swing，中間 160 mm 沒有人規劃。

3. 世界 x 註冊要變成 pipeline 的一部分
   terrain 現在只走到 decide / compose 為止。要讓四隻腳越過【同一個】障礙物，
   body_x 與越障段的座標系必須共用原點——這是一個建模決定，
   跟 Step 3 決定時間軸是同一種性質的決定。
```

重畫（約 70 秒，含 GIF；地形是參數，不是寫死的）：

```bash
python3 -u LegWheel/hybrid_note/scripts/experiments/day12_obstacle_animation_driver.py
python3 -u .../day12_obstacle_animation_driver.py --height-mm 100 --top-length-mm 400
```
"""))

cells.append(code(r"""
import subprocess
proc = subprocess.run(
    ["python3", "-m", "pytest",
     "tests/test_day12_whole_body_animation_2d.py",
     "tests/test_day12_obstacle_registration_2d.py",
     "tests/test_day10_11_composer_2d.py", "-q"],
    cwd=ROOT, capture_output=True, text=True,
    env={**os.environ, "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1"},
)
print(proc.stdout.strip().splitlines()[-1])
"""))

nb.cells.extend(cells)
nbf.write(nb, NB)
print(f"wrote {NB}  ({len(nb.cells)} cells, {len(cells)} appended)")
