# Day 13 交接文件：全機越障規劃的現況與下一步

**寫給下一個對話（含 Fable）。** 讀完這一份就能接手，不需要回頭翻 2683 行的 archive。
最後更新：2026-09-07。

---

> **2026-09-08 更新：全機越障規劃已由 Day 14 接手。** §4 的 composer 接法與 §6 的下一步已被
> `hybrid_note/notes/day14/day14_plan_zh_TW.md`（計畫）與 `day14_implementation_log_zh_TW.md`（逐步紀錄）取代：
> 車體時鐘優先的事件迴圈（`day14_whole_body_planner_2d.py`）、地形規則（`day14_leg_terrain_rule_2d.py`）、
> 五種空中轉換一個生成器（`day14_nominal_transitions_2d.py`）。40 mm × 400 mm 四腳越障已通過、輸出
> `day14_step3_40mm_hardware.csv`（峰值 95.3%）。本文件 §1–§3、§5 的原則與坑仍然有效。

## 0. 三十秒版本

| 項目                     | 狀態                                 |
| ---------------------- | ---------------------------------- |
| 單腳用「平地揮法」爬上障礙物         | **完成，已量測、已測試**                     |
| 40 / 100 / 190 mm 三個高度 | **三個都能上去**（高的要更長的揮動距離）             |
| 接進 Day 10–11 composer  | **能跑了**，但 `frame_rows` 還沒接（見 §4.1） |
| 四腳一起走的越障 CSV           | **還沒有**。這是最後一哩，也是風險最大的一塊           |
| Day 12 凍結數字            | **沒有被動到**（44 passed）               |

**現在最重要的一句話**：單腳可行已經證明了，全機協調還沒有。
不要假設「單腳可以 × 4 = 全機可以」——上一輪就是這樣失敗的（§5.2）。

---

## 1. 專案的工作原則（請務必遵守）

這些是專案擁有者立的規矩，不是建議：

1. **量測優先於推論。** 一個「不可行」的結論，要先排除是自己問錯。
2. **資料引用**：`corgi_ros2_ws-dev` 是以前機器人的資料，**可以參考但不要直接引用**。
   可以引用的只有 `LegWheel` 這包裡面的東西。
3. **一定要留紀錄**：之前的所有工作記進 `day13_implementation_log_zh_TW.md`，你可以新開一個`day13_wholebody_implementation_log_zh_TW.md`來紀錄這這次的。
4. **給擁有者的總結用中文**，中間過程英文沒關係。產出的 notebook 要中文。
5. **不要刻意放寬約束**去讓結果變好看。重要的是符合機器人的真實限制。
   但**產生錯誤指令的 bug 不可放寬**。

### 測試指令

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/ -q \
  --ignore=tests/test_plotly_csv_viewer.py \
  --ignore=tests/test_plotly_robot.py \
  --ignore=tests/test_multi_rim_hybrid_gait_2d.py \
  --ignore=tests/test_single_leg_hybrid_gait_2d.py
```

基準：**1153 passed / 9 failed**（那 9 個與 hybrid_note 無關）。全套約 1 小時 50 分。

### 時間成本（一定要背景跑，不要卡住對話）

| 動作 | 時間 |
|---|---|
| 單腳 roll | ~20 秒 |
| 四腳越障組裝 | ~8 分 |
| `plan_terrain_2d` 每個地形 | ~8 分 |
| `test_day12_nominal_cycle_2d.py` | ~14 分 |

背景跑法：`setsid python3 x.py > x.log 2>&1 < /dev/null & disown`
（**不要用 `nohup &`**，會被殺掉。跑完要確認 process 真的還活著。）

---

## 2. 這台機器人的物理限制（模型 vs 真實）

| 量                                    | 值                                          | 這是誰的限制                              |
| ------------------------------------ | ------------------------------------------ | ----------------------------------- |
| 兩個馬達 `phi_r = θ̇+β̇`、`phi_l = β̇−θ̇` | 各 ≤ 1980 deg/s                             | **真實**（330 rpm 關節側，無減速比，擁有者確認）      |
| theta 關節範圍                           | 17°–160°                                   | **真實**                              |
| 腿掛載點                                 | LF/RF x=+255 mm、LH/RH x=−255 mm（軸距 510 mm） | **真實**                              |
| CoM                                  | 幾何中心                                       | **真實**（擁有者量的；repo 本身沒有質量模型）         |
| `body_rpy` 固定                        | **已刪除**                                    | 曾經是模型假設，擁有者說不合理                     |
| 車體俯仰上限 25°                           | `arctan(h/510)` 推出來                        | 模型；40mm→4.49°、140→15.35°、200→21.41° |
| `gamma` = 0（ABAD）                    | Day 13 的前提就是「不用 ABAD 能解的」                  | 模型                                  |
| margin floor 3 mm                    | **建立在一個猜測上**（±2mm CoM × 1.412 mm/mm）       | 模型，且不可靠                             |
| 準靜態、2D 矢狀面                           |                                            | 模型                                  |
目前的目標是走對稱的障礙就好，所以應該部會需要用到abad的自由度，之後對稱障礙誤會需要abad應該是為了讓機身更平穩

---

## 3. 已完成：B5 單腳用平地揮法上障礙物

### 3.1 擁有者要的是什麼

> 「我想要換成這種 nominal locomotion 也就是走平地的那種揮法，
>   不過 day8-9 不會全棄，因為你還是會需要知道初始位置跟末位置」

原本的 `SWING_UP` 是 Day 8–9 的 **Cartesian swing**：theta 60→54.2→41.0→36.6→48.1→58.7，
**先縮再伸**，所以通過障礙物前緣時腿接近全長，容易撞到。

平地的 `RECOVERY_SWING` 是：**縮到 17° → 順著同一個前向旋轉 → 伸開落地**，
整個旋轉都在最短姿態。

### 3.2 新模組

`hybrid_note/scripts/experiments/day13_b5_nominal_ascent_2d.py`

```python
run_nominal_ascent_2d(
    spec, posture, config,
    approach_hip_x_m = ...,   # Day 10-11 composer 給的起點
    landing_hip_x_m  = ...,   # Day 10-11 composer 給的終點
    beta_takeoff_rad = ..., beta_landing_rad = ...,
) -> NominalAscent2D

segment_from_nominal_ascent_2d(ascent, ...) -> MotionSegment2D
```

分工完全照擁有者說的：**Day 8–9 / Day 10–11 決定在哪起、在哪落；
`run_recovery_swing_2d` 決定怎麼飛過去。兩個既有的東西接起來，不是寫第三個。**

### 3.3 量到的結果

```text
起飛  foot_rim  ground                x 264.2   theta 72.49
落地  foot_rim  day6_7_obstacle_top   (430.7, 40.000)   theta 60.00
      125 frames
```

落地面的 id 是 **`day6_7_obstacle_top` 本身**，不是「高度剛好對」的地面。

三個高度都能上去（`scratchpad/b5_run2.log`）：

```text
h= 40 mm   face  40mm   hip advance  34.5mm ( 60% of nominal)   landed ( 81.8,  40.0)
h=100 mm   face 120mm   hip advance  74.5mm (130% of nominal)   landed (121.8, 100.0)
h=190 mm   face 140mm   hip advance  94.5mm (164% of nominal)   landed (141.8, 190.0)
```

**關鍵洞見：高的障礙物需要的是更長的揮動距離，不是更好的障礙物位置。**
190 mm 要 164% 的標稱揮動距離。這也解釋了擁有者的直覺
「至少 140mm 要用 roll 比較不會翻倒」——標稱揮法在高障礙物上要伸得比平地遠得多。

### 3.4 測試

`tests/test_day13_b5_nominal_ascent_2d.py`，**5 passed**，釘住：

1. 可站立面 = 地面 + 任何 `_top`，且不含垂直面
2. 障礙物必須經由 stroke 進入（回歸測試，見 §5.1）
3. **落地 z == 頂面 z**，且落在頂面範圍之內 ← B5 的主張本身
4. 旋轉全程維持 compact（換揮法的理由）
5. 失敗時仍保留「它被拒絕的是哪個問題」

回歸：`tests/test_day12_nominal_cycle_2d.py` **44 passed in 857s**，凍結數字未動。

---

## 4. 進行到一半：接進 composer

### 4.1 現況

`day10_11_composer_2d.py` 的 `compose_swing_swing_2d()` 加了 `nominal_ascent: bool = False`。
**預設 False，所以每一個既有結果都原封不動。**

驗證過兩條路都能組出來：

```text
nominal_ascent=False: OK  up_segment SWING_UP end (260.0,40.0) on day10_11_obstacle_top  samples  31
nominal_ascent=True : OK  up_segment SWING_UP end (307.3,40.0) on day6_7_obstacle_top    samples 125
```

### 4.2 【未完成】三件事

**(a) `frame_rows` 還沒接。** 這是最重要的一項。
`swing_frame_rows()` 讀的是 `SwingPlan2D`，而標稱 ascent 沒有那個東西，
它的資料在 `up_ascent.swing.frames`。
目前程式碼在 `nominal_ascent=True` 時**只帶下降段的 rows**，並在註解裡標明 UNFINISHED。
**匯出器需要這些 rows**，所以不接完就產不出 CSV。
程式碼裡搜 `UNFINISHED (Day 13 B5)` 就能找到位置。

**(b) 兩個場景的障礙物 id 不一樣**：`day6_7_obstacle_top` vs `day10_11_obstacle_top`。
兩條路徑組出來的 surface_id 不同，接縫處要確認下游（`_standable_surface_ids`
是用 `_top` 結尾判斷的，所以目前兩個都收得到，但**這件事還沒有被測試釘住**）。

**(c) 上層 dispatcher 還沒傳這個旗標。**
`day10_11_composer_2d.py:900` 附近呼叫 `compose_swing_swing_2d` 的地方
還沒有把 `nominal_ascent` 傳下去，`plan_terrain_2d` 也還沒有這個參數。

### 4.3 循環 import 的坑（已解，但要知道）

`day12_transition_mapping_2d` 會 import 本模組的 `ComposedSequence2D`，
所以**在 `day10_11_composer_2d.py` 的模組層 import 任何 day12 的東西都會造成循環**。
現在的做法是把那幾個 import 放進函式裡（lazy import），並在註解說明原因。
**不要「順手整理」把它們搬回檔案開頭。**

---

## 5. 這一輪踩過的坑（不要重踩）

### 5.1 `run_recovery_swing_2d` 的姿態是從 stroke 拿的

```python
posture = stroke.posture   # day12_nominal_cycle_2d.py:996
```

**沒有 posture 參數。** 要讓揮動落在障礙物頂面，障礙物必須在**建 stroke 用的那個 posture** 上：
`standing_stroke_2d(post, ...)`。

先前三次探測（x5/x6/x7）把障礙物只放在 swing 的 posture 上，函式從頭到尾沒看到它。
6/6 的 `TOUCHDOWN_IS_NOT_A_VALID_GROUND_CONTACT` 意思是
**「這個世界裡沒有障礙物」**，不是「腿落不上去」。

更糟的是 x7 有一個「成功」的對照組——它成功落在 **z = 0.0 的平地**。
**一個成功的結果，如果它成功的對象不是我問的那件事，它就是失敗。**

### 5.2 「腿站在地面上」這個假設寫死在三個地方

`surface_ids=(ground_surface_id,)` 在 `day12_nominal_cycle_2d.py` 出現三次：
落地檢查、`_stance_frame`、`standing_stroke_2d`。
只改了落地檢查 → 變成落地檢查放行、下一行卻 raise「沒有接觸」，自相矛盾。

已抽成單一 `_standable_surface_ids(scene)`（地面 + 任何 `_top` 結尾，**不含垂直面**）。
用後綴判斷是因為 `obstacle_top_surface_id` 這個屬性**不存在**（我猜的名字）。

**刻意保留地面限制的兩處**：滾動中追蹤接觸的兩個呼叫點（約 736 / 831 行）。
滾上頂面是另一種主張，而且 Day 12 凍結數字建立在那個限制上。

### 5.3 兩端的站立高度本來就不該一樣

```text
起飛端  theta 72.49（hold_hip_z_m 調變出來的）-> 髖 219.4 mm
落地端  theta 60.00（下一段要接的標稱姿態）    -> 髖 202.2 + h
```

`hip_z_for_flat_stance` 回答的是「固定 theta、髖隨輪弧上下」的姿態；
Hybrid posture 用 theta 調變把髖壓平，站得比它**高 17.2 mm**。
取成同一個，腿不是插進地裡（`APPROACH_POSE_REFUSED`）
就是構不到頂面（`TOUCHDOWN_IS_NOT_A_VALID_GROUND_CONTACT`）。
**起點和終點是不同的姿態——從一個姿態飛到另一個正是揮動的工作。**

### 5.4 掃描窗貼邊，兩次都誤判成「高度上限」

第一次：h≥100 全部失敗，看起來像高度極限。實際上可行的 face 位置
**就在掃描上限外面 3 mm**。

第二次（修好窗之後）：h=140/190 仍然「none」。實際上那個掃描
**把揮動距離釘死在標稱值**，而高障礙物需要 164% 的揮動距離。

**教訓**：一個失敗如果**與你正在變動的那個變數無關**（每個值都失敗），
那就不是那個變數的極限，是你問錯了。
現在掃描程式會自己報告「可行帶是否碰到掃描邊界」。

### 5.5 已經量測過、確定失敗、不要再試的東西

| 東西 | 結果 |
|---|---|
| `whole_body_schedule_2d` 後處理排程 | **更糟**：spread 0→122.5 mm、airborne 75→93、duration 9.8→16.3 s |
| `crossing_stagger_m` | 會把越障段整個歪掉，未採用 |
| 三種 per-leg 重新計時 | 全部失敗，`delay_overlapping_swings_2d` 現在直接 raise 並附上四個量測 |
| `MAX_BRACKET_GAP_M` | 誤診；180 和 200 直接量都是同樣的 182 次失敗，gap 寬度無法區分 |
| `np.repeat`（「不做內插」） | 產生 344 個 10811 deg/s 的尖峰（階梯邊緣） |
| 暫停做成 hold-then-jump | 單一 51908 deg/s 的跳變；改用 cosine ease |

**§5.5 最後一句話**：擁有者說「以全機為考量」是對的方向，
**但實現它不能只是在既有排程之後平移時間**——
時間就是腿對車體位置的說法，改時間就是改說法。
必須讓四腳的軌跡在**生成階段**就互相知道。那是重寫，不是後處理。

---

## 6. 下一步（建議順序）

1. **接完 `frame_rows`**（§4.2a）。不接完產不出 CSV。預估 40 分。
2. **把旗標傳到 dispatcher 和 `plan_terrain_2d`**（§4.2c）。預估 20 分。
3. **產生單腳三個高度的越障 CSV**，先讓擁有者能跑模擬。預估 1 小時（含機器時間）。
4. **四腳協調**——風險最大的一塊。B5 目前只驗證過**單腳**。
   四腳一起跑時 `support_margin` / `at_most_one_airborne` 可能出現新的失敗。
   **如果卡住，先把單腳可行的證據和 CSV 交出去，把全機問題單獨列出來，不要拖著不給東西。**
5. roll / swing 成對 CSV 做比較（論文主軸：除了標稱運動之外，怎麼選擇上障礙物的方式）。

---

## 7. 檔案地圖

| 檔案 | 作用 |
|---|---|
| `day13_handoff_zh_TW.md` | **本檔**，交接用 |
| `day13_plan_zh_TW.md` | 計畫（201 行） |
| `day13_findings_zh_TW.md` | 發現（338 行） |
| `day13_README_zh_TW.md` | 導覽（162 行） |
| `day13_implementation_log_zh_TW.md` | **完整工作紀錄**，§34 是 B5 |
| `archive/day13_implementation_log_zh_TW.md` | 舊的長版（2683 行） |

程式碼：

| 檔案 | 作用 |
|---|---|
| `day13_b5_nominal_ascent_2d.py` | **B5 新模組** |
| `day12_nominal_cycle_2d.py` | 標稱步態 + `_standable_surface_ids` |
| `day10_11_composer_2d.py` | 策略組裝，`nominal_ascent` 旗標在這 |
| `day13_step5_splice_driver.py` | 拼接匯出（平地 + 暫停 + 越障） |
| `day13_hardware_export_2d.py` | 硬體 CSV 匯出 |
| `day12_terrain_generalization_2d.py` | `plan_terrain_2d` |

測試：`tests/test_day13_b5_nominal_ascent_2d.py`（5）、`tests/test_day12_nominal_cycle_2d.py`（44）
