# Day 10–11 實作紀錄與交接檔

> **這份檔案的用途**：記錄 Day 10–11 每一次工作階段實際做了什麼、目前停在哪裡、下一步該做什麼。
> **如果你是在新的對話中讀到這份檔案**，請先讀 §0，它會告訴你需要先讀哪幾份文件、目前的狀態、以及可以直接開始的下一個任務。
>
> 規格與研究論證在 `day10_11_roll_swing_selection_zh_TW.md`（以下簡稱「規格」）。這份只記錄**動作與狀態**，不重複論證。

---

# 0. 新對話快速上手

## 0.0 先讀這一段：結論用語（2026-08-30 定案）

**這份專案裡大量出現「被推翻 / 不可行」。那些全部是關於【目前這個單腿 2D planner】
的敘述，不是關於機器人的。** 引用任何 negative result 之前，先用下面四個標籤定位：

```text
COMPOSED                      目前單腿 planner 已能生成完整、collision-free 的 sequence
DIRECT_HANDOFF_INFEASIBLE     目前的兩個接觸 primitive 無法【直接連續交接】
                              —— 這是關於現有 primitive 集合的敘述
REQUIRES_MULTILEG_REPOSITION  可能需要其他腿支撐、本腿再次離地重新就位，【尚未驗證】
OUT_OF_ENVELOPE               策略本身沒問題，是【這塊地形】超出它的範圍或超出已掃範圍
                              （#1 頂面太短、#5 stride 太長都是這一類）
NOT_MEASURED                  沒人量過這一格。「沒人看過」不是一種「不行」
PHYSICALLY_INFEASIBLE         只有在具備 reach / joint limit / collision / 接觸與支撐
                              的證據時才能用。Day 10-11【沒有任何一格】夠格用它。
```

> 後兩個是實作時補的：第一版把前四個套到整張地圖，19 個拒絕**全部**變成
> `DIRECT_HANDOFF_INFEASIBLE`——但 `#1` 頂面太短**不是交接失敗**。
> **那是同一種過度推廣，只是方向相反。**

`#2` 與 `#3` 是 `DIRECT_HANDOFF_INFEASIBLE`（可能加 `REQUIRES_MULTILEG_REPOSITION`），
**不是** `PHYSICALLY_INFEASIBLE`。完整定義見規格 **§5.5**；
這次討論找到的新缺口 `TOP_REPOSITION` 見規格 **§5.6**。

**改任何文件之前，先讀規格 §12 開頭那八條固定原則。**

## 0.1 必讀順序

```text
0. 本檔 §0.0（結論用語）—— 不讀這段會把條件性結果寫成物理結論
1. day10_11_roll_swing_selection_zh_TW.md    Day 10–11 規格與研究論證
   §2 為什麼改、§5.5 用語、§5.6 TOP_REPOSITION、§7 Step 0–10、§12 八條原則
2. 本檔 §2「目前狀態」與 §3「下一步」
3. ../new_hybrid_gait_research_plan_zh_TW.md 的 Day 10–11 一節
4. 需要細節時才回頭讀 ../day6-7/hybrid_gait_day6_7_v4.md 與 ../day8-9/day8_9_cartesian_swing_planning_zh_TW.md
```

## 0.2 工作規則（使用者指定）

```text
可以引用 / import：  LegWheel/ 這包裡的東西（legwheel.* 與 hybrid_note.scripts.experiments.*）
只能參考、不可引用：  /home/chang/corgi_ws/icra hybrid/corgi_ros2_ws-dev/（以前機器人的資料）
                     可以看、可以參考做法，但【不可以 import，也不可以複製程式碼進來】
每個進度要展示在：    notes/hybrid_gait_day10_11_motion_selection_dashboard.ipynb
                     （固定這一份，不要另開新的 ipynb）
每個工作階段要：      更新本檔（day10_11_implementation_log_zh_TW.md）
                     目的：對話 memory 用完時，可以直接開新對話照著這份往下做
```

**驗證方式**（隨時可重跑）：

```bash
cd LegWheel
grep -rn "corgi_ros2_ws" hybrid_note/scripts/experiments/day10_11_*.py tests/test_day10_11_*.py
# 沒有輸出 = 沒有引用到舊機器人的東西
```

2026-08-30 實測：無輸出。day10-11 的所有模組只 import `legwheel.*` 與
`hybrid_note.scripts.experiments.*`，兩者都在 LegWheel/ 內。

## 0.3 環境

```text
執行位置   /home/chang/corgi_ws/icra hybrid
import 路徑  sys.path.insert(0, os.path.abspath('LegWheel'))
模組       hybrid_note.scripts.experiments.<module>
```

## 0.4 一句話現況

```text
Step 0-9 全部完成並驗證。【Day 10-11 已結束。】
交付物：day10_11_step9_body_requirements.csv —— 一個檔案，392 列，
        Day 12 的四腳 timing 可以只讀它（用只 import csv 的 reader 稽核過）。
下一個是規格 §7 的 Step 10 六個方向，那【已經不是】Day 10-11 的範圍。
Step 2b 與 Step 3 的結論都是 negative：
        #3 (SWING_UP + ROLL_DOWN) 撞 alpha = -40 deg 的 rim 接縫；
        #2 (ROLL_UP  + SWING_DOWN) 撞兩道牆 —— D 是 +40 deg 接縫（不可修）、
           E 是下降側自己的 theta >= 35 deg 下限（可修）。
        Step 5 的候選從五個降為【三個】：#1 / #4 / #5。
Step 3 另外量到：頂面長度是【雙向】約束（rolling 要下界、swing off 要上限）。
Step 4 給出了 §5.3 缺的跨型別排序規則，所以 roll 與 swing 【現在可以比了】。
        而且推翻了 §2.2 的「rolling 是零 body 讓步」：實際是 h + 14.2 mm。
        質心 story 成立但要加限定詞：h >= 120 mm roll 優 9-12%，h <= 100 mm 打平。
Step 5 decision map 完成（Figure D）。decide_2d(h, L_top) 是純函式。
        機制：L_top 不只選策略，還在 #1 內部選 theta_climb -> rolling 的代價是階梯。
        新量測：#5 SWING_OVER（先前沒有任何一步量過），h <= 80 mm 才可行。
        能力圖上有一個【洞】：h >= 100 mm 且 L_top < 205-235 mm，172/609 格全部過不去。
        §2.7 的單調性【被推翻】（但取決於一個沒被最佳化的 landing_distance）。
Step 6 segment schema 完成，三條完成標準全達成（無損表示 Step 10R 與一條 swing）。
        資料逼出四件 §5.4 沒寫的事，最重要的是 alpha_range 不夠用 ——
        corner pivot（alpha 與接觸點都釘死、只有 beta 掃）佔 72/299 幀。
        rolling 段的 duration_s 全是 None：Day 6-7 準靜態、從未指定時間。
Step 7 三條【還活著】的策略各生出一條完整、collision-free 的 sequence。
        規格點名的三對有兩對已被推翻，改以第一級輸出記錄理由。
        【這一步最大的價值是它抓到前面幾步的三個缺陷】——見 §1 的 session 紀錄。
        Step 5 的地圖已用修正後的頂面下界重跑（#1: 205-245 -> 215-245 mm）。
Step 8 五個 case 的正向與負向都成立，但【第三條完成標準無法達成】：
        沒有任何 case 的最佳解是混合的，因為兩個混合對都被推翻了。
        2x2 不是未被驗證，是【對角線之外被推翻】。
        而 case E 量出了代價：h=160 mm 時 roll_up 10/10 成功、roll_down 0/10，
        那個用不了的爬升對 body 只要 80.1 mm，被迫改用的 swing 要 220.0 mm。
Step 9 交接物完成，四條完成標準全部成立。以 x 為自變數（rolling 沒有時間，
        指定它是新的建模決定），已存在的 swing 時間照原樣帶過去。
        資料逼出兩件規格沒預期的事：
        (1) PINNED 不是 Step 2b 的特例，是【每一條 swing 的兩個端點】——
            站姿下 hip_z 與 theta 互相決定，±10 mm 對應 ±5.6 deg。
        (2) Step 6/7 的 segment 純量是 envelope 不是 timeline，#4 兩段各藏 80 mm；
            而 rolling 段【根本沒有】那個純量。
研究計畫已於 2026-08-30 更新：最終交付物是長路 + 多障礙 + 單一 CSV，
Step 6 的 schema 必須含 WHEEL_ROLL / POST_TOUCHDOWN_ROLL（見規劃 §10.5）。
```

---

# 1. 工作階段紀錄

## 2026-08-29 — 規格建立

**動作**

1. 重寫 `../new_hybrid_gait_research_plan_zh_TW.md` 的 Day 10–11 一節（原文保留在該節末）。
   原規則 `if rolling feasible: ROLL else SWING` 已被 Day 6–7 / 8–9 的數據推翻。
2. 修改同檔 §22 的 Hybrid Checkpoint 第三、四問（原本問「是否使用更少 swing」，沒有鑑別力）。
3. 新增規格檔 `day10_11_roll_swing_selection_zh_TW.md`。
4. 依使用者指出的缺口，把策略空間從 3 類改成 **2×2 + 1**（規格 §2.6–§2.8）。
5. 依使用者的地形疑慮，新增規格 §11「Scope 邊界與推廣路徑」。

**產出的關鍵判斷**（詳見規格）

```text
判準      feasibility -> concession（向 body trajectory 索取多少讓步）
共用軸    只放 terrain 幾何 (h, L_top)
方向      planner 輸出 body requirement，不消費 body trajectory
策略空間  {ROLL_UP, SWING_UP} x {ROLL_DOWN, SWING_DOWN} + SWING_OVER
```

**實測記錄（當場跑出來的）**

```text
swing_onto_step_2d(sample_count=31, arc_samples=61)
    140 mm  OK    hip +40 mm
    150 mm  FAIL  theta pinned at limit
    160 mm  OK    hip +60 mm, liftoff +30 mm
=> two-ceiling 搜尋是依序貪婪的，ceiling 2 的 repair 會打壞 ceiling 1 的選擇且不回頭。
   sweep 不可沿用這個 ladder（規格 §6.1）。
```

## 2026-08-30 — Step 0 實作

**動作**

1. 新增 `LegWheel/hybrid_note/scripts/experiments/day10_11_shared_scene_2d.py`。
2. 統一 approach 幣別為**前緣餘裕**，包裝既有 bisection，不複製。
3. 一般化 roll-exit → swing-start 的交接（原本只有讀 cache 的單一 case 版本）。
4. 量化交接誤差與 rim 幾何差。

**新增／修改的檔案**

```text
新增  LegWheel/hybrid_note/scripts/experiments/day10_11_shared_scene_2d.py
新增  LegWheel/hybrid_note/scripts/experiments/day10_11_step0_driver.py
新增  notes/hybrid_gait_day10_11_motion_selection_dashboard.ipynb
新增  notes/day10-11/day10_11_step0_scene_alignment.csv
新增  notes/day10-11/day10_11_step0_roll_exit_handoff.csv
新增  notes/day10-11/day10_11_implementation_log_zh_TW.md（本檔）
修改  notes/day10-11/day10_11_roll_swing_selection_zh_TW.md（新增 §11 scope）
```

**Step 0 的四個決定**

```text
1. approach 幣別 = 前緣餘裕 c，不是 hip x
   理由：腿的等效半徑隨 theta 變；而且 §26.5(6) 的失敗機制本來就是餘裕問題
   實作：包裝既有的 _hip_x_for_start_clearance，不複製

2. 障礙幾何統一到 rolling 那組 (x_start = 0.10, width = L_top)
   理由：rolling 側的 map 已跑完，重新表達 swing 側比重跑 rolling 側便宜

3. SharedTerrainSpec2D.arc_samples 預設 121，對齊 SweepSettings2D
   注意：這【不是】swing 側的 leg_arc_samples (241)。後者是碰撞檢查的腿部取樣，
         前者是 rim 的接觸取樣，兩者不衝突
   注意：務必配 seam_bridge_for_sampling_m(arc_samples)。121 對應 17.5 mm，
         不是預設的 5 mm；用預設會偽否決 right->left handover

4. 交接誤差【量測但不阻擋】
   一般化版的 roll_exit_swing_start_2d 不像 cache 版那樣在分歧時丟例外，
   因為 Day 10-11 需要那個分歧當數據
```

**量測結果**

```text
standing alignment（80 cells：4 heights x 5 thetas x 4 clearances）
    hip_z 兩條路徑的差 = 恰好 1 nm 的 surface_offset，80/80 全部成立
    實現的前緣餘裕誤差 < 1 nm（bisection 收斂到底）
    站姿接觸永遠是 foot_rim @ alpha = 0 -> rim 幾何差為 0

roll-exit -> swing-start 交接
    在接觸管線自己的幾何裡是【精確的】：contact drift = 0.0 mm、alpha 差 = 0.0 deg、
    rim 標籤相符
    對物理腿模型的偏差 = 1.2 mm，且【兩個交接點都非零】
        ROLL_UP 出口          -> right_rim
        WHEEL_TRANSITION 出口 -> left_rim
    兩個都是 upper tyre，正好是 LegModel.rim_point (0.145 m) 與繪圖弧 (0.1438 m)
    差 1.2 mm 的地方
```

**這修正了規格 §4.3(b) 的說法**

原本寫「1.2 mm 會在交接點第一次真的咬到」。實測顯示更精確的說法是：

```text
它【不會】污染 roll -> swing 的交接      交接在接觸管線內是自洽的
它污染的是【往物理模型的映射】            兩個交接點都落在 upper rim
=> 不是 Day 10-11 的阻礙，是 Day 12 產生真實 joint command 時才必須處理的事
```

**順帶量到的事**

```text
完整 traversal 很慢：Day 6-7 sweep 自己的 runtime_s 中位數 120 s、最長 302 s
（arc_samples = 121）。本次交接量測落在同一範圍（單一 cell 254.6 s）。
=> 任何牽涉完整 traversal 的 sweep 都必須平行化並先估總時間；Step 4 會再踩到。
```

**Step 0 完整 sweep 的最終結果**（2026-08-30 重跑，前一次 session 中斷）

```text
standing alignment   80 cells：4 heights x 5 thetas x 4 clearances
    80/80 兩條路徑的 hip_z 差 = 恰好 1 nm 的 surface_offset
    實現的前緣餘裕誤差 < 1 nm

roll-exit handoff    20 traversals x 3 stages = 60 個 stage 出口
    52/60 抵達（8 個是 cell 在該 stage 之前就失敗）
    rim 標籤相符 52/52、contact drift 0.0 mm、alpha 差 0.0 deg
    rim 幾何差：left_rim 23 個、right_rim 28 個 全部 1.2 mm；foot_rim 1 個為 0
    traversal 中位數 264 s，15 workers 平行 445.8 s 跑完
```

**額外收穫：Step 0 順帶逐格重現了 Day 6–7 的 feasibility map**

交接量測必須跑完整 traversal，所以每個 cell 順帶產生一個可行性判定。拿去和
`day6_7_step11r_feasibility_sweep.csv` 對照：

```text
20 / 20 格完全相同，包含兩處非單調：
    h = 120 mm   在 40/50/60/70/85 deg 全部不可行
                 （Day 6-7 完整掃描顯示只有 45 與 55 deg 可行，50 deg 是洞）
    h = 140 mm   40 與 70 deg 可行，50/60/85 deg 不可行
```

**這比 1 nm 的姿態對齊更有說服力**：對齊只證明兩邊「建出同一個姿態」，
這證明兩邊「跑出同一個結論」，連地圖上的洞都對得起來。

**另一個結構性發現**

```text
WHEEL_TRANSITION 的出口停在哪個 rim，與整條 traversal 可不可行【完全相關】
    left_rim  -> 可行  12/12
    right_rim -> 不可行 8/8
```

這正是 Step 8R 的設計（`LEFT_RIM_READY` 是抵達後緣時的前提條件）。
對 Day 10–11 的意義：**這一格就是策略 #3（SWING_UP + ROLL_DOWN）要繞過的東西**——
若 swing 能直接落進已滿足 `LEFT_RIM_READY` 的狀態，整段 rim handover 與它花掉的
`L_transition` 都不必付。這是把 Step 2b 提前的理由。

## 2026-08-30 — Step 1 實作

**動作**

1. 新增 `day10_11_concession_2d.py`（契約）與 `day10_11_step1_driver.py`（驗證）。
2. 新增 `tests/test_day10_11_concession_2d.py`，20 個不變量測試，全過。
3. 重建 notebook，納入 Step 0 + Step 1。

**設計決定**

```text
1. 「無損」用【重建】而不是【解析】來證明
   parser 只證明字串讀得出來；從欄位重建字串才證明欄位足以產生它們，
   也就是沒有資訊只活在散文裡
   -> adjustment_strings_from_concession()

2. ConcessionSource 記錄數字【怎麼來的】
   GREEDY_LADDER 答的是「搜尋停在哪」，GRID_MINIMUM 答的是「body 最少要給多少」
   只有後者可以跨 cell 比較（is_comparable）

3. BindingCeiling 命名的是【到達了哪個階段】，不是原因
   原因在 failure 欄。兩者的組合才是診斷 —— 見下

4. compare_concessions 拒絕跨 kind 比較
   把 hip 下界和 hip 軌跡按大小排序 = 預設了規格 §5.3 的答案。
   要 Step 4 先量出哪種苛刻，才允許 allow_cross_kind=True

5. BindingCeiling 多了一個 STANCE
   Day 8-9 的「兩道天花板」故事沒涵蓋：腿在端點根本站不住，連軌跡都沒嘗試。
   它要的修正不同（approach / landing 距離，不是 hip 高度或控制點）
```

**量測結果：無損 16/16**

```text
onto   20 /  60 / 100 mm   免費
onto  120 / 140 / 160 mm   hip +20 / +40 / +60 mm
onto  150 mm               失敗（貪婪假洞）
onto  200 mm               失敗 TERRAIN_COLLISION

off    20 /  60 / 100 mm   免費
off   120 / 140 mm         hold 0.25（30 / 35 mm）
off   150 / 160 mm         hold 0.50（75 / 80 mm）
off   200 mm               失敗 TOUCHDOWN_VELOCITY_TOO_HIGH
```

下降側是**新資料**（Day 8-9 §28.1 只做了 climb），而且下降側**單調、沒有假洞**。
`off 160 mm` 要 hold 0.5，獨立重現了 `swing_off_step_2d` docstring 記的那句量測
（hip 跟著掉會撞，hold 一半有約 1.8 mm 餘裕）。

**兩個由數據逼出來的額外欄位**

```text
greedy_backtrack_suspected
    簽名 = binding_ceiling FIT + reach 類的 failure
    抵達 fit 階段代表 reach 探針通過；在那裡以 reach 類原因失敗
    = repair 把 reach 弄壞了，而流程不回頭
    實測只標記到 onto 150 mm（IK_NOT_CONVERGED）
    onto 200 (TERRAIN_COLLISION) 與 off 200 (TOUCHDOWN_VELOCITY_TOO_HIGH) 不被誤標
    -> Step 2 可以直接把「沒有任何 cell 帶這個旗標」當驗收條件

contiguous_span_around_best_deg
    由 §0.6 的 h = 120 mm 逼出來：可行 theta 窗口是【梳狀】的，不是區間。
    max - min 會把它報成 10 deg，但區間中間的 50 deg 是洞。
    Step 5 若用「窗口較寬」當 tie-break 會偏好梳子 —— 與 robust 相反。
    -> 同時報 theta_window_is_contiguous 與 contiguous_span_around_best_deg
```

## 2026-08-30 — Step 2 實作

**動作**

1. 新增 `day10_11_swing_sweep_2d.py`（格點搜尋）與 `day10_11_step2_driver.py`（sweep + 圖 + 驗收）。
2. 測試加到 21 個（新增 Step 2 契約測試：格點結果必須 `is_comparable` 且不可能帶貪婪簽名）。
3. 重建並執行 notebook，納入 Step 2。

**核心設計：為什麼這不是 Day 8–9 的那個搜尋**

```text
Day 8-9 貪婪 ladder   升階判準是【便宜的 _reachable 探針】
                      repair 事後抬 liftoff，可以把探針通過的那個 reach 打壞，且不回頭
Step 2 完整格點       升階判準是【plan.valid 本身】
                      在 (hip_lift x liftoff_rise) 升序走，第一個【真的 valid】的組合
                      就是 lexicographic 最小值 -> 是地形的性質，不是搜尋順序的性質
```

**規模**：88 cells、872 次 `generate_swing_2d`、708 s（15 workers）。83/88 可行。

**驗收結果**

```text
[成立] clearance 軸會翻轉 cell
       h = 150 / 160 / 180 mm 在 c = 20 mm 失敗、c = 40 mm 成功
       h = 200 mm 在 c = 20 / 40 mm 失敗、c = 60 mm 成功

[成立] 對高度單調，0 個洞 —— 150 mm 的貪婪假洞【沒有】在格點搜尋下重現
       高度天花板：c = 20 -> 140 mm；c = 40 -> 180 mm；c >= 60 -> 200 mm（掃描上限）

[成立] 200 mm：c >= 60 mm 可行；c = 20 / 40 mm 為 TERRAIN_COLLISION

[成立] 88/88 有 binding_ceiling；0 個貪婪簽名；全部 grid_minimum

[推翻] 轉折點不在輪半徑附近（見下）
```

**一個被數據推翻的規格預期**

規格 Step 2 的完成標準要求「轉折點附近 c 與輪半徑 0.145 m 的關係被明確寫出來」，
出處是 Day 8–9 §26.5(6) 提的機制。實測 `h = 150 mm`：

```text
c =  20 mm -> 接觸點到前緣 176 mm
c = 160 mm -> 接觸點到前緣 319 mm
整條軸上都 > 145 mm，但 c = 20 mm 仍然失敗
```

所以那個機制在本幾何下**從未成立**。真正的機制由「哪個旋鈕修得好它」指出來：

```text
修好小 c 的旋鈕是 liftoff_rise
    c = 20 mm   h >= 60 mm 每一格都需要 50 mm（ladder 頂端）
    c = 40 mm   隨高度 0 -> 10 -> 20 -> 30 -> 50 mm
    c >= 60 mm  全部 0 mm
liftoff_rise 是在 swing【起始】把腿垂直抬起的旋鈕
=> 問題是腿體在還貼著前緣時的【掃掠體積】，不是接觸點的靜態位置
```

**§26.5(6) 的「效應」成立且被量化了，但它提出的「機制」不成立。**
兩者必須分開講，否則 paper 會寫出一個數據不支持的因果。

**一個沒預期到、但對 Step 5 有直接影響的結構**

```text
clearance 軸在 c ≈ 60 mm【飽和】
    c = 60 … 160 mm 每一列的 min hip lift 完全相同
=> approach clearance 在 decision rule 裡應該當【門檻條件】，不是連續代價軸
```

門檻之上，swing 的上升 body 讓步只由高度決定——這一欄就是 Step 5 要拿去和 rolling 比的：

```text
h <= 100 mm   hip  0 mm
h =  120 mm   hip 20 mm
h =  140 mm   hip 40 mm
h =  150 mm   hip 40 mm
h =  160 mm   hip 60 mm
h =  180 mm   hip 60 mm
h =  200 mm   hip 80 mm
```

**但現在還不能比**：rolling 側的 body 要求是一條軌跡而不是一個下界（規格 §5.3），
`compare_concessions` 會拒絕跨 kind 比較。Step 4 必須先量出來。

## 2026-08-30 — Step 2 close-out

**動作**

1. 新增 `day10_11_step2_closeout_driver.py`：三段掃描，補上主 sweep 固定住但沒論證過的三個軸。
2. 三段的 clearance 一律設在飽和之後的 80 mm，所以變動的只有被測項。
3. 重建 notebook 的 §2.7 close-out 一節（含 theta 的絕對幣別換算表）。

**規模**：48 cells、961 次 `generate_swing_2d`、987 s。0 個貪婪簽名。

**三個結果，其中兩個修正了主 sweep 的結論**

```text
A. 天花板         240 mm，且是 reach 型（腿搆不到，不是放不下）
                  c =  60 mm -> 最高 200 mm
                  c = 100 mm -> 最高 240 mm
                  c = 160 mm -> 最高 240 mm
   => 「c ≈ 60 mm 飽和」只在 h <= 200 mm 成立。h = 220/240 門檻升到 100 mm。
      Step 5 要寫 c_threshold(h)，不能寫常數 60 mm。

B. landing        天花板以下完全無影響（0.10 … 0.19 每一列相同）
                  h = 200 mm（貼近 240 天花板）時 0.19 要多付 20 mm、0.22 失敗
   => 固定 landing = 0.16 m 可以，但理由是「天花板以下無所謂」，不是「無所謂」

C. theta          swing 最強的內部自由度，而且最佳 theta 隨高度上移
                  落地時 hip 距台面高度 [mm]
                      h \ theta   40     50     60     70     85
                      100      263.6  241.5  219.4  237.5  264.9   最佳 60
                      150      303.6  281.5  259.4  257.5  264.9   最佳 70
                      200          x  321.5  299.4  277.5  264.9   最佳 85
   => 主 sweep 的 map 是【上界】。h = 200 mm 高估 34.6 mm。
      規格 §3.1 自己寫了「每個 primitive 對內部自由度取最佳」，Step 2 沒做，Step 5 前必須補。
```

**注意這【不】推翻 §3.1 否決 sweep (ii) 的理由。** theta 該被「取最佳掉」，不是被「當共用軸」。
theta 對 swing 效應這麼大，反而更說明把它和 rolling 的 `theta_climb` 對齊會有多誤導。

## 2026-08-30 — Step 2b 實作

**動作**

1. 擴充契約：`BodyRequirementKind` 新增 `PINNED`，`SwingConcession2D` 新增
   `pinned_hip_above_surface_m`（見下方「被數據逼出來的契約修改」）。
2. 新增 `day10_11_left_rim_landing_2d.py`（Step 2b 的模組）與 `day10_11_step2b_driver.py`。
3. 新增 `tests/test_day10_11_left_rim_landing_2d.py`，14 個測試；全套 35 passed。

**被數據逼出來的契約修改：`PINNED`**

```text
Step 2 的上升用 min_hip_lift 計價 —— 「hip 至少要比標稱站姿高多少」，是一個【下界】。
Step 2b 沒有這個旋鈕：
    把落地 contact state 釘死成 (point, left rim, alpha) 而且要求 theta = 17 deg，
    hip 就完全沒有自由度了 —— 17 度的腿幾乎就是一個圓，hip 必須【剛好】在台面上方
    一個輪半徑處（實測 143.80 mm，與 beta 無關）。
    把 hip 抬高不會讓落地變容易，只會讓它變成【另一個】落地（theta 更大），
    而那個落地過不了 LEFT_RIM_READY。
=> 需求形狀是【等式】不是【不等式】。BodyRequirementKind.PINNED。
   compare_concessions 現在也會拒絕 PINNED vs LOWER_BOUND 的排序。
   唯一剩下的旋鈕是 liftoff_rise。
```

**設計決定：precondition 用 import 的，不重寫**

```text
LEFT_RIM_READY 由 Day 6-7 自己的 _corner_readiness_failure 判定，
descent 由 Day 6-7 自己的 run_left_rim_roll_down_2d 執行。
做法是把 swing 落地姿態包成一個 RetractResetFrame2D，再包成 WheelModeTransitionResult2D
（branch_result 只放一個 _SingleFrameBranch —— Step 9R 只讀 final_frame，
  帶完整的 RetractResetBranchResult2D 會需要一次完整 rolling ascent，
  等於為了證明 swing 不需要 rolling 而先跑一次 rolling）。
理由與 Step 0 相同：swing 側再寫一份 readiness 判定，兩邊一定會漂。
```

**Section A 結果：(b) 與 (c) 都成立，而且有數字**

```text
(b) left rim 可達，而且窗口很寬
    theta = 17 deg 時 beta in [-312, -180] deg 使 left rim 承接接觸
    寬 132 deg、連續、【四個高度完全相同】-> 這是腿的性質，不是台階的性質
    rolling ascent 自己抵達的 beta in [-238.2, -212.7] 完全落在窗口內
    hip 被釘在台面上方 143.80 mm（每個 beta 都一樣）

(c) alpha 接縫不是硬阻擋，但它訂出策略 #3 的高度上限
    rim budget（接觸點往 -40 deg 方向剩下的弧）與 seam margin 是【1:1 互換】的
    落地選擇規則（無自由參數）：先買足 corner pivot 需要的 budget，再最大化 seam margin

    height | pivot 需要 | 選到的 beta | 落地 alpha | rim budget | seam margin
        60 |   54.1 deg |   -250 deg |   -110.0 deg |   70.0 deg |    70.0 deg
       100 |   71.9 deg |   -248 deg |   -112.3 deg |   72.3 deg |    67.7 deg
       140 |   88.0 deg |   -231 deg |   -128.7 deg |   88.7 deg |    51.3 deg
       160 |   95.9 deg |   -223 deg |   -136.8 deg |   96.8 deg |    43.2 deg
       200 |  112.3 deg |   -207 deg |   -153.2 deg |  113.2 deg |    26.8 deg
    => 台階越高，pivot 吃掉越多 rim，落地就被逼得越靠近 -180 deg 那道縫。
       rim 的總長 140 deg 訂出 pivot 的硬上限 -> 對應 h ≈ 256 mm。
```

**Section B / C / D 結果：(a) 不成立，而且四個先驗上合理的解釋都被排除**

```text
B  L_top = 0.35 m、5 高度 x 5 d_corner = 25 cells    swing 全部失敗
C  d_corner = 20 mm、5 高度 x 7 L_top  = 35 cells    swing 全部失敗
D  L_top = 0.12 m、5 approach theta x 3 clearance    全部 15 格失敗

[排除] 不是 top length     0.08 … 0.35 m 每一個都失敗
[排除] 不是 hip travel     最短只要 0.28 m；Step 2 成功走過 0.38 m
[排除] 不是落地姿態        35/35 全部通過 LEFT_RIM_READY
[排除] 不是 approach 姿態  theta 40…85 x c 60/100/160 mm 全部失敗
```

**第一道牆：`LEFT_RIM_READY` 要求腿處在自己的機械極限**

```text
legwheel/config: MIN_THETA_DEG = 17.0
=> theta = 17 deg 不是普通姿態，是這隻腿【能收到的最短狀態】
IK 原話：「theta is pinned at its limit, so the target is out of reach」

滾上去的腿可以到 17 度，因為它是【已經被頂面支撐著】慢慢收縮過去的；
swing 卻必須在【還懸在角落上空】時就已經在 17 度。
```

**先排除關節步長：那是排程假影**

```text
samples=31,  0.6s -> max_step 11.7/10 deg  IK_NOT_CONVERGED  ik_ok  11/31
samples=61,  0.6s -> max_step  5.8/10 deg  IK_NOT_CONVERGED  ik_ok  21/61
samples=61,  1.2s -> max_step  5.8/10 deg  IK_NOT_CONVERGED  ik_ok  21/61
samples=121, 1.8s -> max_step  2.8/10 deg  IK_NOT_CONVERGED  ik_ok 41/121
=> 違規消失、失敗不變，IK 收斂比例固定 1/3 -> 是路徑的幾何性質
```

**缺的是什麼：中段拱起、端點釘死的 hip 軌跡**

```text
兩端 hip 一起抬高（HipTrajectory2D 是直線，這是唯一能做的「拱起」）：
    +  0 mm  ik_ok 21/61  IK_NOT_CONVERGED
    + 20 mm  ik_ok 39/61  JOINT_DISCONTINUITY
    + 40 mm  ik_ok 61/61  JOINT_DISCONTINUITY   <- reach 問題完全消失
    + 60 mm  ik_ok 61/61  JOINT_DISCONTINUITY
=> 飛行途中 hip 差【約 40 mm】。但端點被釘死在 143.8 mm，
   一條直線做不到「中間高 40 mm、端點又剛好」。
   策略 #3 的 body 需求 = 端點釘死 + 中段拱高 40 mm，HipTrajectory2D 表達不出來。
   -> 這是 Day 12 的輸入，不是 Day 10-11 的失敗。
   -> 誠實界線：拱起被證明【必要】，沒有被證明【充分】。
```

**第二道牆：alpha = -40 deg 的接縫是真的不連續**

```text
hip +40 mm（IK 全收斂）之後只改取樣密度：
    samples= 61  max joint step 24.3 deg
    samples=121  max joint step 27.5 deg
    samples=241  max joint step 28.5 deg
    samples=481  max joint step 29.3 deg
=> 加密不讓它變小，收斂到約 29 deg -> 真的不連續
位置：sample 42  foot_rim alpha=-39.3  ->  left_rim alpha=-40.3，跳 23.6 deg
      （第二大的一步只有 2.7 deg）

射程比策略 #3 大得多：站姿接觸永遠在 foot_rim alpha=0（Step 0 的 80/80），
所以【任何】要落在 left rim 的 swing 都必須穿過這道縫。
```

**沒預期到：`LEFT_RIM_READY` 對「空中來的」抵達【不是充分條件】**

```text
35/35 落地姿態通過 LEFT_RIM_READY，但 14 個（h = 160 / 200 mm 全部）Step 9R 下不去
失敗：LEFT_RIM_ROLL_DOWN / NO_LEGAL_CORNER_PIVOT_CONTINUATION
而且不是 rim budget 不夠：h=140 多買 0.6 deg 成功、h=160 多買 0.9 deg 失敗
=> 一階的 arccos(1 - h/R) 不是真正的綁束條件
=> 下降側對這種抵達有自己的天花板，落在 140-160 mm 之間，與 swing 無關
=> 這是「把 Day 6-7 的 precondition import 來用而不是重寫」才會發現的事
```

**Section E（退化版本）：問題不在 17 度，在 left rim**

```text
--only degenerate：落地 theta ∈ {17, 25, 35, 45, 60} x 3 高度，L_top = 0.20 m
每個 theta 的 left-rim 窗口都存在、hip 一樣被釘死：
    17 deg  窗口 132 deg  hip 143.8 mm      45 deg  窗口 120 deg  hip 157.1 mm
    25 deg  窗口 126 deg  hip 147.5 mm      60 deg  窗口 110 deg  hip 164.8 mm
    35 deg  窗口 120 deg  hip 152.2 mm

swing 可行嗎：15/15 全部失敗，放寬到 60 度也一樣。
失敗模式混合 IK_NOT_CONVERGED 與 JOINT_DISCONTINUITY（60 度以後者為主）。
註：E 段失敗模式是混合的，不能全歸給接縫。接縫的乾淨證據是上面那個專門探針。
```

**Step 2b 結論**

```text
(b) 成立   left-rim 窗口 132 deg、連續、與高度無關
(c) 成立   rim budget 與 seam margin 1:1 互換，無自由參數的選擇規則
(a) 不成立 而且退化版本也不成立

swing planner 目前只能交出【foot rim】的落地
=> SWING_UP + ROLL_DOWN 仍需在頂面做 rim handover（foot->left 取代 right->left）
=> #3 並沒有躲掉 L_transition，只是換了個名字
=> §2.6 那個「swing 直接落進 LEFT_RIM_READY 跳過 L_transition」的論證【不成立】
=> Step 5 的候選先從五個降為四個（Step 3 推翻 #2 之後再降為三個）

歸屬清楚：擋住 #3 的是 rim 分段接縫這個【模型性質】，不是機器人做不到。
rim 之間的接觸連續化之後要重測這一格。
```

---

## 2026-08-30 — Step 3 實作（下降側 swing sweep）

**動作**

1. 新增 `day10_11_swing_off_sweep_2d.py`（約 700 行）：`SwingOffGridSettings2D`、
   `minimum_swing_off_concession_2d`、`RollUpExitPose2D` / `roll_up_exit_pose_2d`、
   `minimum_roll_up_descent_2d`、`run_*` 批次入口。
2. 新增 `day10_11_step3_driver.py`：A–F 六段，`--sections` 可續跑、`--plots-only` 可只重畫。
3. 新增 `tests/test_day10_11_swing_off_sweep_2d.py`，12 個測試；全套 47 passed。
4. 三段圖改成**依「發現」而非依「段」**組織（A 上限、B theta、C 兩道牆）。

**三個 ladder 的巢狀順序（跑之前就決定，並寫進規格）**

```text
for hip_hold:            <- 唯一的【body 讓步】，Step 5 要比的就是它 -> 放最外層
    for touchdown_drop:  <- 軌跡整形
        for duration:    <- 軌跡整形（下降獨有；上升側 18 個不可行 cell 沒有一個是時間問題）
```

hold 在最外層才能保證回報的 `min_hip_hold_fraction` 是真正的最小 body 讓步。

**結果**

```text
段  內容                                    cells  可行   關鍵數字
A   map: height x takeoff distance          99     74    takeoff 上限 0.24 -> 0.12 m
B   160 mm claim（兩種取樣 x 3 個 hold）      6      0    -1.63 mm / -0.95 mm（未跨 0）
C   theta 40-85（h = 100/150/200）           15     10    最佳 theta 隨高度上升
D   arrival = ROLL_UP 直接交接                45     0    right rim, alpha +82…+107.7 deg
E   退化：retract 到 theta = 17/25/30         30     0    全部 IK_NOT_CONVERGED
F   theta 32/35/38 x takeoff 0.08/0.12/0.16   9      6    下限 = 35 deg，與 takeoff 無關
```

**被數據逼出來的三件事**

```text
(1) 頂面長度是【雙向】約束
    rolling   要 L_top  >= 196-249 mm （L_transition，下界）
    swing off 要 takeoff <= 240 mm (h<=100) … <= 120 mm (h=200)（上限，隨 h 收縮）
    => Step 5 的第一個判準是頂面長度的【方向】，不是代價比較。
       短頂面只有 swing 下得去；長頂面兩者都行、才輪到比 body 讓步。

(2) planner 會【拒答】而不是回報不可行
    generate_swing_2d 在腳最後停在空中（final_rim=None）時丟 ValueError。
    A 段有 153 次落在 11 格。記成 planner_refusals，【不改 planner】——
    那支 planner 產出了 Step 2 / 2b 的全部結果，不能在 Step 3 中途換掉。

(3) D 和 E 是【兩道不同的牆】，不要合併成一句「#2 不可行」
    D  binding = fit    JOINT_DISCONTINUITY 26 + IK_NOT_CONVERGED 19
       -> 起飛在 right rim alpha +82…+107.7 deg，要跨 +40 deg 接縫。
          Step 2b 的鏡像，同樣是模型性質，【不可修】。
    E  binding = reach  IK_NOT_CONVERGED 30，每一個 takeoff 距離都失敗
       -> 撞的是下降側自己的 theta >= 35 deg 下限（F 段夾出來）。
          Day 6-7 Step 6.5 把 retract 的 theta_target 寫死成 17 deg，
          因為它的目的是【進 wheel mode】給 ROLL_DOWN 用。
          SWING_DOWN 不需要 wheel mode，只需要接觸回 foot rim + theta >= 35 deg。
          【可修】，而且很便宜：
          L_transition = retract_forward (20.5-64.4) + wheel_mode_forward (131.8-228.4)
          停在 35 deg 就不用付後半 -> 頂面需求掉 3-10 倍。修好後 #2 要重測。
```

**B 段那句 160 mm 的處理方式**

不是「前人量錯」。兩種取樣（121/31 與 241/51）都跑一次，加密後穿透從 -1.63 收斂到
-0.95 mm 卻仍未跨 0，**所以不是取樣假影**。A 段給出歸屬：`h = 160 mm` 的 takeoff 上限是
**0.14 m**，而 docstring 是在 0.16 m 量的。**高度沒錯，距離錯了。**
同一批數據也推翻了 Day 8–9「off 200 mm 不可行」——takeoff <= 0.12 m 時可行。

**這次踩到的三個坑（已修）**

```text
1. 第一次跑在 D 段崩掉：ValueError「touchdown contact state 未驗證」。
   根因是 (2)。修法是在 _search_body_and_repairs 內接住並計為 refusal。
2. 第二次跑在寫 B 段 CSV 時崩掉：dict 有 fieldnames 沒有的欄位。
   write_rows_csv 用第一列的 keys 當表頭；checkpoint() 改成取所有列的欄位聯集。
3. B 段用 next(r for r in rows if r["min_hip_hold_fraction"] == 0.0) 取列，
   對不可行的 cell 會 StopIteration（該欄是 None）。改成先建 by_hold dict。
```

---

## 2026-08-30 — Step 4 實作（rolling map 換幣別 + 質心 story 的關鍵實驗）

**動作**

1. 新增 `day10_11_roll_concession_2d.py`（約 660 行）：`HipExcursion2D`、`hip_excursion_2d`、
   `RollTrajectory2D` + `load_roll_trajectories_2d`、`roll_concessions_by_height_2d`、
   `SwingHipProfile2D` + `swing_onto/off_hip_profile_2d`、`swing_obstacle_hip_profile_2d`、
   `BodyDemandComparison2D`、`compare_body_demand_2d`。
2. 新增 `day10_11_step4_driver.py`：A–G 七段，`--plots-only` 可只重畫。
3. 新增 `tests/test_day10_11_roll_concession_2d.py`，20 個測試；全套 67 passed。

**最重要的實作決定：沒有重跑任何 traversal**

```text
day6_7_step11r_sweep_trajectories.csv (3.8 MB) 已經記了全部 70 個 cell 的
hip_x_m / hip_z_m，含 stage、phase、accepted，且完整覆蓋 42 個可行 cell。
重跑要花約 264 s/cell（陷阱 5）去重算硬碟上已有的數字。

唯一是【推導】而非量測的是 L_top 依賴，而它由資料本身授權：
WHEEL_MODE_TOP_ROLL 與 LEFT_RIM_READY 兩段把 hip 高度保持在 0.027 mm 以內，
所以加長頂面只加平的前進距離。per_forward_at_top_length() 會在
(a) 頂面短於 Step 12R 實測下限、(b) 該 cell 的平頂其實不平 時【拒答】。
```

**前提檢查（沒有它整步無效）**

```text
Day 6-7 step11r      obstacle_width_m = 0.35,  x_start = 0.10
Step 2 (swing onto)  top_length_m     = 0.35,  x_start = 0.10
Step 3 (swing off)   top_length_m     = 0.35,  x_start = 0.10
=> 三個 sweep 是同一根柱子。沒有任何程式強制它，所以測試裡有一個專門盯著。
```

**結果**

```text
段  內容                                      關鍵數字
A   70 個 cell 逐格計價                        42/70 可行；平頂變化最大 0.027 mm
B   對 theta_climb 取最佳                      hip_z_travel = h + overhead(theta)
                                              theta=40 -> overhead 14.2-14.9 mm（每個高度都一樣）
                                              theta=85 -> 49.4 mm
C   matched stages（兩邊都不含頂面）            swing 每個高度都便宜，1.1x - 2.3x
D   whole obstacle                            swing 贏 h<=100（0.4-3.2%）
                                              roll  贏 h>=120（8.9 / 12.2%）
E   L_top 依賴（兩側都外推，閉式交叉點）         V_roll < V_swing 在每個高度都成立
                                              L* = 0.386 - 0.666 m (h<=100)；h>=120 已經贏了
F   §5.3 的跨型別規則                          h<=100 -> SWING；h>=120 -> ROLL
G   theta_climb 的取捨                         theta 40->85：起伏 +34.5~46.8 mm、頂面 -64.4 mm
```

**被數據逼出來的四件事**

```text
(1) 「rolling 是零 body 讓步」是錯的，而且錯得可以量化
    rolling 的 hip 起伏 = h + 14.2 mm。規格 §2.2 那句直覺必須從 paper 拿掉。
    真正的差別是【形狀】：rolling 是梯形（含平頂），swing 是三角形。

(2) C 與 D 結論相反，而且原因很具體
    matched stages 把頂面拿掉了，而【那正是 rolling 免費賺前進距離的地方】
    （wheel mode 平走，垂直位移 0）。拿掉它等於只留下 rolling 最貴的兩段。
    要回答 paper 的 story 必須用 D —— walking 也得走過頂面。

(3) 無因次指標對「誰能自選什麼」極度敏感（這是一個【踩到才發現】的坑）
    swing 可自選 approach clearance，較大的 clearance 在零垂直代價下加長前進距離、
    稀釋它自己的分母；rolling 的 clearance 被 Day 6-7 凍結在 40 mm。
    讓 swing 自選會在 h = 60 / 80 mm 【翻盤】。
    做法：主要結果用對齊的 c = 40 mm，自選版當敏感度，兩者都寫進 CSV 的 swing_variant。

(4) theta_climb 是取捨不是最佳化
    B 段的「取最小可行 theta」只在 body 起伏這一個座標成立。
    Step 12R 量的是另一半：姿態越伸展、需要的頂面越短。
    Step 5 只按 body 代價選 theta，會選到放不進它自己挑的那個障礙的姿態。
```

**§5.3 那條規則長什麼樣**

```text
一個 LOWER_BOUND 是【一族】body 軌跡，而那一族裡最便宜的成員，
其垂直起伏【恰好等於】那個下界。
=> 兩者在各自的最小值上可比：rolling 的起伏被規定死、已經是最小；
   swing 的最小值就是它的下界。

大小沒捕捉到的：rolling 還規定了【形狀】，swing 只規定一個極值。
=> 所以【大小打平時算 swing 贏】。compare_body_demand_2d 就是這樣寫的。
=> 這條規則沒有分母，所以不受 (3) 那個稀釋問題影響。
```

**對 paper §25.1 story 的結論**

```text
不能寫：「hybrid 滾走的質心變化比 walking 小」
可以寫：「h >= 120 mm 時滾走的 hip 起伏比 swing 少 9-12%；
        h <= 100 mm 兩者在 1-3% 內打平（swing 略優）。
        交叉點由【swing 的 hip lift 何時開始長】決定 ——
        lift 在 h <= 100 是 0，之後跳成 20 / 40 mm，
        而 rolling 的 overhead 是常數 14.2 mm。」
這比原本的 story 更有力：有機制、有交叉點、有數字，交叉點位置可預測。
```

---

## 2026-08-30 — Step 5 實作（疊圖與 decision rule / Figure D）

**動作**

1. 新增 `day10_11_decision_map_2d.py`（約 800 行）：`StrategyId`、`Availability`、
   `Limiter`、`DecisionTables2D` + `load_tables_2d`、
   `roll_roll_cell_2d` / `swing_swing_cell_2d` / `swing_over_cell_2d` / `refuted_cell_2d`、
   `Decision2D` + `decide_2d`。**刻意保持純函式：不跑 planner、不碰檔案系統。**
2. 新增 `day10_11_swing_over_2d.py`：策略 `#5` 的量測模組。
3. 新增 `day10_11_step5_driver.py`：A–F 六段，`--skip-over` / `--plots-only`。
4. 新增兩份測試（23 + 10），全套 100 passed。

**A 段：`SWING_OVER` 先前【沒有任何一步量過】**

Step 1 的 showcase 只有 `onto` / `off`，Step 2/3 掃的是落腳的兩半。
完成標準寫「五個候選都有 concession 數值」，所以必須先補一次掃描。

```text
80 格（8 個高度 x 10 個頂面長度），38 分鐘，33/80 可行

h <=  80 mm  可行；h >= 100 mm 全滅，失敗模式【100% IK_NOT_CONVERGED】
閉式預測 h_max = standing_hip(85) - apex - min_leg = 264.9 - 30 - 143.8 = 91.1 mm
        -> 實測天花板落在 80 與 100 之間，一致

寬度天花板隨高度收縮 350 -> 280 -> 240 -> 170 mm  (h = 20/40/60/80)
需要的 theta 隨高度上升 50 -> 60 -> 70 -> 80 deg
stride 需求 281 - 734 mm
```

**它的旋鈕是 theta，而且是被逼出來的**：apex 在頂面上方 30 mm，飛行中段腳約在
地面上方 `h + 30`，髖在站姿高度；腿最短的伸展是 `theta = 17 deg` 的一個輪半徑
143.8 mm。`HipTrajectory2D` 是直線，沒有旋鈕能讓髖在中段拱起（Step 2b 同一道牆），
唯一辦法是兩端同時抬 —— 更伸展的站姿。

**核心機制：`L_top` 決定策略，也在 `#1` 內部決定 `theta_climb`**

這是把 Day 6–7 與 Step 4 兩邊的量測接起來才浮現的：

```text
theta 40 -> 85 deg
    required_top_length   269.4 -> 205.0 mm   （Day 6-7 Step 12R，單調【下降】）
    hip 起伏 overhead       14.2 ->  49.4 mm   （Step 4 B 段，單調【上升】）

required_top_length 只由 theta 決定：70 個 cell 跨高度的離散，
10 個 theta 裡有 8 個小於 3 um。

=> 最便宜的 rolling 計畫 = 「required_top_length 還放得下」的【最小】theta。
   不需要搜尋，最佳解永遠在約束邊界上。
   rolling 的代價因此是 L_top 的【階梯】，不是常數。
```

**B–D 段：Figure D**

```text
609 格（7 個高度 x 87 個頂面長度）
    #4 SWING + SWING   210 格
    #5 SWING_OVER      129 格
    #1 ROLL  + ROLL     98 格
    無解               172 格
    257 格有兩個選項、10 格有三個
```

切片（策略隨頂面變短的移動）：

```text
h =  40 mm   #5@20 -> #4@285
h =  60 mm   #5@20 -> #4@245
h =  80 mm   #5@20 -> 無解@175 -> #1@205 -> #4@240
h = 100 mm   #1@205 -> #4@240
h = 120 mm   #4@240 -> #1@265
h = 140 mm   #1@225
h = 160 mm   #4@240
```

`h = 120 mm` 那一列是機制的最佳例證：`L_top = 240-260` 時 `#4` 贏、`>= 265` 時 `#1` 贏，
**不是因為 `#1` 在 240 不可行**，而是那裡只有 `theta = 55` 放得下（144.2 mm），
輸給 `#4` 的 140 mm；到 265 時 `theta = 45` 放得下（137.6 mm）就反超。

**被數據逼出來的四件事**

```text
(1) 能力圖上有一個【洞】，而且是能力缺口不是資料缺口
    h >= 100 mm 且 L_top < 205-235 mm -> 172/609 格完全過不去。
    三個策略各自被【不同】的理由擋住：
      #5 搆不到（h > 91 mm）、#1 付不起 L_transition（>= 205）、
      #4 塞不下落點加起跳點（>= 240）。
    這指出了 Day 12 之後最值得攻的方向。

(2) decision rule 對 lexicographic 順序【敏感】：約 1/3 的格子會變
    margin 放 body 前面 -> 208/609；roll 偏好放前面 -> 186/609。
    第一版不調權重是對的，但【順序本身是未被論證的選擇】，要跟結果一起講。
    另外：預設順序下 tie-break【從未被觸發】—— body 代價永遠分得開前兩名。

(3) margin 門檻不能預設在 0.0（踩到才改的）
    原本預設 0.0，在 h = 60/80 mm 開出一條【假的無解帶】。
    原因：三個 sweep 都有少數可行計畫的最小餘裕在 1e-4 ~ 1e-3 mm 量級 ——
    swing 最緊的一點通常就是【觸地】，那裡餘裕依定義為零。
    這比 planner 自己的 collision_tolerance_m = 1 mm 細一千倍。
    => 預設改成 None（信任 planner 已經判過），門檻做成敏感度：
       任何 >= 0 的門檻都會改變 58 格。

(4) §2.7 的單調性【被推翻】，但真假取決於一個沒被最佳化的參數
    #1 需要 >= 205-245 mm、#4 需要 >= 240 mm -> #4 只在 1/6 個高度比 #1 短。
    但 #4 的下界 = landing_distance(160) + 最短 takeoff(80)，
    而 160 只是 Step 2 格點的固定值；close-out 掃過這一軸，
    100 mm 在 h = 100/150/200 都可行且【lift 完全不變】-> 改用它變成 >= 180 mm，
    每個高度都比 #1 短、claim 成立。兩個版本都記錄。
```

**兩個刻意的設計決定**

```text
A. SWING_OVER 的 body 代價是【零】，而且那是真的
   兩端都站在低地、同一個 theta，直線 hip 軌跡是水平的。
   所以它在可行處一律勝出。代價改成付在【站姿高度】與【stride】。
   站姿代價刻意【沒有】折進比較 —— 它是否已經付掉取決於障礙【之間】的步態，
   那是 Day 15-16 的問題。以獨立欄位 stance_hip_above_min_mm 回報。

B. #5 在未掃到的 L_top 上做【單調閉包】，而閉包是被資料授權的
   可行集在每個高度都是嚴格前綴（h=20 全過、40 前九、60 前八、80 前六），
   而且同高度下較短的頂面就是較短的 stride，嚴格更容易。
   沒有閉包的話，Figure D 上的 #5 會變成十個孤立的點。
   閉包內取「不比任何更寬的已量測頂面更收縮」的 theta，並標 measured_exactly。
```

**這次踩到的三個坑（已修）**

```text
1. 第一次跑在讀 CSV 時崩掉：infeasible 的 #5 列 theta_deg 是空的，
   float(row["theta_deg"]) 沒防護。改用 _num()，並把型別標成 float | None。
2. 地圖的 L_top 軸原本從 150 mm 起，把 #5 的整個定義域（20-170 mm）藏掉了。
   改成從 20 mm 起。
3. pool.map 要全部跑完才回傳，所以 38 分鐘的 A 段【中途完全沒有進度可看】。
   已改成 as_completed + on_result callback（保持結果順序），每 5 格回報一次
   進度與剩餘時間估計。其他 driver 早就有分段 checkpoint，這支漏了。
```

---

## 2026-08-30 — Step 6 實作（segment 級 sequence schema）

**動作**

1. 新增 `day10_11_motion_schema_2d.py`（約 600 行）：`SegmentKind`、`RollingMode`、
   `PointContact2D`、`RollingContact2D`、`RollSampling2D` / `SwingSampling2D`、
   `SwingShaping2D`、`BodyRequirement2D`、`FrameRef2D`、
   `MotionSegment2D`、`MotionSequence2D`。
2. 新增 `day10_11_sequence_builders_2d.py`：兩個 builder 與兩份證據
   （`coverage_report_2d` / `handoff_report_2d`）。
3. 新增 `day10_11_step6_driver.py`：A–C 三段。
4. 新增 `tests/test_day10_11_motion_schema_2d.py`，31 個測試；全套 131 passed。

**結果**

```text
(1) Day 6-7 Step 10R -> 10 個 segment、299/299 幀、無缺漏無重複
(2) Day 8-9 的一條 swing -> 四個 shaping 旋鈕與四個取樣參數全部保留
(3) 四個取樣參數各自被【演示】成會改變它描述的軌跡
```

**四件被資料逼出來、規格 §5.4 沒寫的事**

```text
1. alpha_range 不夠用 —— 這是最重要的一件
   Step 10R 的 LEFT_RIM_TRAILING_TRANSITION 與 LEFT_RIM_ROLL_DOWN
   把 alpha 釘在 -134.5 deg、接觸點釘在後緣角 (450.0, 100.0) mm，
   持續【72 幀】，而 beta 掃了 71 度。腿不是在滾，是繞著 rim 上一個點在轉。
   只記 alpha_range 的 schema 會把這 72/299 幀記成【一個靜止姿態】。
   => RollingContact2D 另記 beta_range / theta_range，
      並用 RollingMode（SURFACE_ROLL / CORNER_PIVOT）指名。
      is_static 再把「兩者都沒動」分出來 —— 只會發生在單幀 segment。

2. rolling 與 swing 不共用取樣詞彙
   swing 以 sample_count 切時間、對 per-sample 關節步長檢查；
   rolling 以 beta_step（retract 段另加 theta_step）推進到停止條件。
   一個半數欄位是 None 的結構，會讓「哪些必填」變成無法回答的問題 ——
   而那正是第 3 條完成標準在測的。
   => 兩個型別，各自完整，都【沒有預設值】。

3. 端點一律是 PointContact2D，即使 rolling 段也是
   §5.4 寫的是 ContactState | RollingContact（二選一），但兩個都要：
   Step 7 的交接檢查要比對關節值，兩端各需要一個確定的姿態；
   §2.5 抱怨的是兩端【之間】。union 型別保留，builder 不產生它。

4. duration_s 對整條 rolling traversal 是 None
   Day 6-7 是準靜態的、從未指定時間。記成缺失而不是編造。
   MotionSequence2D.total_duration_s 因此也是 None（部分求和會被誤讀）。
   => Step 7 的 composer 必須替 rolling 段指定時間，那是一個【新的決定】。
```

**交接量到的事：接觸點可以跳，關節不行**

```text
最大接觸點跳躍   180.5 mm   （foot rim -> right rim：站在地上換成頂到前緣）
最大 theta 跳躍    1.00 deg
最大 beta  跳躍    1.75 deg
```

而且**修正了陷阱 4 的一個容易誤讀處**：`WHEEL_MODE_TOP_ROLL -> LEFT_RIM_READY`
這一格，alpha 從 179.4 變成 -179.4，未折疊看起來跳 358.8 度，
但**接觸點只移動 2.9 mm**。162 mm 是同一 rim 參數化下的值；
實際換到另一個 rim 之後兩者物理上幾乎重合。那是換座標卡，不是運動。

**第三條完成標準怎麼滿足的**

不是斷言欄位存在，是**演示**四個參數各自會改變它描述的軌跡：

```text
arc_samples          seam bridge 34.73 mm (61) -> 5.00 mm (481)，差 6.9 倍
sample_count         最大關節步長 3.41 deg (16) -> 1.71 deg (31)
leg_arc_samples      最小餘裕 1.67085 mm (31) -> 1.66949 mm (241)
max_joint_step_rad   per-sample 限制，(0.20, 16) 與 (0.10, 31) 是同一個約束
                     -> 兩個欄位都記才定得下來
```

**兩個實作時撞到的小地方（已修）**

```text
1. BodyRequirement2D 原本要求 profile 至少兩點，但 RIGHT_RIM_FRONT_CONTACT
   只有 1 幀（輪子剛碰到前緣的那一瞬間），而它是 traversal 自己命名的 phase。
   逼它併進鄰段就是這一步要避免的損失 => 放寬到單點。
2. FrameRef2D 用明確 index 列表而不是 (start, stop) 區間：
   Step 10R 自己的輸出跳過 index 11（一幀被提出又否決），區間會默默把它算進去。
```

---

## 2026-08-30 — Step 7 實作（sequence composer 與交接連續性）

**動作**

1. 新增 `day10_11_composer_2d.py`（約 700 行）：`compose_roll_roll_2d` /
   `compose_swing_swing_2d` / `compose_swing_over_2d` / `compose_2d`、
   `SequenceHandoff2D` + `sequence_handoffs_2d`、`seam_margin_deg`、
   `swing_frame_rows`、`ComposedSequence2D`。
2. 新增 `day10_11_step7_driver.py`：A–E 五段。
3. 新增 `tests/test_day10_11_composer_2d.py`，11 個測試；全套 144 passed。
4. **修正並重跑了 Step 5**（見下面的錯 2 / 錯 3）。

**規格點名的三對，有兩對已經不存在**

Step 7 的任務清單寫在 Steps 2b / 3 / 5 之前。誠實的讀法是串通還活著的三對，
並把兩個推翻當成**第一級輸出**：

```text
#1 ROLL + ROLL     h=140 L=225 mm   10 段 272 幀   餘裕 1.834 mm   用掉 202.0 mm 頂面
#4 SWING + SWING   h= 80 L=240 mm    2 段  62 幀   餘裕 1.393 mm   用掉 240.0 mm
#5 SWING_OVER      h= 60 L= 75 mm    1 段  31 幀   餘裕 1.298 mm   用掉   0.0 mm
#2 / #3            以 day10_11_step7_refusals.csv 記錄理由
```

每一條都在 **Step 5 的規則實際會選它的那一格**組出來——規則永遠不會選的策略，
生得出軌跡也證明不了那條規則。

---

**這一步真正的價值：它抓到前面幾步的三個缺陷**

Step 7 是第一次拿決策去【實際生軌跡】，那揭露了三件事。

```text
錯 1：決策回報的參數【不足以重建它選的動作】
     Step 2/3 是靠 repair ladder（liftoff_rise / touchdown_drop / duration_scale）
     走到可行的，而 decide_2d 只回報 body 旋鈕。
     第一版 composer 照著重建，重現了 sweep 早就修掉的 TERRAIN_COLLISION ——
     #4 在 h = 80 mm 實際需要 ascent_liftoff_rise = 50 mm。
     => 已補進決策輸出。但【沒有】折進 body_deviation_m：
        它們是軌跡整形不是 body 讓步（Step 3 把 hold 放最外層就是這個理由）。

錯 2：#1 的頂面下界拿【輸出】當【前提】
     required_top_length_m 是一次成功 traversal 回報的消耗量。
     Day 6-7 Step 12R 自己的 sweep（它自己的設定）在正好等於它的長度上失敗：
         h=60 theta=40   L=269.4 失敗（LEFT_RIM_HAS_NOT_TAKEN_OVER_AT_CORNER）
                         L=279.4 成功

錯 3：composer 沒有重現 sweep 的取樣設定 —— 而這是 #1 前兩次失敗的【真正原因】
     ObstacleSpec2D.arc_samples      預設 241，但 sweep 跑 121
     TraversalConstraints2D
         .max_seam_bridge_m          預設 5 mm，但必須跟 arc_samples 配對：
                                     seam_bridge_for_sampling_m(121) = 17.5 mm
     這正是陷阱 2，而我踩了。修好之後 #1 在 L=350 和 L=235 都組得出來。
     => 修法不是把數字抄對，是【沿用 Day 6-7 自己的 SweepSettings2D bundle】。
```

**錯 3 改變了錯 2 的歸因，這件事要記下來**

我一度把 `#1` 在 L=225 / L=235 的失敗當成錯 2 的證據。修好錯 3 之後兩者都成立，
所以**那是設定 bug，不是界線的證據**。註解與測試裡引用它們的地方已全部改掉，
錯 2 現在只引用 Day 6-7 自己的 sweep。

**教訓：先確認自己重現得了原實驗，再拿失敗當數據。**

---

**Step 7 自己的新量測，而且它否決了我對錯 2 的第一版修法**

我一度把界線改成「一律 `required + 10 mm`」。但 10 mm 是 θ=40 那次 sweep 的
**格點間距**，不是量到的差額。Step 7 在另一個 θ 直接夾了一次：

```text
h = 140 mm, theta = 70 deg      required_top_length = 222.5 mm
    L = 220 mm   失敗   LEFT_RIM_HAS_NOT_TAKEN_OVER_AT_CORNER
    L = 225 mm   成功
=> 真實下界在 (220, 225]，而 222.5 mm 【落在區間內】—— 一律 +10 mm 會超調。
```

最終規則：

```text
Step 12R 實測過的 theta（40 / 60 / 85）  -> 實測最小值 279.4 / 247.2 / 215.0 mm
其餘 7 個 theta                          -> required_top_length，標記【未驗證】
```

副作用：界線在 theta 上**不再單調**（θ=60 的實測 247.2 > θ=55 的需求 243.0）。
那是兩種來源混用的可見假影，**留著讓它可見**——抹平就等於發明那個已被否證的邊際值。

**Step 5 已用修正後的界線重跑**：`#1` 下界 205–245 → **215–245 mm**；
§2.7 判決不變（1/6）；順序敏感度 200 / 184（原 208 / 186）。

---

**交接檢查**

```text
最大接觸點跳躍   215.9 mm   （foot rim -> right rim）—— 換 rim，不是不連續
最大 theta 跳躍    1.00 deg  = 一個取樣步長
最大 beta  跳躍    1.75 deg
最大 rim 幾何 gap  1.2000 mm = 正好是 Step 0 的標稱值，【不累積】
                              （檢查的是最大【值】而不是變化量：
                                foot rim 0、upper tyre 1.2，交接只是切換不是相加）
```

**但 §6.3 的擔心被量成數字了，而且要當成已知風險記錄。**
在地圖允許的最短頂面上（`#1`, h=140, L=225 mm）：

```text
LEFT_RIM_READY                  1 幀   alpha = -178.83
LEFT_RIM_TRAILING_TRANSITION    7 幀   alpha = -178.83
LEFT_RIM_ROLL_DOWN             81 幀   alpha = -178.83
=> 88 幀全部在距離 ±180 度接縫【1.17 度】的地方進行。

對照 L_top = 350 mm 的同類 traversal：LEFT_RIM_READY 有 46 幀、
alpha 從 -179.4 滾到 -134.5 —— 交出去之後還有【45 度】的 left-rim 弧。

=> #1 的頂面下界，實質上就是【left-rim 弧預算歸零】的那一點。
   交接本身乾淨（接觸點只移動 5.9 mm），但沒有任何餘裕。
   Day 12 之後若要動 arc_samples，這一格要重測。
```

**頂面用量 vs L_transition**

```text
#4 用掉 240.0 mm   在 L_transition 的 196-249 mm 帶內
#1 用掉 202.0 mm   也在帶內，但它【需要】L_top >= 225 mm
#5 用掉   0.0 mm
```

`#1` 那 **23 mm** 的差是前後緣的進出餘裕：`L_transition` 只是 retract 加
wheel-mode 滾動的部分，**不是整個頂面需求**。引用它當「rolling 需要多長的頂面」
會低估 23 mm。

**Step 5 留下的三件全部驗證**

```text
1. #5 的單調閉包        成立 —— L = 75 mm（sweep 未掃過）組得出來
2. #4 的 close-out landing  成立 —— h=100 mm、L=200 mm、landing 100 mm 組得出來
3. 那個【洞】             不是單一天花板：#1/#4 被 top length 擋、#5 被 stride 擋
```

**這次踩到的一個自己造成的坑（已修）**

```text
改寫 compose_roll_roll_2d 時，用「從函式開頭到下一個區段標題」的範圍取代，
把中間後來插入的 swing_frame_rows 一起覆蓋掉了 -> NameError。
大範圍字串取代要先確認範圍內沒有別的東西。
```

---

## 2026-08-30 — Step 8 實作（五個 terrain case 的 regression）

**動作**

1. 新增 `day10_11_step8_driver.py`：A–D 四段，`--plots-only` 從 CSV 重畫。
2. `ComposedSequence2D` 新增 `partial` 欄與 `compose_roll_roll_2d(keep_partial=)`。
3. `tests/test_day10_11_composer_2d.py` 增 2 個測試；全套 146 passed。

**第三條完成標準【無法達成】，而那本身就是這一步最重要的結果**

規格要「至少有一個 case 的最佳策略是混合的，否則 2x2 沒有被驗證」。

```text
#2 ROLL_UP  + SWING_DOWN   被 Step 3 D/E 推翻
#3 SWING_UP + ROLL_DOWN    被 Step 2b 推翻
=> 兩個混合對都不在了 -> 沒有任何 case 的最佳解會是混合的。
   2x2 不是「未被驗證」，是【對角線之外被推翻】。
```

**五個 case**（重新切到 Step 5 的實際 region，但保留每個原 case 的目的；
每個 case 都把【所有】策略都組一次，負向結論建立在實際執行上）

```text
                              贏家              輸家的處境
A  h= 40 L= 50   #5 SWING_OVER   0.0 mm    #1 / #4 都在 top length 上失敗
B  h=140 L=350   #1 ROLL+ROLL  154.2 mm    #4 【可行但要 180.0 mm】—— 貴 25.8 mm
C  h= 80 L=350   #4 SWING+SWING 80.0 mm    #1 【可行但要  94.2 mm】—— 貴 14.2 mm
D  h=120 L=150   都不行                     三個拒絕、三個不同理由
E  h=160 L=350   #4 SWING+SWING 220.0 mm   #1 不可行（沒有任何 theta 走得完）
```

**B / C 是決策規則真正被驗證的地方**：輸家不是「不可行」，是**確實可行、確實較貴**，
而且兩個方向都驗到了。

**負向：19 個拒絕、0 個沒有理由**

那個洞（case D）的三個拒絕有 **3 個不同理由**但只有 **2 個 limiter 名稱**：

```text
#1  top length : 每個可行 theta 都要更長的頂面，最省的要 243.0 mm
#4  top length : 落點吃 160 mm、最短起跳 80 mm，所以頂面至少 240 mm
#5  stride     : 沒有任何 theta 跨得過去
=> #1 和 #4 共用一個 limiter 名稱卻是【不同的牆】。
   比較拒絕理由時要比【完整理由】，不是第一個子句。
```

**Case E：量化推翻 #2 的代價 —— 這一步的頭條**

規格原本為 `#2` 設計的地形，前提**完全正確**：

```text
Day 6-7 在 h = 160 mm：roll_up 成功 10/10 個 theta、roll_down 成功 0/10。
```

用 `keep_partial` 把那個「爬得上去卻用不了」的 ascent 實際計價：

```text
ROLL_UP  hip peak-to-peak    80.1 mm   （2 段 60 幀，theta = 85 deg）
SWING_UP hip peak-to-peak   220.0 mm   （= h 160 + min_hip_lift 60）
=> 滾上去對 body 的要求少【2.7 倍】，而且它在【每一個】theta 都成立。
```

它用不了的原因**和這個 ascent 完全無關**——`#2` 被推翻是下降側的理由。
partial traversal 停在 `ROLL_DOWN: NO_LEGAL_CORNER_PIVOT_CONTINUATION`，
和 Step 2b 在 h >= 160 mm 對「空中來的抵達」量到的**同一個**失敗。

```text
=> 這證明分開決策【確實】有價值：那塊地形上最好的爬升方式和最好的下降方式
   分屬不同 primitive。分解不是錯的，是【現在做不到】。
   而做不到的那道牆（Step 3 E）是可修的 —— 這一格量出了修它值 2.7 倍。
```

**新增的機制：partial sequence**

```text
compose_roll_roll_2d(keep_partial=True) 在 traversal 失敗時，
仍用已接受的幀建出序列並標 partial=True。
ComposedSequence2D.composed 對 partial 回傳 False —— 它不是一個計畫，
是腿在停下來之前做到的事。存在的理由：一個【在失敗的策略裡成功的階段】
仍然值得計價。
```

---

## 2026-08-31 — Step 9 實作（body-requirement timeline，Day 12 交接物）

**動作**

1. 新增 `day10_11_body_timeline_2d.py`：timeline 型別 + 從 `ComposedSequence2D` 導出
   + 交付 CSV 的列產生器 + **只用標準函式庫的 reader 稽核**（`reader_check_2d`）。
2. 新增 `day10_11_step9_driver.py`：A–F 六段，`--plots-only` 從交付的 CSV 重畫。
3. 新增 `tests/test_day10_11_body_timeline_2d.py`：22 個測試；全套 **181 passed**。
4. Notebook 加 Step 9 一節（9.1–9.8）並改寫最後的「下一步」為「Day 10–11 結束」。

**交付物**（規格 §9 點名的檔名）

```text
day10_11_step9_body_requirements.csv        392 列 x 63 欄   <- Day 12 只讀這一個
    provenance 7 / sequence 5 / segment 13 / knot 365 / unresolved_transition 2
day10_11_step9_pinned_endpoint_evidence.csv  端點是硬的的量測 + envelope 用量
day10_11_step9_completion_criteria.csv       四條完成標準 + reader 稽核結果
day10_11_step9_body_requirements.png         【從交付的 CSV 畫出來的】
```

**第一個決定：以 `x` 為自變數（規格要求擇一並寫明理由）**

```text
Day 6-7 的 rolling traversal 是準靜態的，duration_s 全部是 None（陷阱 33）。
=> 指定 rolling 的時間是【新的建模決定】，不是量測結果。
選 (a) 以 x 為自變數；已經存在的時間【照原樣帶過去】：
    swing 段有 segment_duration_s = 0.6 與逐取樣的 knot_time_s
    rolling 段兩欄留白 —— 留白的意思是「從未被指定」，不是 0
而「x 能不能當自變數」是【被檢查的】：三條 sequence 全部單調（tolerance 1e-9 m）。
```

**三條 timeline**

```text
                          knots  硬   偏好  最大 knot 間距  clearance budget
#1 ROLL_ROLL   h=140 L=225  272  272    0      40.6 mm        1.834 mm
#4 SWING_SWING h= 80 L=240   62    4   58       9.8 mm        1.393 mm
#5 SWING_OVER  h= 60 L= 75   31    2   29      13.5 mm        1.298 mm
#2 / #3                       0    0    0        --             --
```

`clearance_budget_mm` 是整條 sequence 上量到的最小地形餘裕。Day 10–11 **沒有量過
TRACK 的容差**，這是唯一有的替代品：body 偏離要求時吃掉的就是它。

**資料逼出來的第一件事：`PINNED` 不是 Step 2b 的特例**

規格把 `PINNED` 指給 Step 2b 那種落地。照那張表直接套，`PINNED` 在三條活著的
sequence 裡**一次都不會出現**（Step 2b 那種落地屬於已被推翻的 `#3`）。

**那個結論是錯的。** 一條 swing 的**兩個端點**都是接觸瞬間：

```text
量測（day10_11_step9_pinned_endpoint_evidence.csv）：
    站在一個面上時 hip_z 是 theta 的嚴格單調函數
    theta = 60 deg -> hip_z = 219.4 mm
    body 低 10 mm  -> theta = 54.4 deg
    body 高 10 mm  -> theta = 65.6 deg
=> 端點的 hip 高度與 theta【互相決定】，body 在那裡不是自由的。
```

這是陷阱 16 的另一面：`plan.valid` 只說 swing 成功了，不說落在要求的 contact state；
抬高 hip 會讓 IK 解出不同的 theta 而 plan 仍然 valid。**把整條 swing 標成偏好，
Day 12 會在觸地那一刻換掉落地姿態而不自知。**

```text
所以 timeline 的規則是：
    rolling 每一幀            TRACK        HARD
    swing 的頭尾兩幀          PINNED       HARD        <- 規格沒寫的
    swing 的中段              LOWER_BOUND  PREFERENCE
    => 278 硬 / 87 偏好
contact_phase 同時把 Day 12 的 duty 交出去：STANCE（rolling 全段 + swing 端點）
vs FLIGHT（swing 中段）。
```

**資料逼出來的第二件事：segment 的純量是 envelope，不是 timeline**

Step 6 的 builder 對一整條 swing 只留一個 `hip_z_min` = 那條 hip 軌跡的**最大值**。

```text
#4 SWING_UP    envelope 299.4 mm，起跳端的真實要求 219.4 mm  -> 多 80.0 mm
#4 SWING_DOWN  envelope 299.4 mm，觸地端的真實要求 219.4 mm  -> 多 80.0 mm
#5 SWING_OVER  envelope 237.5 mm = 全段要求                  -> 多  0.0 mm
```

那 80 mm 剛好是障礙高度：拿 envelope 當 timeline，等於在 swing 還沒起跳就叫 body
先站到頂面高度。**兩個都寫進檔案**：`segment_envelope_hip_z_mm` 在 segment 列
（比較策略時用它，Step 5 就是拿它決策的），`hip_z_required_mm` 在 knot 列（沿路走用它）。

**實作時自己踩到、值得記著的一件事**

```text
第一版對 TRACK 段也算了一個 envelope = max(profile)，於是 #1 被報成
「被多約束了 136.9 mm」。但 TRACK 的要求【就是】那條 profile，
它從來沒有一個純量可以被誤讀 —— 那個數字是憑空製造出來的。
修法：_segment_envelope_m() 對 TRACK 回傳 None，欄位留白，
      max_envelope_excess_mm 也留白（不是 0.0）。

同一類的第二個：blocked pair 的 x_is_monotonic 原本寫 True（空集合上為真），
但那什麼也沒說，會被讀成「檢查過了，沒問題」。改成留白。

【和 Step 9 前置修正踩到的是同一類錯】：一個為某個情況設計的量或標籤，
套到不適用的情況上，就會生出一個沒有人主張過的結論。
```

**未解的 transition：2 列，姿態欄寫 `NOT_GENERATED`**

```text
#2 ROLL_UP + SWING_DOWN   h=160 L=350（Step 8 case E 的地形）
    需要：頂面上一個 theta >= 35 deg 的 foot-rim 起跳
    單腿修法：RETRACT_FOR_SWING_DOWN（Step 8 已標價 2.7 倍）
#3 SWING_UP + ROLL_DOWN   h=140 L=225
    需要：LEFT_RIM_READY
    單腿修法：沒有；只有四腳路徑

theta_deg / beta_deg / segment_duration_s = NOT_GENERATED，不是留白 ——
留白會被讀成 0，而它們不是 0，是還沒有人算過。
resolved 永遠 False（設 True 會拋錯）。
```

**第一條完成標準怎麼檢查的**

規格說「Day 12 只讀這個檔案就夠」。誠實的檢查方式不是在文件裡宣稱它，而是
**用一個不 import 這個專案任何東西的 reader 把它讀回來**：

```text
reader_check_2d()  只 import csv。它稽核：
    每個 segment 宣告的 frame_count 與實際 knot 列數相符
    每條 sequence 的 x 單調
    每個 knot 的 constraint_class 落在 HARD / PREFERENCE 之內
    每個 knot 的 hip_z_upper_bound_mm 是 NOT_MEASURED（留白讀起來像「無上界」）
    每列未解 transition 的 resolved=False 且姿態欄是 NOT_GENERATED
    沒有任何一列宣稱 PHYSICALLY_INFEASIBLE
結果：PASS，5 sequences / 365 knots / 2 unresolved / 0 gap。
測試裡有三個【故意弄壞檔案】的案例確認它會抓到（拿掉 provenance、
少一列 knot、把 verdict 改成 PHYSICALLY_INFEASIBLE）。
```

檔案自己帶 7 列 `provenance`：時間基準、硬/偏好的定義與違反後果、envelope 與
timeline 的差別、`NOT_MEASURED` 的意思、用語階梯、scope。**這些不能只留在 notebook
裡**——完成標準要的是 Day 12 打開那個檔案就看得到。

**圖是從交付的 CSV 畫出來的**，不是從記憶體裡的物件——所以它同時是第一條完成標準
的示範，而不只是一張插圖。`--plots-only` 因此是安全的（Step 7 沒有這個旗標，
理由是它的圖要跑才有；Step 9 相反）。

**四條完成標準全部成立。Day 10–11 到此結束。**

---

## 2026-08-30 — 研究計畫更新：長路 + 多障礙 + CSV 交付

**觸發**：使用者確認最終交付物是「一段夠長的路、跨過一個以上障礙、輸出單一 trajectory.csv」，
並且希望 swing 完之後能沿 rim 繼續滾，以突顯與 walk 的差異。

**動作**：只改 `../new_hybrid_gait_research_plan_zh_TW.md`，未動程式。

```text
新增  §1.1     最終交付物規格（>= 2.0 m、>= 2 個障礙、單一 CSV）
新增  §10.5    rolling 的三種形態、平地推進、落地後滾走（含實測幾何）
新增  §14.1    地形參數量化（間距是實驗變數，不是佈景）
新增  Day 15–16 長路 composer 與 CSV 交付
修改  §5       CSV 加 segment_index / segment_kind / legN_roll_budget_remaining_m
修改  §11/§19/§23  長路的歸屬與優先順序
修改  §27      原本停在 Day 3–5，改寫成目前進度與下一步
追加  Day 10–11 完成標準：Step 6 schema 必須含 WHEEL_ROLL / POST_TOUCHDOWN_ROLL
追加  Day 12   介面要求：以 segment 序列為輸入，不是以「一個障礙」為輸入
```

**兩個實測數字（`LegModel`，R_outer = 145 mm），是 §10.5 的依據**

```text
rim 弧圓心離 hip 的距離     17°: 0.0 mm   45°: 17.8   60°: 29.9   80°: 50.6   100°: 77.7
=> theta0 = 17° 時圓心與 hip 重合，腿是真圓、hip 高度恆定；
   θ 越大滾動時 hip 起伏越大，振幅就是偏移量

滾 200 mm 的 hip 起伏：rolling stance vs walk stance（繞固定接觸點 pivot）
    30°   6.3 / 33.5     45°  14.4 / 28.0     60°  24.2 / 24.1  <- 打平
    80°  41.0 / 20.4    100°  62.9 / 17.6
=> 分水嶺在 θ ≈ 60°，而 Day 8–9 的落地姿態 REGRESSION_THETA_RAD 正好是 60°
=> 要讓「揮完再滾」有別於 walk，落地 θ 必須往下壓
```

滾走預算：`r_eff × 弧`，upper rim 140° → 354 mm、foot rim 80° → 202 mm。是 stride 尺度，
不是零頭；我先前口頭估的「十幾公分」偏低。

**對本檔 §3 下一步的影響**

Step 2b 仍然是第一順位。但在它之後、Step 3 之前，插入一個很輕的工作：
`validate_swing_touchdown_2d` 加 `remaining_rim_arc_m`，`SwingConcession2D` 加
`touchdown_theta_rad` / `post_touchdown_roll_budget_m`。做完就能在既有的 88 個 cell 上
直接看到每個落點還剩多少滾走預算，再決定 Step 2/3 要不要多掃「落地 θ」那一軸。

---

## 2026-08-30 — 質心 story 寫入計畫（不改 Step 順序）

**觸發**：與指導老師討論後確立的 paper story——
**hybrid 滾走步態相對於一般 walking 的優勢，在於機身質心的變化較小。**

**動作**：只改文件，未動程式。

```text
研究計畫  新增 §10.6   質心變化：故事線與它目前被證實到哪裡
          修改 §15     加入四項 CoM metric（主推無因次的 com_z_per_distance）
          修改 §17     contribution bullet 2 補上「讓步 = 質心垂直位移需求」
          修改 §25     story 改寫成含質心機制的版本，新增 §25.2「目前被證實到哪裡」
                       的分界表；原文保留在 §25.3

本檔規格  Step 4       追加定位說明、任務 4（質心代理量）、完成標準四條、兩個輸出檔
```

**明確不做：不把 Step 4 往前提。** 依使用者指示，Step 3 已在進行中，順序維持
`Step 3 -> Step 4 -> Step 5`。質心的內容直接加在 Step 4 裡，做完 Step 3 再進 Step 4。

**這條 story 目前的證據分界**（寫 paper 時必須照這個分界，不可把未證實的寫成結果）

```text
[已有]   swing 側的質心位移需求
         Step 2 的 min_hip_lift 字面上就是這個量
         h <= 100 免費；120/140/160/200 mm -> 20/40/60/80 mm

[已有]   stance 期機制與門檻（單腳一階近似，非整機量測）
         接觸點沿 rim 移動 vs 繞固定點 pivot
         rim 弧圓心離 hip：17deg 0.0mm / 45deg 17.8 / 60deg 29.9 / 80deg 50.6 / 100deg 77.7
         滾 200mm 的 hip 起伏 vs walk pivot：
             30deg 6.3/33.5   45deg 14.4/28.0   60deg 24.2/24.1(打平)
             80deg 41.0/20.4  100deg 62.9/17.6
         => 分水嶺在落地 theta 約 60 度，而 Day 8-9 的 REGRESSION_THETA_RAD 正好是 60 度

[未量]   roll 側的質心軌跡需求  -> Step 4
         rolling 的 hip_z 是【輸出】不是自由變數，所以不是「零質心變化」，
         是另一種形狀的質心變化。若量出來起伏更大，story 要改寫成
         「兩種形式的 body 要求之間的取捨」——是 negative result，不是失誤

[未量]   整機 CoM                     -> Day 12 之後（四腳 trajectory）
[未量]   與 Walk baseline 的實際比較   -> §15 的 Walk baseline 尚未實作
```

**命名約定**：Step 4 的欄位一律用 `hip_*`，不要用 `com_*`。整機 CoM 是 Day 12 之後
才有的量，用 `com_` 開頭會讓後續讀者以為單腳這一步已經算出質心。

**順帶發現的文件不一致**（未修改分析內容，只加註）：研究計畫 §12 的 Day 10–11 一節
仍寫「策略空間 2×2 + 1、五個候選」，但 Step 2b 已把策略 #3 推翻。已在該處加註指向本檔。

---

# 2. 目前狀態

```text
Step 0   完成並驗證（含與 Day 6-7 map 逐格一致的額外佐證）
Step 1   完成並驗證（無損 16/16）
Step 2   完成並驗證（88 cells）+ close-out 完成（48 cells，三個固定軸都有答案）
Step 2b  完成。(b)(c) 成立、(a) 不成立且退化版本也不成立 -> 策略 #3 在目前 planner 下被推翻
Step 3   完成（204 cells / 6 段 / 10,305 次 generate_swing_2d）
         三個完成標準全達成，其中兩個是以「推翻並記錄」的形式達成 -> 策略 #2 也被推翻
Step 4   完成（七段 A-G，77 + 78 rows，【沒有重跑任何 traversal】）
         五個完成標準全達成；§5.3 的跨型別規則已實作，roll 與 swing 現在可以比
Step 5   完成（六段 A-F，4675 rows；只有 A 段跑 planner，80 cells / 38 分鐘）
         五條完成標準四條達成；第 2 條「每個 region 生一條軌跡」是 Step 7 的工作
Step 6   完成（schema + builders + 兩個無損往返 + 四個取樣參數的演示）
Step 7   完成（三條 sequence + 兩個推翻的記錄 + 交接檢查 + Step 5 三件的驗證）
         同時修正了 Step 5 的頂面下界並重跑了它的地圖
Step 8   完成（五個 case、19 個具名拒絕、2x2 的最終判決、混合對代價的量化）
Step 9 前置  完成（Verdict 階梯 + BLOCKED_PAIRS + TransitionRequirement2D）
Step 9   完成（timeline 型別 + 交付 CSV + 只用 csv 的 reader 稽核 + 圖）
         四條完成標準全達成；資料逼出 PINNED 的推廣與 envelope/timeline 的分離
【Day 10-11 結束】
測試     181 passed（21 concession + 14 left-rim + 12 swing-off + 20 roll-concession
                    + 30 decision-map + 10 swing-over + 36 motion-schema + 16 composer
                    + 22 body-timeline）
```

**可以直接使用的東西**

```python
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,          # 共用 terrain，兩個世界都吃
    approach_hip_x_for_clearance_2d,
    standing_scene_2d,            # 站在下層地面或 obstacle top
    rolling_inputs_2d,            # -> (ObstacleSpec2D, TraversalInitialState2D)
    swing_start_from_standing_2d, # -> SwingStartState2D
    check_standing_alignment_2d,
    roll_exit_swing_start_2d,     # 一般化的 roll-exit -> swing-start
    write_rows_csv,
)

from hybrid_note.scripts.experiments.day10_11_swing_sweep_2d import (
    SwingGridSettings2D, SwingGridCell2D,
    minimum_swing_onto_concession_2d, run_swing_onto_grid_2d,
    run_swing_cells_2d,          # 明確 (h, c, settings) 任務清單，settings 可逐格不同
)

from hybrid_note.scripts.experiments.day10_11_left_rim_landing_2d import (
    LEFT_RIM_READY_THETA_RAD,
    left_rim_beta_window_2d, run_beta_windows_2d,   # (b)(c)：beta 窗口與 alpha 接縫
    predicted_pivot_deg_2d, choose_landing_beta_2d, # 落地 beta 選擇（無自由參數）
    left_rim_landing_scene_2d,                      # 依【接觸點】放置落地姿態
    left_rim_ready_from_landing_2d,                 # 用 Day 6-7 自己的 precondition 判定 + 跑 Step 9R
    minimum_swing_to_left_rim_ready_2d, run_left_rim_cells_2d,
)

from hybrid_note.scripts.experiments.day10_11_body_timeline_2d import (
    ConstraintClass, ContactPhase, RequirementBasis,   # HARD / STANCE / 依據
    BodyKnot2D, TimelineSegment2D, BodyTimeline2D,
    timeline_from_composed_2d,    # ComposedSequence2D -> timeline
    timeline_rows_2d,             # -> 交付 CSV 的列（含 provenance）
    reader_check_2d,              # 【只 import csv】把交付檔讀回來稽核
    read_timeline_rows_2d,        # 交付檔 -> 純 dict 列（畫圖用）
    TIME_BASIS, NOT_GENERATED, UPPER_BOUND_STATUS,
)

from hybrid_note.scripts.experiments.day10_11_concession_2d import (
    BindingCeiling, ConcessionSource, BodyRequirementKind,
    SwingConcession2D, swing_concession_from_showcase,
    adjustment_strings_from_concession,
    RollCellConcession2D, roll_cell_concession_from_result,
    RollConcession2D, roll_concession_from_cells,
    compare_concessions,
)
```

**跑測試要加 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`**，否則 ROS 的 `launch_testing`
plugin 會被 pytest 自動載入並因為缺 `yaml` 而整個崩掉：

```bash
cd LegWheel && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_day10_11_concession_2d.py -q
```

**重跑 Step 0 / Step 1 的輸出**

```bash
cd "icra hybrid"
python3 -u LegWheel/hybrid_note/scripts/experiments/day10_11_step0_driver.py --skip-alignment
python3 -u LegWheel/hybrid_note/scripts/experiments/day10_11_step1_driver.py --workers 4
python3 -u LegWheel/hybrid_note/scripts/experiments/day10_11_step2_driver.py           # ~12 min
python3 -u LegWheel/hybrid_note/scripts/experiments/day10_11_step2_driver.py --plots-only
python3 -u LegWheel/hybrid_note/scripts/experiments/day10_11_step2_closeout_driver.py  # ~17 min
python3 -u LegWheel/hybrid_note/scripts/experiments/day10_11_step2b_driver.py          # ~25 min
python3 -u LegWheel/hybrid_note/scripts/experiments/day10_11_step9_driver.py           # ~3.5 min
python3 -u LegWheel/hybrid_note/scripts/experiments/day10_11_step9_driver.py --plots-only
```

**Step 9 的 `--plots-only` 是安全的**：那張圖本來就是從交付的 CSV 畫的，
不是從記憶體裡的物件。（Step 7 刻意沒有這個旗標，理由相反：它的圖要跑才有。）

**重建並執行 notebook**：generator 腳本不在 repo 裡（是 scratchpad 的一次性工具）。
notebook 本身已包含全部內容與輸出，直接開啟即可；要重跑就在 `hybrid_note/notes/` 下
用 `nbclient` 執行它。

---

# 3. 下一步

**Step 0–9 全部完成。Day 10–11 已關閉。**

```text
下一個是規格 §7 的 Step 10（六個後續方向）—— 已經【不是】Day 10-11 的範圍，
或者直接進 Day 12（四腳 timing），輸入就是 day10_11_step9_body_requirements.csv。
```

**交給 Day 12 的三件事**

```text
已解    #1 / #4 / #5 的 body-requirement timeline
        day10_11_step9_body_requirements.csv —— 365 個 knot，
        278 硬（TRACK + swing 端點的 PINNED）/ 87 偏好（swing 中段的 LOWER_BOUND），
        以 x 為自變數；contact_phase 直接給出這條腿的 duty。

未解    #2 / #3 的 TOP_REPOSITION（2 列，resolved=False，姿態欄 NOT_GENERATED）
        只有到了四腳 timing 才驗證得了 —— 它需要「其他腿能否承重」這個判斷。

未解    rolling 段沒有 duration。Step 9 選了以 x 為自變數，
        所以 Day 12 要自己補時間軸，而且那是它自己的建模決定。
```

**Day 12 讀這個檔案時要知道的三件事**

```text
1. hip_z_required_mm（knot 列）才是沿路的要求；
   segment_envelope_hip_z_mm（segment 列）是【比較策略用的】envelope。
   拿 envelope 當 timeline 會在 #4 的兩段各多抬 80 mm。
2. swing 的【頭尾兩幀】是 PINNED 不是 LOWER_BOUND。
   在那裡動 body 會換掉落地姿態而 plan 仍然 valid（陷阱 16）。
3. hip_z_upper_bound_mm 是 NOT_MEASURED，不是「無上界」。
   抬高 hip 會增加 reach 需求，Day 10-11 沒有掃過那條線。
```

## 3.1 Step 9 前置修正【已完成，這裡記錄結果】

### (i) `Verdict` 階梯與 `BLOCKED_PAIRS`

```text
day10_11_decision_map_2d.py
    Verdict                   COMPOSED / DIRECT_HANDOFF_INFEASIBLE /
                              REQUIRES_MULTILEG_REPOSITION / OUT_OF_ENVELOPE /
                              NOT_MEASURED / PHYSICALLY_INFEASIBLE
    StrategyCell2D.verdict    每格帶標籤
        .effective_verdict    依 Availability 對照（不是二分猜測）；
                              【永遠不推導出】PHYSICALLY_INFEASIBLE
    Availability.REFUTED   -> Availability.HANDOFF_BLOCKED
    Limiter.REFUTED        -> Limiter.HANDOFF_BLOCKED
    refuted_cell_2d        -> blocked_pair_cell_2d（舊名保留為別名）
    REFUTATIONS（純字串）   -> BLOCKED_PAIRS（BlockedPair2D 紀錄）
        verdict / evidence / single_leg_fix / multileg_route
        REFUTATIONS 仍以 summary 字串保留，舊讀者不會壞
```

**為什麼從字串換成紀錄**：字串會被單獨引用，於是「被推翻」就旅行出去了。
紀錄強迫呼叫端**連同結論等級與退路一起帶走**。
`#2` 有單腿修法（`RETRACT_FOR_SWING_DOWN`），`#3` 沒有——這個差別以前是隱形的。

### (ii) `TransitionRequirement2D`

```text
day10_11_motion_schema_2d.py
    TransitionKind.TOP_REPOSITION
    TransitionRequirement2D
        kind / source_contact / target_condition / evidence
        requires_external_support = True
        resolved = False        <- 設成 True 會【拋錯】
    MotionSequence2D.unresolved      未解的需求
                    .is_complete     有任何一個未解 -> 不是完整計畫
                    .rows()          segment 與 unresolved 同表，用 row_kind 區分

day10_11_composer_2d.py
    ComposedSequence2D.verdict / .unresolved
    top_reposition_requirement_2d(strategy, h, L_top)
    CSV 多了 verdict / unresolved_transitions / unresolved_kinds
```

三個刻意的設計決定：

```text
1. 它【不帶】軌跡也不帶時間 —— 單腿模型答不出「離地期間誰支撐 body」。
2. resolved=True 會拋錯 —— 解掉它的方式是【換成真正的動作】，不是翻旗標。
3. target_condition 是文字不是姿態 —— 釘死姿態正是 §5.6 要延後的工作。
```

### 已重新產生的輸出

```text
day10_11_step5_decision_map.csv   多了 verdict 欄（數字未變）
day10_11_step8_cases.csv          多了 verdict / unresolved_* 欄
day10_11_step7_*.csv              【尚未重跑】—— 需要 ~2 分鐘的 rolling traversal，
                                  數字不會變，只有欄位會變
```

測試：**159 passed**（新增 11 個，釘住新用語與新型別）。

### 實作時自己踩到的一件事（已修，值得記著）

第一版把 §5.5 原本的四個標籤直接套到整張地圖，於是 **19 個拒絕全部變成
`DIRECT_HANDOFF_INFEASIBLE`**——包括 `#1` 只是因為頂面太短的那些。

```text
那是同一種過度推廣，只是方向相反：
    原本擔心 -> 把條件性結果寫成物理結論
    實際發生 -> 把幾何界線寫成交接失敗
```

修法是補 `OUT_OF_ENVELOPE` / `NOT_MEASURED`，並讓 `effective_verdict`
**依 `Availability` 對照而不是二分猜測**。

**教訓**：一套為某個特定問題設計的標籤，套到更大的範圍之前，
要先檢查**每一類拒絕**落在哪裡。

## 3.3 Step 9 的內容與它必須明說的一件事【已完成，這裡保留當時的規格】

```text
#1 / #4 / #5   輸出已 composed sequence 的 body-requirement timeline
#2 / #3        輸出 REQUIRES_MULTILEG_REPOSITION 或更具體的 unresolved transition
               【不要】猜測 top reposition 的 theta / beta / duration
```

**必須明說的問題：rolling 沒有時間。**

```text
Day 6-7 的 rolling trajectory 是【準靜態】資料，沒有 duration。
=> 任何 rolling timing 都是【新的 modeling decision】，不是量測結果。
   (a) 以 x 為自變數  不需要新決定
   (b) 以 t 為自變數  要指定 rolling 段的時間，並寫出假設與理由
   兩者都可接受，但必須擇一並寫明。
```

硬約束 vs 偏好的材料已經有了：

```text
硬約束   BodyRequirementKind.TRACK        rolling 段 —— 違反則接觸幾何不成立
         BodyRequirementKind.PINNED       Step 2b 那種落地 —— hip 沒有自由度
偏好     BodyRequirementKind.LOWER_BOUND  swing 段 —— body 可以更高，只是沒必要
```

> **已完成的答案**：選了 (a) 以 `x` 為自變數，而且「x 能不能當自變數」是被檢查的
> （三條 sequence 全部單調）。已存在的 swing 時間照原樣帶過去。
> 硬/偏好的對照實作出來之後**多了一條規格沒寫的**：swing 的兩個端點是 `PINNED`。
> 完整經過見 §1 的 2026-08-31 session 紀錄。

## 3.4 Step 9 用到 / 產出的東西

```text
[用到]
compose_2d(h, L_top, tables, strategy=...)   一條 sequence，或拒絕的理由
MotionSegment2D.body_requirement             每段的 kind / x_range / hip_z 下界或軌跡
ComposedSequence2D.frame_rows                FrameRef2D 指向的逐幀資料

[產出]
timeline_from_composed_2d(result)            ComposedSequence2D -> BodyTimeline2D
    .knots / .segments / .unresolved / .gaps
    .is_executable      有動作、沒有未解需求、沒有未規劃缺口
    .x_is_monotonic     x 能不能當自變數（被檢查，不是假設）
    .max_knot_spacing_m timeline 是折線，這是最粗的一段
timeline_rows_2d([...])                      -> 交付 CSV 的列（provenance 在最前面）
reader_check_2d(path)                        【只 import csv】把交付檔讀回來稽核
```

## 3.5 Day 10–11 結束後仍然開著的四件事（技術債，不是缺陷）

```text
1. #4 在較長頂面上有一段【走過頂面】誰都生不出來
   Step 7 / 8 刻意挑了兩段 swing 首尾相接的格避開它。
   引擎缺 run_retract_and_reset_branch_2d 的 stop_at = "forward_distance"。

2. #1 最短頂面那條的接縫餘裕只有 1.17 deg
   88 幀的下降全部貼著 ±180 度接縫。若要動 arc_samples，這一格要重測。

3. 7 個 theta 的頂面下界仍【未驗證】（只有 40 / 60 / 85 有實測）
   那些格子帶著 top_length_bound_measured = 0 的標記。

4. Step 2 的 88-cell 主 map 仍固定 theta = 60 deg
   => #4 的 body 代價是【偏保守的上界】；重掃後 #4 的 region 只會變大不會變小。
```

## 3.6 Day 10–11 之後的六個方向（規格 §7 Step 10 有完整版）

```text
優先 1  Swing endpoint theta 自動選擇          —— 直接壓低 #4 目前保守的 body 代價
優先 2  最佳化 landing / takeoff distance      —— 可能縮小「高障礙 + 短頂面」的洞
優先 3  新增 RETRACT_FOR_SWING_DOWN            —— 【最可能快速拿回 #2 的單腿修正】
                                                 價碼已知：h=160 mm 上 2.7 倍
優先 4  分段 hip trajectory（中段拱高）         —— 解 SWING_OVER 的 91 mm 天花板與 Step 2b
優先 5  四腳 TOP_REPOSITION                    —— 此時才能【真正重測 #2 / #3】
優先 6  多障礙長路 coverage test               —— 在做完之前不要宣稱「多數 obstacle 可通過」
```

---

# 4. 已知陷阱（做下去之前一定要知道）

1. **不要沿用 `swing_onto_step_2d` 的貪婪 hip ladder 做 sweep。** 規格 §6.1，150 mm 假洞已實測。
2. **`arc_samples` 不能為了加速隨便調低。** seam 寬度是取樣假影，低於 ~140 會偽否決 right→left
   handover；用 `seam_bridge_for_sampling_m(arc_samples)` 配對，不要用預設 5 mm。
3. **兩套 rim 幾何差 1.2 mm。** `LegModel.rim_point` 用 0.145 m，接觸管線的繪圖弧在 upper rim
   是 0.1438 m。沿用 Day 8–9 決定：量化記錄、不修。
4. **α 在 ±40° / ±180° 接縫上會瞬移**（45 mm / 162 mm）。交接點要檢查離接縫多遠。
   **Step 2b 量到它在 swing 側有多硬**：把 reach 問題排除之後，跨過 `α = -40°`
   那一步的關節跳躍是 **約 29°**，而且加密取樣（61 → 481）只會讓它收斂上去，不會變小。
   **後果**：站姿接觸永遠在 `foot_rim, α = 0`，所以**任何**要落在 left rim 的 swing
   都必須穿過它 —— 這就是策略 #3 被推翻的原因。要落 left rim，得先讓 rim 之間的
   接觸連續化（或讓 IK 允許跨段延續）。
5. **完整 traversal 很慢**（實測中位數 264 s／cell）。sweep 要平行化，並先估總時間。
   Step 4 要重做 rolling 側時規模更大。
6. **貪婪 ladder 的結果不是最小值。** 任何用 `swing_onto_step_2d` / `swing_off_step_2d`
   得到的 concession，`source` 都是 `GREEDY_LADDER`、`is_comparable` 為 False。
   Step 2 必須改成完整格點搜尋才會變成 `GRID_MINIMUM`。
7. **可行 theta 窗口是梳狀的**（h = 120 mm 在 45 與 55 可行、50 是洞）。
   不要用 `max - min` 當 robustness 指標。
8. **pytest 需要 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`**（見 §2）。
9. **approach clearance 在 c ≈ 60 mm 飽和。** 掃到 160 mm 是為了確認飽和，
   不是因為那裡還有資訊。Step 3 的下降側網格可以據此縮小。
10. **不要把 §26.5(6) 的「效應」和「機制」混為一談。** 效應成立（approach 是主變數），
    機制不成立（輪半徑 vs 接觸點距離）。真正的機制是 lift-off 階段的掃掠體積。
11. **notebook 用 nbclient 執行時要 `%matplotlib inline` 加上顯式
    `matplotlib.use('module://matplotlib_inline.backend_inline')`**，
    否則圖會變成 "Agg is non-interactive" 警告而完全不產生。
12. **長時間 sweep 一定要完全脫離 session。** 已經被中斷兩次（Step 0 handoff、Step 2 close-out），
    每次都白跑幾十分鐘。原因有兩個：log 寫在 session 專屬的 scratchpad（session 重啟就被清掉），
    以及程序沒脫離 process group。正確寫法：

    ```bash
    cd "icra hybrid" && setsid nohup python3 -u <driver>.py > /tmp/<name>.log 2>&1 < /dev/null & disown
    ```

    另外 **`pkill -f <driver 名稱>` 會連自己的 shell 一起殺掉**（wrapper 的命令列含有那個字串）。
13. **theta = 17 deg 的 beta 窗口掃描很慢**（每個 beta 都要建 scene + 跑接觸查詢，
    0.5 deg 步長約 200 s）。它與高度無關，所以**算一次就好**，用 `run_beta_windows_2d`
    平行跑幾個高度只是為了驗證這件事，不是因為需要多份。
    `minimum_swing_to_left_rim_ready_2d` 一定要傳 `landing_beta_deg`，否則每個 cell 都會重算。
14. **落地 hip 高度在 Step 2b 是【釘死】的，不是下界**（`BodyRequirementKind.PINNED`）。
    把落地 contact state 釘死成 (point, rim, alpha) 並要求某個 theta 之後，hip 就沒有自由度了。
    契約會擋下對 pinned landing 加 `min_hip_lift_m`；為什麼那個擋很重要，見第 16 則。
15. **`LEFT_RIM_READY` 是必要條件，不是充分條件。** 它只檢查【腿】（theta、rim、接觸合法），
    不檢查【接下來下得去嗎】。Step 2b 實測 35/35 通過 precondition，但 h >= 160 mm 的 14 個
    Step 9R 仍然失敗於 `NO_LEGAL_CORNER_PIVOT_CONTINUATION`。要判斷一個抵達可不可用，
    必須真的跑 `run_left_rim_roll_down_2d`，不能只看 precondition。
16. **落地 theta 是 IK 的【輸出】，不是輸入。** `plan.valid` 只說「swing 成功了」，
    不說「落在要求的 contact state」。抬高 hip 會讓 IK 解出不同的 theta 而 plan 仍然 valid。
    所以 Step 2b 在 `plan.valid` 之後【還要】用最後一個 sample 重建姿態、
    再跑一次 `_corner_readiness_failure`。
17. **rim budget 的方向是往 alpha = -40 deg，不是往 -180 deg。**
    `_rim_region_alpha_end_rad` 沿 sample index 往前走到區域盡頭，實測 5 個 beta 全部符合
    `budget = -40 - alpha`。所以 corner pivot 需要的弧是【接觸點到 -40】那一段，
    落地 alpha 越靠近 -180，budget 越大但 seam margin 越小 —— 1:1 互換。
18. **`generate_swing_2d` 會【丟例外】而不是回報不可行。** 腳最後停在空中
    （`final_rim=None`）時它丟 `ValueError`，因為無法判定一個沒有觸地的 touchdown
    contact state。Step 3 A 段有 153 次落在 11 格。**要接住並記成 `planner_refusals`，
    不要去改 planner**——它產出了 Step 2 / 2b 的全部結果，中途換掉就不能互相比較了。
    也不要把它併進 infeasible：那會混淆「planner 表達不了」與「機器人做不到」。
19. **下降側有一個 theta 下限 = 35 deg，而且與 takeoff 距離無關。**
    32 deg 失敗、35 deg 可行（hold 1.000）、38 deg 可行（hold 0.875），三個 takeoff
    距離結果一致。所以任何「先 retract 再 swing down」的路徑都**不能** retract 到
    17 deg 的 wheel mode。Day 6–7 Step 6.5 寫死 `theta_target=17`，那是為 ROLL_DOWN 準備的。
20. **下降側需要 `duration` 這個上升側不需要的旋鈕。** 觸地速度（`TOUCHDOWN_VELOCITY_TOO_HIGH`）
    是下降特有的綁束條件——腳帶著整個落差抵達低地，而它的旋鈕是【時間】不是幾何。
    Step 3 A 段 25 個不可行 cell 有 13 個是這個原因；上升側 18 個不可行 cell **一個都沒有**。
    沒有 duration ladder 的格點會把「只是太趕」報成不可行。
21. **`write_rows_csv` 用第一列的 keys 當表頭。** 分段 checkpoint 時，如果後面的段多了欄位
    （例如 B 段的 `claim_variant`），寫檔會丟 `dict contains fields not in fieldnames`。
    要先取所有列的**欄位聯集**再補空值。
22. **「rolling 是零 body 讓步」是錯的。** rolling 的 hip 起伏 = `h + overhead(theta_climb)`，
    `theta = 40 deg` 時 overhead 是**常數 14.2 mm**。規格 §2.2 那句直覺已標註修正。
    真正的差別是**形狀**：rolling 是梯形（含一段平頂），swing 是三角形。
23. **任何有分母的比較，對「誰能自選什麼」極度敏感。** swing 可自選 approach clearance，
    而較大的 clearance 在**零垂直代價**下加長前進距離、稀釋它自己的無因次指標；
    rolling 的 clearance 被 Day 6–7 凍結在 40 mm。實測讓 swing 自選會在 h = 60 / 80 mm
    **翻盤**。主要結果一律用**對齊的自由度**，自選版當敏感度檢查，兩者都寫進 CSV。
    沒有分母的比較（`compare_body_demand_2d`）不受影響。
24. **rolling 的 traversal 不是姿態中性的。** 它站著（`theta_climb`）出發、停在 wheel mode
    （`theta = 17 deg`），淨差 **−39.8 mm**。那不是跨越障礙的代價。
    在 peak-to-peak 裡會抵銷，在絕對高度圖裡不會 —— 不先講會誤導。
    `HipExcursion2D.is_posture_neutral` 把它標出來。
25. **matched stages 和 whole obstacle 會得到相反的結論，而且兩個都對。**
    matched stages 把頂面那段拿掉，而**那正是 rolling 免費賺前進距離的地方**
    （wheel mode 平走，垂直位移 0）。要回答質心 story 必須用 whole obstacle。
    引用 Step 4 的數字時一定要說是哪一種。
26. **Day 6–7 的軌跡 CSV 已經夠用，不要重跑 traversal。**
    `day6_7_step11r_sweep_trajectories.csv` 有全部 70 個 cell 的 `hip_x_m` / `hip_z_m`，
    含 stage / phase / `accepted`。Step 4 整步沒有跑過一次 traversal。
27. **`margin` 門檻不能預設在 `0.0`。** 三個 sweep 都有少數**可行**計畫的最小餘裕在
    **1e-4 ~ 1e-3 mm** 量級——swing 最緊的一點通常就是**觸地**，那裡餘裕依定義為零。
    planner 自己的 `collision_tolerance_m` 是 **1 mm**，所以 `0.0` 等於用比模型自身
    容差銳利 1000 倍的標準推翻模型的判定。實際後果：在 h = 60 / 80 mm 開出一條
    **假的「無解」帶**。預設改成 `None`（信任 planner），門檻做成敏感度。
28. **`L_transition` / `required_top_length` 只由 `theta_climb` 決定，與高度無關。**
    Day 6–7 Step 12R 的 70 個 cell，10 個 θ 裡有 8 個跨高度離散 < 3 μm。
    這讓 `L_top` 軸可以**不重跑**就建出來，也是 Step 5 decision rule 的骨架：
    `required_top_length(θ)` 單調下降、hip 起伏 `overhead(θ)` 單調上升，
    所以最佳 θ 永遠在約束邊界上。
29. **`pool.map` 要全部跑完才回傳。** 38 分鐘的 sweep 中途完全沒有進度可看，
    而且無法估計剩餘時間。用 `as_completed` + callback（結果順序照樣可以保住）。
    其他 driver 早就有分段 checkpoint，Step 5 的第一版漏了。
30. **畫地圖時先確認軸涵蓋每個候選的定義域。** Step 5 的 `L_top` 軸原本從 150 mm 起，
    把 `#5 SWING_OVER` 的整個定義域（20–170 mm）藏掉了，圖上看起來像它幾乎不可行。
31. **rolling 有兩種不同的運動，而且只看 `alpha` 分不出來。**
    `SURFACE_ROLL`（rim 滾過地形，α 與接觸點都動）和 `CORNER_PIVOT`
    （腿繞著 rim 上一個點轉，α 與接觸點都釘死、只有 β 掃）。
    Step 10R 有 **72 / 299 幀**是後者。只記 `alpha_range` 會把它們記成一個靜止姿態。
32. **±180° 接縫的「162 mm」是同一 rim 參數化下的值。** 實際跨過去換到另一個 rim 之後，
    接觸點**只移動 2.9 mm**——兩者物理上幾乎重合。α 從 `179.4` 變 `-179.4`
    看起來像跳 358.8 度，那是**換座標卡不是運動**，比較時要把角差折進 `(-180, 180]`。
    同一份報告：整條 traversal 最大**接觸點**跳躍 180.5 mm，
    但最大**關節**跳躍只有 θ 1.00° / β 1.75°。**接觸點大跳不是不連續，關節大跳才是。**
33. **Day 6–7 的 traversal 沒有時間。** 它是準靜態的，`duration_s` 全部是 `None`。
    要串出有時間的 sequence，composer 必須自己指定——那是一個**新的建模決定**，
    不是從既有資料讀得出來的。編一個數字等於在真數字該去的位置放假資料。
34. **Step 10R 的 frame index 不連續**（跳過 11，一幀被提出又否決）。
    用 `(start, stop)` 區間表示一段的幀，會默默把它算進去。用明確的 index 列表。
35. **一個決策若不含 repair 旋鈕，就不足以重建它選的動作。**
    Step 2 / Step 3 是靠 `liftoff_rise` / `touchdown_drop` / `duration_scale` 走到可行的。
    只回報 body 旋鈕的決策，重建時會**重現 sweep 早就修掉的碰撞**。
    但那些旋鈕**不能**折進 `body_deviation_m`——它們是軌跡整形不是 body 讓步。
36. **不要把「成功後回報的消耗量」當成「可行性前提」。**
    `required_top_length_m` 是一次成功 traversal 消耗了多少頂面；Day 6–7 Step 12R
    自己的 sweep 在正好等於它的長度上**失敗**。但也**不要一律加一個邊際值**：
    Step 7 在另一個 θ 夾出 `(220, 225]`，而 `required` 的 222.5 mm 落在區間內。
    做法：實測過的 θ 用實測值，其餘標記未驗證；代價是界線在 θ 上不再單調，留著讓它可見。
37. **重跑別人的實驗時，沿用它自己的 settings bundle，不要自己重建。**
    `ObstacleSpec2D.arc_samples` 預設 241 但 Day 6–7 的 sweep 跑 121，
    而 `max_seam_bridge_m` 必須跟它**配對**（17.5 mm vs 預設 5 mm）。兩個預設都不對，
    結果 traversal 在 sweep 說可行的地方失敗。用 `SweepSettings2D` 建三樣東西。
    **教訓**：那兩次失敗一度被當成第 36 條的證據——錯誤歸因。
    **先確認自己重現得了原實驗，再拿失敗當數據。**
38. **`#1` 的頂面下界，實質上就是 left-rim 弧預算歸零的那一點。**
    在最短可用頂面上（h=140, L=225 mm），`LEFT_RIM_READY` 只有 1 幀，
    接著 88 幀的下降全部把 α 釘在 `-178.83 deg`——**距離 ±180 度接縫 1.17 度**。
    對照 `L_top = 350 mm`，交出去之後還有 45 度的 left-rim 弧。
    交接本身乾淨（接觸點只移動 5.9 mm），但**沒有任何餘裕**。
39. **`L_transition` 不是整個頂面需求。** `#1` 用掉 202.0 mm 但需要 `L_top >= 225 mm`——
    差的 23 mm 是前後緣的進出餘裕。引用 `L_transition` 當「rolling 需要多長的頂面」
    會低估 23 mm。
40. **大範圍字串取代前，先確認範圍內沒有別的東西。** 改寫 `compose_roll_roll_2d` 時
    用「從函式開頭到下一個區段標題」的範圍取代，把中間後來插入的 `swing_frame_rows`
    一起覆蓋掉了。
41. **比較「拒絕理由」時要比完整理由，不是它的第一個子句。**
    Step 8 那個洞的三個拒絕裡，`#1` 和 `#4` **共用 `top length` 這個 limiter 名稱
    卻是不同的牆**——一個付不起 `L_transition`，另一個塞不下「落點加起跳點」。
    只比開頭會把它們算成同一個原因。
42. **一個「在失敗的策略裡成功的階段」仍然值得計價。**
    `compose_roll_roll_2d(keep_partial=True)` 在 traversal 失敗時仍用已接受的幀
    建出序列並標 `partial=True`；`composed` 對它回傳 `False`，因為它不是計畫。
    Step 8 靠它量出 `h = 160 mm` 的 `ROLL_UP` 只要 **80.1 mm** 而被迫改用的
    `SWING_UP` 要 **220.0 mm**——那是 2×2 對角線塌陷的**價碼**。
43. **一條 swing 的兩個端點對 body 是【硬的】，不是偏好。** 腳踩在面上時
    `hip_z` 與 `theta` 互相決定（實測：`theta = 60 deg` -> `hip_z = 219.4 mm`；
    body ±10 mm -> `theta` 54.4 / 65.6 deg）。把整條 swing 標成 `LOWER_BOUND`，
    下游會在觸地那一刻抬高 body，**`plan.valid` 仍然成立而落地姿態已經換掉**
    （這是第 16 則的另一面）。Step 9 因此把 §5.3 的 `PINNED` 從「Step 2b 的特例」
    推廣成「每一條 swing 的頭尾兩幀」。
44. **`MotionSegment2D.body_requirement` 的純量是 envelope，不是 timeline。**
    Step 6 的 builder 對一條 swing 只留 `hip_z_min = max(hip 軌跡)`。那是正確的
    envelope（Step 5 就是拿它決策的），但沿路用它會在 `#4` 的兩段各多抬 **80 mm**
    ——剛好是障礙高度。交付檔兩個都寫：`segment_envelope_hip_z_mm`（比較策略）
    與 `hip_z_required_mm`（沿路走）。
45. **`TRACK` 段【沒有】那個純量，不要替它算一個。** Step 9 第一版對 TRACK 也取
    `max(profile)` 當 envelope，於是 `#1` 被報成「被多約束 136.9 mm」——但 TRACK
    的要求**就是**那條 profile，它從來沒有一個純量可以被誤讀。同一次還有第二個：
    blocked pair 的 `x_is_monotonic` 原本寫 `True`（空集合上為真），那什麼也沒說，
    卻會被讀成「檢查過了，沒問題」。兩個都改成**留白**。
    **和 Step 9 前置修正踩到的是同一類錯**：一個為某個情況設計的量或標籤，套到
    不適用的情況上，就會生出一個沒有人主張過的結論。

---

# 5. 檔案索引

```text
規格        day10_11_roll_swing_selection_zh_TW.md
本檔        day10_11_implementation_log_zh_TW.md
notebook    ../hybrid_gait_day10_11_motion_selection_dashboard.ipynb

程式        LegWheel/hybrid_note/scripts/experiments/
                day10_11_shared_scene_2d.py     Step 0 共用 scene 與幣別
                day10_11_step0_driver.py        Step 0 批次
                day10_11_concession_2d.py       Step 1 契約
                day10_11_step1_driver.py        Step 1 驗證批次
                day10_11_swing_sweep_2d.py      Step 2 格點搜尋
                day10_11_step2_driver.py        Step 2 sweep + 圖 + 驗收
                day10_11_step2_closeout_driver.py  Step 2 close-out（--plots-only 可只重畫）
                day10_11_left_rim_landing_2d.py    Step 2b 模組（窗口 / 落地 / Step 9R 轉接）
                day10_11_step2b_driver.py          Step 2b A–E 段（--only degenerate 可單跑 E）
                day10_11_swing_off_sweep_2d.py     Step 3 模組（三層 ladder / ROLL_UP 出口）
                day10_11_step3_driver.py           Step 3 A–F 段（--sections 續跑、--plots-only）
                day10_11_roll_concession_2d.py     Step 4 模組（hip 起伏度量 / 跨型別規則）
                day10_11_step4_driver.py           Step 4 A–G 段（--plots-only）
                day10_11_decision_map_2d.py        Step 5 決策函式（【純函式】，不跑 planner）
                day10_11_swing_over_2d.py          Step 5 策略 #5 的量測模組
                day10_11_step5_driver.py           Step 5 A–F 段（--skip-over / --plots-only）
                day10_11_motion_schema_2d.py       Step 6 schema（segment / sampling / body req）
                day10_11_sequence_builders_2d.py   Step 6 builders + 無損與交接證據
                day10_11_step6_driver.py           Step 6 A–C 段（--plots-only）
                day10_11_composer_2d.py            Step 7 composer + 交接檢查
                day10_11_step7_driver.py           Step 7 A–E 段（無 --plots-only：圖要跑才有）
                day10_11_step8_driver.py           Step 8 A–D 段（--plots-only 從 CSV 重畫）
                day10_11_body_timeline_2d.py       Step 9 timeline 型別 + 交付列 + reader 稽核
                day10_11_step9_driver.py           Step 9 A–F 段（--plots-only 從交付 CSV 重畫）
測試        LegWheel/tests/test_day10_11_concession_2d.py         21 passed
            LegWheel/tests/test_day10_11_left_rim_landing_2d.py   14 passed
            LegWheel/tests/test_day10_11_swing_off_sweep_2d.py    12 passed
            LegWheel/tests/test_day10_11_roll_concession_2d.py    20 passed
            LegWheel/tests/test_day10_11_decision_map_2d.py       30 passed
            LegWheel/tests/test_day10_11_swing_over_2d.py         10 passed
            LegWheel/tests/test_day10_11_motion_schema_2d.py      36 passed
            LegWheel/tests/test_day10_11_composer_2d.py           16 passed
            LegWheel/tests/test_day10_11_body_timeline_2d.py      22 passed
                                                          合計   181 passed

輸出        day10_11_step0_scene_alignment.csv        80 cells
            day10_11_step0_roll_exit_handoff.csv      60 stage exits
            day10_11_step1_swing_concessions.csv      16 showcases
            day10_11_step2_swing_onto_sweep.csv       88 cells
            day10_11_step2_min_hip_lift_map.png
            day10_11_step2_binding_ceiling_map.png
            day10_11_step2_min_liftoff_map.png
            day10_11_step2_closeout.csv               48 cells（ceiling/landing/theta 三段）
            day10_11_step2_closeout.png
            day10_11_step2b_beta_window.csv           4 heights
            day10_11_step2b_swing_to_left_rim_ready.csv  90 cells
                                                      （landing 25 / budget 35 / diagnostic 15 / degenerate 15）
            day10_11_step2b_alpha_seam_distance.png
            day10_11_step2b_landing_map.png
            day10_11_step3_swing_off_sweep.csv        204 cells
                                                      （map 99 / claim 6 / theta 15 /
                                                        roll_up_arrival 45 / retract 30 / theta_floor 9）
            day10_11_step3_min_hip_hold_map.png
            day10_11_step4_roll_concession.csv        77 rows（cell 70 / concession 7）
            day10_11_step4_hip_excursion.csv          78 rows
                                                      （matched 12 / whole 12 / top_length 6 /
                                                        rule 6 / theta_trade 42）
            day10_11_step4_roll_hip_profile.png
            day10_11_step4_hip_excursion_roll_vs_swing.png
            day10_11_step5_swing_over.csv             80 cells（8 高度 x 10 頂面長度）
            day10_11_step5_decision_map.csv           4675 rows
                                                      （decision 609 / strategy_cell 3045 /
                                                        slice 609 / order_sensitivity 395 /
                                                        monotonicity 7 / landing_sensitivity 7 /
                                                        margin_sensitivity 3）
            day10_11_step5_figure_d.png               <- paper 的 Figure D
            day10_11_step5_cost_gap.png
            day10_11_step5_top_length_slice.png
            day10_11_step6_sequence_segments.csv      11 segments（10 rolling + 1 swing）
            day10_11_step6_handoff.csv                9 hand-overs
            day10_11_step6_sampling_evidence.csv      4 parameters
            day10_11_step6_sequence_segments.png
            day10_11_step7_pair1_roll_roll_frames.csv / _summary.csv
            day10_11_step7_pair4_swing_swing_frames.csv / _summary.csv
            day10_11_step7_pair5_swing_over_frames.csv / _summary.csv
                                                      （pair2 / pair3 不存在：被推翻，
                                                        改以 refusals.csv 記錄）
            day10_11_step7_handoff_report.csv         10 hand-overs
            day10_11_step7_refusals.csv               2 pairs
            day10_11_step7_top_length_budget.csv      3 strategies
            day10_11_step7_open_items.csv             Step 5 留下的三件
            day10_11_step7_compose_attempts.csv
            day10_11_step7_sequences.png
            day10_11_step8_cases.csv                  25 rows（5 cases x 5 strategies）
            day10_11_step8_two_by_two_verdict.csv     第三條完成標準的判決
            day10_11_step8_mixed_pair_cost.csv        推翻 #2 的代價（2.7 倍）
            day10_11_step8_cases.png
            day10_11_step9_body_requirements.csv      392 rows x 63 cols
                                                      （provenance 7 / sequence 5 /
                                                        segment 13 / knot 365 /
                                                        unresolved_transition 2）
                                                      <- Day 12 的交接物，只讀這一個
            day10_11_step9_pinned_endpoint_evidence.csv  端點 PINNED 的量測 + envelope 用量
            day10_11_step9_completion_criteria.csv       四條完成標準 + reader 稽核結果
            day10_11_step9_body_requirements.png         【從交付的 CSV 畫出來的】
```

**Step 4 讀進來但沒有重寫的 Day 6–7 檔案**

```text
day6-7/day6_7_step11r_sweep_trajectories.csv   3.8 MB，70 cells 的 hip 軌跡（Step 4 的主要輸入）
day6-7/day6_7_step11r_feasibility_sweep.csv    可行性與 L_transition
day6-7/day6_7_step12r_minimum_top_length.csv   頂面下限（只有 5 個 (h, theta) 組合）
```
