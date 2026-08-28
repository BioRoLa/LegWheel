# Hybrid Gait Day 6–7 修正版：Step 6.75 之後的收尾計畫

> **定位**：本文件不是重寫 Day 6–7 已完成的開發歷程，而是從目前已完成 **Step 6.75** 的位置重新收斂後續工作，讓 Day 6–7 可以在有限步驟內完成，並乾淨銜接 Day 8–9 的 swing trajectory 研究。
>
> **目前範圍**：single leg、2D sagittal plane、known rectangular obstacle、offline planning。  
> **暫不處理**：四腳協調、online replanning、ABAD、完整 dynamics / force feasibility、energy optimization。

---

# 1. 為什麼現在需要重新收斂 Day 6–7

原本 Day 6–7 從單純的：

```text
Can the right rim roll onto an obstacle?
```

逐漸延伸成：

```text
roll-up
→ retract to 17°
→ wheel reset
→ foot-rim recovery
→ re-extension
→ rolling-assisted swing
→ direct swing
→ motion classification
```

這些探索並沒有浪費，尤其 Step 5、6、6.5、6.75 幫助確認了：

1. 「成功滾上 obstacle」與「在 obstacle top 恢復 foot rim」是兩個不同問題。
2. 若要求全程 continuous contact 完成 foot-rim reset，可能需要很大的 rotation 與很長的 top distance。
3. Step 6.75 的 airborne reset 證明：必要時可以在 roll-up 後離地重新配置腿部。

但如果 Day 6–7 繼續同時處理所有 recovery branch，會和 Day 8–9 的 swing 問題重疊。

因此從現在開始，Day 6–7 重新聚焦成：

> **研究單腳是否能利用 continuous rim rolling 通過 rectangular obstacle 的 leading edge、top surface 與 trailing edge。**

也就是優先完成一個真正的：

```text
GROUND
→ RIGHT-RIM ROLL-UP
→ TOP ROLL
→ ROLL-DOWN
→ GROUND
```

primitive。

---

# 2. Step 6.75 在新版架構中的定位

Step 6.75 **保留，但不再當 Day 6–7 主流程的必要步驟**。

它的定位改成：

> **Exploratory recovery / future rolling-to-swing transition candidate**

也就是：

```text
如果未來 rolling 無法完整通過 obstacle，
可以從某個合法 rolling state liftoff，
再利用 airborne leg reconfiguration 接 Day 8–9 的 swing。
```

目前先不要繼續把 Step 6.75 擴充成：

```text
airborne reset
→ obstacle-top touchdown
→ 再次 liftoff
→ Bezier swing down
```

因為這會在 Day 6–7 過早引入完整 swing planning。

Day 8–9 會專門處理 swing，因此 Day 6–7 只需要保留：

```text
rolling end state / liftoff state
```

作為未來 swing 的接口即可。

---

# 3. 新版 Day 6–7 最終核心問題

Day 6–7 最後只回答三個問題。

## Q1 — Right rim 能不能真的滾上去？

不是 fixed-hip geometry reachability，而是包含：

```text
hip_x forward motion
+
continuous contact
+
front face / corner / top transition
+
collision-free geometry
```

的真正 rolling feasibility。

---

## Q2 — 上去後能不能沿 obstacle top 繼續滾並從 trailing edge 滾下來？

如果 obstacle 足夠長，不一定需要在 obstacle top 中途把 foot rim reset 到下面。

可以直接：

```text
roll-up
→ top roll
→ roll-down
```

因此需要新增 trailing-edge roll-down feasibility。

---

## Q3 — 哪些 obstacle / theta 組合可以完成這個 rolling primitive？

最後建立：

```text
obstacle height × theta_climb
→ rolling feasibility
```

必要時再加入：

```text
obstacle top length
```

但 top length 的意義改成：

> 是否有足夠空間完成 top rolling 並進入 trailing-edge roll-down，

而不是「是否有足夠距離完成 foot-rim wheel reset」。

---

# 4. 新版 Day 6–7 主流程

```text
STEP 6.75 已完成
      │
      │  保留為 exploratory result
      ▼
Step 7R
建立 trailing-edge roll-down
      ↓
Step 8R
串成完整 roll-up → top → roll-down trajectory
      ↓
Step 9R
重新做真正的 obstacle height × theta rolling feasibility sweep
      ↓
Step 10R
加入 obstacle top length / full-traversal feasibility
      ↓
Step 11R
定義 rolling-to-swing interface
      ↓
Day 6–7 FREEZE
      ↓
進入 Day 8–9：Bezier / hybrid swing
```

其中真正新增的 coding 工作只有 **Step 7R–11R**。

---

# 5. Step 7R — Trailing-Edge Roll-Down

## 研究目的

目前已經研究 leading edge：

```text
front face
→ leading corner
→ obstacle top
```

下一步要研究相反方向的 terrain transition：

```text
obstacle top
→ trailing corner
→ vertical drop side
→ lower ground
```

目標不是直接假設 roll-down 與 roll-up 完全對稱，而是實際用目前的 contact / rolling simulator 驗證。

## 起始條件

從一個已經穩定位於 obstacle top 的合法 rolling state 開始：

```text
hip_x
hip_z
theta
beta
active rim
alpha
contact point
```

並讓 hip 繼續往 `+x` 前進。

## 成功條件

```text
continuous legal rim contact
AND
passes obstacle trailing edge
AND
reaches lower ground
AND
no invalid penetration / collision
AND
theta / beta trajectory remains continuous
```

第一版不要求最後一定是 foot rim。

只要能從 obstacle top 以合法 rim contact 回到 lower ground，即可視為 roll-down success。

## Codex 指令

```text
請在目前 Day 6–7 的 continuous rolling simulator 上新增 TRAILING_EDGE_ROLL_DOWN。

背景：
目前已經能完成 right-rim leading-edge roll-up，並在 obstacle top 上持續 rolling。
現在不要處理 foot-rim reset，也不要使用 Step 6.75 airborne reset。
我要測試：如果 obstacle top 足夠長，是否可以直接沿 top 繼續 rolling，並通過 trailing edge 回到 lower ground。

請從一個合法 obstacle-top rolling state 開始。

每個 simulation step：
1. hip_x 繼續往 +x 前進。
2. 使用上一幀 theta / beta / contact state 作為 local continuation initial guess。
3. 允許 theta / beta 做連續的小幅調整。
4. 維持合法 rim-terrain contact。
5. 正確處理：
   - obstacle_top
   - trailing_corner
   - obstacle_back_face / drop-side interaction
   - lower_ground
6. 檢查：
   - link collision
   - geometry penetration
   - illegal rim jump
   - theta / beta discontinuity
7. 每幀保存：
   - phase
   - hip_x, hip_z
   - theta, beta
   - active_rim, alpha
   - contact_x, contact_z
   - terrain_surface
   - collision / penetration
   - accepted / failure_reason

請建立 phase：
TOP_ROLL
→ TRAILING_CORNER_TRANSITION
→ ROLL_DOWN
→ LOWER_GROUND_CONTACT

成功條件：
- trajectory 通過 trailing edge；
- 最後重新取得 lower ground 的合法 contact；
- 全程無 invalid collision / penetration；
- configuration 與 contact transition 連續。

注意：
- 不要假設 roll-down 是 roll-up trajectory 的簡單反轉或鏡像。
- 不要做 swing。
- 不要要求最後一定是 foot rim。
- 不要做 obstacle sweep。
- 先只完成一個人工確認正確的成功 case。

最後請提供：
1. trajectory visualization / animation
2. 每個 phase 的關鍵 frame
3. success / failure summary
4. 若失敗，清楚指出 failure phase 與 geometry reason
```

## 驗收

```text
至少一組 case：
obstacle top
→ trailing edge
→ lower ground

可以全程 continuous-contact roll-down。
```

---

# 6. Step 8R — 完整 Rolling Traversal Primitive

## 研究目的

把已完成的 leading-edge roll-up 與 Step 7R roll-down 串起來。

正式 primitive：

```text
APPROACH
→ FRONT_FACE_CONTACT
→ LEADING_CORNER_TRANSITION
→ TOP_ROLL
→ TRAILING_CORNER_TRANSITION
→ ROLL_DOWN
→ LOWER_GROUND_CONTACT
```

這一步才是 Day 6–7 的主要 motion result。

## 重要原則

不要在 top 中途強迫：

```text
theta = 17°
→ foot rim reset
→ re-extend
```

只要 rolling contact 可以自然通過整個 obstacle，就讓它繼續滾。

## Codex 指令

```text
請把目前已完成的 leading-edge roll-up 與新的 trailing-edge roll-down 串成一個完整 single-leg rolling obstacle traversal primitive。

希望流程：

APPROACH
→ FRONT_FACE_CONTACT
→ LEADING_CORNER_TRANSITION
→ TOP_ROLL
→ TRAILING_CORNER_TRANSITION
→ ROLL_DOWN
→ LOWER_GROUND_CONTACT

要求：
1. reuse 現有 roll-up 與 roll-down continuation logic，不要複製兩套 leg model。
2. hip_z 第一版固定，hip_x 為主要 forward progression variable。
3. theta / beta 可以依 contact continuation 做連續調整。
4. 每個 phase 都必須使用相同 terrain-aware collision / contact validation。
5. top 上不要執行 foot-rim reset。
6. 不要呼叫 airborne reset。
7. 不要呼叫 swing planner。
8. 保存完整 trajectory 與 phase transition frame。
9. 建立一個 reusable function，例如：

check_continuous_rolling_traversal(
    obstacle,
    initial_state,
    theta_climb,
    constraints
) -> RollingTraversalResult

RollingTraversalResult 至少包含：
- feasible
- failure_phase
- failure_reason
- trajectory
- roll_up_success
- top_roll_success
- roll_down_success
- final_state
- minimum_collision_margin

成功定義：
只有從 obstacle 前方一路 continuous rolling 到 obstacle 後方 lower ground 才算 full traversal success。

請先只驗證一個已知可行 obstacle，不要開始 sweep。
```

## 驗收

得到一條：

```text
ground
→ roll-up
→ top
→ roll-down
→ ground
```

完整動畫 / trajectory。

---

# 7. Step 9R — 重新做 Height × Theta 真正 Rolling Feasibility Sweep

## 為什麼要「重新做」

早期 Step 4 的 theta sweep 主要用於找 candidate configuration。

真正的 Day 6–7 feasibility 應該使用：

```text
hip_x motion
+
continuous contact
+
leading edge
+
top
+
trailing edge
```

的完整 simulator。

因此正式 result 應該重新 sweep。

## 第一張主要結果

```text
obstacle height × theta_climb
→ FULL_CONTINUOUS_ROLLING_SUCCESS / FAILURE
```

同時保留 failure phase：

```text
ROLL_UP_FAIL
TOP_ROLL_FAIL
ROLL_DOWN_FAIL
```

這樣可以知道到底是「爬不上去」還是「上去了但下不來」。

## Codex 指令

```text
請使用目前完整的 check_continuous_rolling_traversal() 做 Day 6–7 正式 obstacle-height × theta-climb feasibility sweep。

這次不要使用早期 fixed-hip theta feasibility 當最終結果。

固定：
- gamma = 0
- hip_z = fixed
- initial approach condition = fixed
- obstacle top length 先設成足夠長，使 trailing-edge traversal 有足夠 top rolling space

掃描：
- obstacle_height
- theta_climb

對每一組 case 執行完整：

roll-up
→ top roll
→ roll-down
→ lower ground

輸出 CSV 至少包含：
- obstacle_height
- theta_climb
- feasible
- roll_up_success
- top_roll_success
- roll_down_success
- failure_phase
- failure_reason
- final_rim
- final_theta
- final_beta
- total_hip_progress
- minimum_collision_margin

產生：
1. obstacle height × theta_climb full rolling feasibility map
2. 每個 obstacle height 的 feasible theta range
3. obstacle height → minimum feasible theta_climb
4. failure-phase map（可選，但 raw data 必須保留）

注意：
- full traversal 成功才標 feasible。
- 只 roll-up 成功不能算 full rolling feasible。
- 不要加入 swing。
- 不要加入 Step 6.75 recovery。
```

## 驗收

至少得到：

```text
height × theta feasibility map
```

並可以回答：

> 對某個 obstacle height，哪些 theta 能讓 single leg 以 continuous rolling 完整通過？

---

# 8. Step 10R — Obstacle Top Length 對 Full Rolling Traversal 的影響

## 新的 top-length 定義

舊版把 `L_top` 主要拿來判斷：

```text
是否有足夠空間完成 foot-rim reset
```

新版不再以此為 Day 6–7 主問題。

現在 `L_top` 的問題是：

> **從 leading-edge roll-up 的 exit state，到 trailing-edge roll-down 的 entry state，中間是否有足夠 top distance 讓 rolling trajectory 接得起來？**

太短時可能出現：

```text
剛離開 leading corner
→ 還沒進入穩定 top rolling
→ 就遇到 trailing corner
```

因此可能無法完成 continuous rolling traversal。

## Codex 指令

```text
請在 full continuous rolling traversal 上加入 obstacle top length sweep。

研究問題改為：
obstacle top 是否有足夠長度，讓 leading-edge roll-up 的 exit state 可以連續銜接到 trailing-edge roll-down？

請固定幾組代表性的：
- obstacle_height
- feasible theta_climb

然後掃描：
- obstacle_top_length

每一組都執行完整：
roll-up
→ top
→ roll-down

記錄：
- obstacle_height
- theta_climb
- top_length
- full_traversal_success
- roll_up_success
- roll_down_success
- failure_phase
- failure_reason
- top_contact_distance
- leading_edge_exit_state
- trailing_edge_entry_state

找出：
minimum_top_length_for_continuous_rolling

注意：
1. 不要再用 foot-rim reset distance L_reset 作為這一步的主要 criterion。
2. 必須用實際 geometry simulation 判斷。
3. obstacle 很短時，leading corner 與 trailing corner 的 transition 可能互相影響，這本身就是重要結果。
4. 不要在失敗 case 自動呼叫 swing；只記錄 rolling failure state。
```

## 驗收

可以得到：

```text
(height, theta)
→ minimum obstacle top length for full continuous rolling traversal
```

或至少：

```text
top length vs full rolling feasibility
```

---

# 9. Step 11R — 建立 Day 8–9 所需的 Rolling-to-Swing Interface

這一步**不做 swing trajectory**。

它只負責讓 Day 6–7 的輸出能乾淨交給 Day 8–9。

## 為什麼需要

未來可能有兩種情況：

### Full rolling 可行

```text
roll-up
→ top
→ roll-down
→ ground
```

不需要 swing。

### Full rolling 不可行，但 rolling 已經產生有利 progress

例如：

```text
roll-up 成功
→ 到 obstacle top
→ 無法安全 roll-down
```

這時 Day 6–7 只需要輸出：

```text
最後一個適合 liftoff 的合法 state
```

Day 8–9 再從這個 state 生成 Bezier swing。

Step 6.75 已經證明 airborne reconfiguration 是可探索的，因此它可以作為這個 interface 的參考，但不要在 Day 6–7 把後續 swing 寫死。

## 建議資料結構

```text
SwingEntryState
- hip_x
- hip_z
- theta
- beta
- gamma
- active_rim
- contact_x
- contact_z
- terrain_surface
- obstacle_height
- obstacle_top_length
- source_phase
- recommended_liftoff
```

另外保留：

```text
rolling_progress_before_swing
failure_phase
failure_reason
```

## Codex 指令

```text
請為 Day 6–7 建立一個乾淨的 rolling-to-swing interface，但不要生成任何 Bezier swing trajectory。

目的：
當 full continuous rolling traversal 失敗時，保留一個可以交給 Day 8–9 HybridSwing planner 的合法 start state。

請建立類似：

SwingEntryState

至少包含：
- hip_x, hip_z
- theta, beta, gamma
- active_rim, alpha
- contact_x, contact_z
- terrain_surface
- obstacle_height
- obstacle_top_length
- source_phase
- rolling_progress
- failure_phase
- failure_reason

請在 RollingTraversalResult 中額外提供：
- swing_entry_available
- swing_entry_state

選擇原則：
1. 只從已通過 terrain-aware collision validation 的合法 frame 選。
2. 優先保留 full rolling failure 前最後一個穩定、可 liftoff 的 state。
3. 如果 roll-up 本身完全不可行，可以回傳 swing_entry_available = false，讓 Day 8–9 從原始 stance / approach state 做 direct swing。
4. Step 6.75 的 airborne reset 可以作為「這類 state 可進入 airborne motion」的開發參考，但這一步不要呼叫 Step 6.75 trajectory，也不要寫 Bezier swing。
5. 不要現在比較 direct swing 與 rolling-assisted swing cost。

最後請提供：
- 一個 full rolling success case：不需要 swing entry
- 一個 roll-up 後 full traversal failure case：有 swing entry
- 一個 roll-up failure case：回傳 failure，交由之後 direct swing 處理
```

## 驗收

Day 8–9 可以直接拿：

```text
SwingEntryState
```

作為 swing start state，而不需要重新解析 Day 6–7 simulator 的內部狀態。

---

# 10. Day 6–7 到這裡就 Freeze

完成 Step 11R 後，不再在 Day 6–7 做：

```text
Bezier swing
direct swing
rolling-assisted swing trajectory
swing touchdown optimization
rolling vs swing cost comparison
energy comparison
完整三類 motion selector
```

這些移到後面。

Day 6–7 的正式輸出只有：

## Output A — Continuous Rolling Primitive

```text
roll-up
→ top roll
→ roll-down
```

## Output B — Rolling Feasibility Evaluator

```text
obstacle geometry + theta
→ full rolling feasible / infeasible
```

## Output C — Feasibility Data

```text
height × theta
top length × feasibility
failure phase
```

## Output D — Swing Interface

```text
rolling failure
→ SwingEntryState
```

這樣 Day 8–9 可以直接接手 airborne trajectory。

---

# 11. Day 8–9 的銜接方式

Day 8–9 不需要重新處理 rolling geometry。

它只需要接受兩種 start state。

## Case A — Direct Swing

Day 6–7 判定：

```text
roll-up unavailable
```

Day 8–9 從原始 stance / approach configuration 開始 swing。

```text
START_STANCE
→ Bezier swing
→ target foothold
```

## Case B — Rolling-Assisted Swing

Day 6–7 判定：

```text
rolling made useful progress
but full rolling traversal failed
```

Day 8–9 從：

```text
SwingEntryState
```

開始：

```text
ROLLING
→ LIFTOFF
→ Bezier swing
→ target foothold / lower ground
```

因此 Day 8–9 的 swing generator 最重要的 generalization 是：

> **swing start point 不一定是傳統 foot-rim stance point，而可以是任意合法 rolling exit / liftoff state。**

這正好對應 Hybrid Swing 與一般 walking Bezier swing 的差異。

---

# 12. Day 6–7 最終建議 Research Figures

完成新版步驟後，優先保留以下結果。

## Figure A — Continuous Rolling Sequence

```text
front face
→ leading corner
→ top
→ trailing corner
→ lower ground
```

這會是最直觀的 rolling primitive figure。

## Figure B — Height × Theta Full-Traversal Feasibility

不是單純「碰得到 top」，而是：

```text
ground → obstacle → ground
```

完整 rolling 成功。

## Figure C — Top Length Feasibility

顯示：

```text
obstacle top length
→ full continuous rolling success / failure
```

並標出 minimum usable top length。

## Figure D — Rolling Failure State / Swing Entry

展示一個：

```text
rolling can help
but cannot finish
```

的 case，並標出交給 Day 8–9 的 `SwingEntryState`。

這張 figure 未來可以用來引出 rolling-assisted swing。

---

# 13. Step 6.5 / 6.75 怎麼保留在 Paper 發想筆記

不要刪除舊結果。

可以保留這段研究脈絡：

```text
Initially, recovery on the obstacle top was investigated by retracting
the leg-wheel to the wheel-like configuration and restoring the foot-rim
contact while maintaining terrain contact.

This analysis revealed that continuous-contact reset may require a large
rotation and substantial top-surface travel.

An airborne reset was therefore also explored, demonstrating that a
legal rolling state can transition to an airborne leg reconfiguration.

These observations motivated a cleaner separation:
continuous rolling is used when the terrain permits full traversal,
whereas airborne swing is handled as a separate primitive when rolling
alone is insufficient.
```

也就是 Step 6.5 / 6.75 的價值變成：

> **幫助建立 rolling 與 swing 應該分開處理的設計理由。**

---

# 14. 接下來實際執行順序

從你現在的進度，建議直接照這個順序：

```text
目前：Step 6.75 完成
        ↓
Step 7R
先做 trailing-edge roll-down 單一 case
        ↓
【人工看動畫，確認 contact transition 合理】
        ↓
Step 8R
串成完整 roll-up → top → roll-down
        ↓
【Checkpoint 1：完整 continuous rolling primitive】
        ↓
Step 9R
height × theta full-traversal sweep
        ↓
【Checkpoint 2：第一張正式 feasibility map】
        ↓
Step 10R
top length feasibility
        ↓
【Checkpoint 3：知道什麼 terrain 能一路滾完】
        ↓
Step 11R
輸出 SwingEntryState
        ↓
DAY 6–7 FREEZE
        ↓
DAY 8–9
開始做 HybridSwing / Bezier trajectory
```

---

# 15. Day 6–7 新的完成標準

完成以下五件事即可停止 Day 6–7：

- [ ] **Trailing-edge roll-down** 至少一個成功 case。
- [ ] **完整 roll-up → top → roll-down** continuous rolling trajectory。
- [ ] **Obstacle height × theta** full-traversal feasibility map。
- [ ] 至少初步分析 **obstacle top length** 對 full rolling 的影響。
- [ ] 定義並輸出可交給 Day 8–9 的 **SwingEntryState**。

以下不再是 Day 6–7 必做：

- [ ] foot rim 必須在 obstacle top reset 到下方
- [ ] Step 6.75 後再落在 obstacle top
- [ ] Step 6.75 後再接第二次 Bezier swing
- [ ] direct swing implementation
- [ ] rolling-assisted swing implementation
- [ ] 三策略 cost comparison
- [ ] energy optimization

---

# 16. 一句話版本

新版 Day 6–7 的研究任務可以縮成：

> **先找出 leg-wheel 在什麼 obstacle geometry 與 leg configuration 下，可以利用 continuous rim contact 完成「滾上去、滾過去、滾下來」；若 rolling 無法完成，只輸出最後的合法 rolling exit state，將真正的 airborne repositioning 留給 Day 8–9 的 Hybrid Swing。**

這樣 Day 6–7 與 Day 8–9 的責任邊界會非常清楚：

```text
Day 6–7:
terrain-contact rolling feasibility

Day 8–9:
airborne swing trajectory generation

Later:
offline planner chooses / combines them
```
