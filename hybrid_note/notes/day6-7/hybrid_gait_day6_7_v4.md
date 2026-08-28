# Hybrid Gait Day 6–7 最終收斂版 Note
## Right-Rim Up → Wheel-Mode Transition → Left-Rim Down

> **目前進度基準：已完成到 Step 6.75。**
>
> 這份 note 的目的不是重寫 Step 1–6.75，而是把目前已完成的工作、研究過程中的探索，以及接下來真正需要完成的 Day 6–7 主線整理清楚，避免繼續增加 recovery / swing branch。
>
> **Day 6–7 從現在開始只研究 rolling capability。**
>
> Day 8–9 才研究 airborne / Bezier swing。

---

# 1. Day 6–7 最終要回答什麼？

Day 6–7 的核心問題收斂成：

> **新的 leg-wheel morphology 是否能利用不同 rim 與 terrain 的連續接觸，完成 rectangular obstacle 的 rolling traversal？**

目前選定的主要 motion definition 是：

```text
GROUND
  ↓
RIGHT-RIM APPROACH
  ↓
RIGHT-RIM FRONT-FACE CONTACT
  ↓
RIGHT-RIM LEADING-CORNER ROLL-UP
  ↓
RIGHT-RIM TOP CONTACT
  ↓
RETRACT θ → 17°
  ↓
WHEEL-MODE FORWARD ROLL
  ↓
LEFT_RIM_READY
  ↓
LEFT-RIM TRAILING-EDGE ROLL-DOWN
  ↓
LOWER GROUND
```

簡化成一句話：

```text
RIGHT RIM UP
→ θ = 17° WHEEL TRANSITION
→ LEFT RIM DOWN
```

這是 Day 6–7 接下來唯一的主線。

---

# 2. Day 6–7 已經完成什麼？

目前已完成的底層與 prototype 包含：

- single-leg 2D geometry
- flat ground + one rectangular obstacle
- gamma = 0
- terrain-aware contact query
- right-rim front-face planned contact
- front / leading-corner / top contact continuation
- sampled rim / link collision query
- fixed-theta roll-up
- theta candidate sweep
- hip_x forward rolling
- contact-point-based top-roll distance
- roll-up 後 retract 到 theta = 17°
- Step 6.5 signed no-slip continuous-contact recovery exploration
- Step 6.75 airborne retract / reset exploration
- Step 6.75 foot-rim touchdown on obstacle top
- CSV / plot / animation / regression tests

目前已經能證明：

```text
lower ground
→ right-rim contact obstacle front face
→ leading corner
→ obstacle top
```

不是只有 fixed-hip pose reachability，而是已經加入 hip_x forward motion 與 contact continuation。

---

# 3. Step 6.5 / Step 6.75 現在怎麼定位？

## Step 6.5

Step 6.5 探索：

```text
保持 terrain contact
→ retract / reset
→ 嘗試恢復 foot rim
```

這讓我們知道 continuous-contact foot-rim reset 可能需要很大的 rotation 與很長的 obstacle top。

## Step 6.75

Step 6.75 探索：

```text
right-rim roll-up
→ top roll
→ liftoff
→ airborne retract theta
→ airborne reset beta
→ foot-rim touchdown on top
```

它證明在目前 single-leg 2D kinematic / collision model 下，存在合法的 airborne reset path。

## 從現在開始

兩者都保留為：

```text
exploratory recovery results
```

但**不再繼續擴充成 Day 6–7 的主流程**。

尤其不要現在做：

```text
Step 6.75
→ touchdown on top
→ 再 liftoff
→ Bezier swing down
```

這會和 Day 8–9 重疊。

---

# 4. 為什麼現在選 Right-Up → Left-Down？

原本曾考慮：

```text
right rim roll-up
→ foot rim reset
→ re-extend
```

但如果 obstacle 本身允許持續 rolling，沒有必要在 obstacle top 中途強迫恢復 foot-rim stance。

因此目前更自然的 rolling primitive 是：

```text
right rim 負責 leading edge climb
        ↓
theta 收回 wheel-like 17°
        ↓
在 top 上利用 rolling 改變 rim phase
        ↓
left rim 進入 descent-ready configuration
        ↓
left rim 負責 trailing edge descent
```

這樣 Day 6–7 的研究重點就維持在：

> **terrain-contact rolling capability**

而不是把 swing / recovery / stance loading 全部混進來。

---

# 5. `theta = 17°` 的角色

新版流程裡：

```text
theta = 17°
```

**不是為了把 foot rim 轉到底下。**

它的角色是：

> **將 climbing configuration 收回 wheel-like configuration，使 leg-wheel 可以在 obstacle top 上以較標準的 rolling geometry 往前轉動，直到 left rim 到達適合下降的 phase。**

因此：

```text
RIGHT-RIM TOP STATE
        ↓
retract theta
        ↓
θ = 17°
        ↓
wheel-mode forward roll
        ↓
LEFT_RIM_READY
```

這個 transition 是接下來要正式建立的東西。

---

# 6. 接下來只剩 6 個步驟

```text
目前：Step 6.75 已完成
        ↓
Step 7R
Right-rim roll-up end → retract θ to 17° on top
        ↓
Step 8R
θ=17° wheel-mode roll → LEFT_RIM_READY
        ↓
Step 9R
LEFT_RIM_READY → left-rim roll-down
        ↓
Step 10R
串成完整 RIGHT-UP → LEFT-DOWN trajectory
        ↓
Step 11R
正式 rolling feasibility sweep
        ↓
Step 12R
Top-length / transition-distance analysis
        ↓
DAY 6–7 FREEZE
        ↓
DAY 8–9 Swing
```

---

# 7. Step 7R — Roll-Up End → Retract θ to 17°

## 目的

從真正成功的 right-rim roll-up final frame 開始。

不是重新手動指定一個 top pose。

希望：

```text
RIGHT-RIM TOP STATE
→ continuous retract
→ θ = 17°
```

並且仍保持 obstacle top 上的合法 contact。

## 變數

第一版：

```text
gamma = 0
hip_z = fixed
hip_x = allowed to progress in +x
theta = decreases toward 17°
beta = local continuous correction
```

## 成功條件

```text
theta reaches 17°
AND
terrain surface remains obstacle_top
AND
no invalid collision / penetration
AND
theta / beta / hip trajectory remains continuous
```

不要求：

```text
foot rim ready
left rim ready
roll-down
```

---

## 給 Codex 的 Step 7R 指令

```text
請在目前 Day 6–7 single-leg rolling simulator 上實作新的 Step 7R：
RETRACT_TO_WHEEL_ON_TOP。

請先閱讀目前 Day 6–7 progress note 與既有
single_leg_rolling_scene_2d.py，
並 reuse 現有 Step 4.5 right-rim roll-up final frame。

研究意圖：
我現在不再要求在 obstacle top 把 foot rim reset 到下面。
Step 7R 只負責：
right rim 已成功滾上 obstacle top 後，
保持合法 obstacle-top contact，
同時把 theta 連續收回到 17 degrees，
準備下一步 wheel-mode rolling。

要求：

1. 起點必須直接使用真正 roll-up 成功 trajectory 的 final frame：
   - hip_x
   - hip_z
   - theta
   - beta
   - active rim
   - alpha
   - contact point

2. 不要重新手動指定一個理想 top pose。

3. theta 從目前 theta_climb 逐步下降到 17 deg。

4. hip_z 第一版固定。

5. hip_x 不要鎖死。
   允許 hip 在 +x 方向小幅持續前進，以維持合理 rolling/contact continuation。

6. beta 可以以上一幀為 initial guess 做局部連續修正。

7. 每一幀都呼叫目前 terrain-aware contact / collision query。

8. 每一幀檢查：
   - valid terrain contact
   - terrain surface == obstacle_top
   - active rim / alpha
   - rim/link collision
   - penetration
   - joint limits
   - theta / beta continuity

9. 每一幀保存：
   - step
   - phase
   - hip_x, hip_z
   - theta, beta
   - active_rim, alpha
   - contact_x, contact_z
   - terrain_surface
   - collision
   - penetration
   - accepted
   - failure_reason

10. 成功條件：
    theta reaches 17 deg
    AND contact remains legal on obstacle top
    AND no invalid collision / penetration
    AND trajectory is continuous.

11. 不要做 foot-rim reset。

12. 不要做 LEFT_RIM_READY search。

13. 不要做 roll-down。

14. 不要做 swing / airborne motion。

15. 不要覆蓋 Step 6.5 / Step 6.75 baseline；
    新增獨立 Step 7R result / CSV / visualization。

請提供：
- Step 7R animation
- start / middle / final key frames
- CSV
- success / failure summary
- 若失敗，指出第一個 failure frame 與原因
```

## Step 7R 驗收

只要得到：

```text
right-rim top state
→ θ continuously retracts
→ θ = 17°
```

就完成。

---

# 8. Step 8R — Wheel Mode → LEFT_RIM_READY

## 目的

從 Step 7R 的：

```text
theta = 17°
```

final frame 開始。

保持 wheel-like configuration，繼續往 +x rolling，直到 left rim 到達適合下一步下降的 configuration。

這一步是：

```text
RIGHT-RIM SIDE
→ wheel rolling
→ LEFT-RIM SIDE
```

不是 foot-rim reset。

---

## LEFT_RIM_READY 第一版定義

第一版不要做太複雜。

可以先定義：

```text
theta == 17°
AND
left rim is the valid / intended obstacle-top contact rim
AND
no collision / penetration
AND
configuration is geometrically valid for trailing-edge approach
```

如果需要 tolerance，可以放在 config 中，不要散落 magic numbers。

---

## 需要量測

這一步很重要的輸出是：

```text
required_beta_rotation
required_hip_forward_distance
required_contact_forward_distance
```

並定義：

```text
L_transition
```

為：

> 從 right-rim roll-up exit / Step 7R transition 開始，到 LEFT_RIM_READY 所需要的實際 obstacle-top forward distance。

---

## 給 Codex 的 Step 8R 指令

```text
請接續 Step 7R，實作新的 Step 8R：
WHEEL_MODE_TO_LEFT_RIM_READY。

起點：
直接使用 Step 7R 成功後的 final frame。

研究意圖：
theta 已經收回 17 degrees。
現在我要讓 leg-wheel 在 obstacle top 上繼續往 +x rolling，
直到 left rim 進入適合 trailing-edge roll-down 的 configuration。

注意：
這一步不是 foot-rim reset。
不要使用 foot_rim_ready 作為終止條件。

要求：

1. theta 固定在 17 deg。

2. hip_z 固定。

3. hip_x 持續往 +x 前進。

4. beta 按目前 forward rolling direction 連續更新。

5. 使用 signed rolling / no-slip semantics；
   不要出現 beta rotation direction 與 contact forward progress 互相矛盾的情況。

6. 每一步使用上一幀作為 continuation initial guess。

7. 每一步追蹤：
   - hip_x, hip_z
   - theta, beta
   - active_rim
   - alpha
   - contact_x, contact_z
   - terrain_surface
   - hip forward displacement
   - contact forward displacement
   - collision / penetration

8. 新增 LEFT_RIM_READY 判斷。

9. 第一版 LEFT_RIM_READY 至少要求：
   - theta == 17 deg
   - left rim 為合法 obstacle-top contact / intended descent candidate
   - no invalid collision / penetration
   - joint limits valid
   - configuration 可作為下一步 trailing-edge descent 的起點

10. 找到 LEFT_RIM_READY 後停止。

11. 計算並輸出：
    - required_beta_rotation
    - required_hip_forward_distance
    - required_contact_forward_distance

12. 將從 transition start 到 LEFT_RIM_READY 的實際 top progress 記為 L_transition。

13. 不要做 trailing-edge roll-down。

14. 不要做 swing。

15. 不要 fallback 到 Step 6.75。

請提供：
- animation：
  right-rim top / wheel transition
  → theta=17
  → forward wheel roll
  → LEFT_RIM_READY
- LEFT_RIM_READY final frame visualization
- CSV
- L_transition summary
```

## Step 8R 驗收

程式可以回答：

> **從 roll-up 後的 wheel-mode state，還需要往前滾多少，left rim 才準備好下降？**

---

# 9. Step 9R — Left-Rim Trailing-Edge Roll-Down

## 目的

這是目前真正還沒有驗證的核心 motion。

從：

```text
LEFT_RIM_READY
```

開始，讓 hip 繼續往 +x。

希望建立：

```text
left rim on obstacle top
→ trailing corner
→ descending-side contact
→ lower ground
```

這可以視為 roll-up 的「方向對應版本」，但不能直接假設完全 mirror。

---

## 第一版不要做什麼

不要掃：

```text
theta × beta × rim × height
```

Step 9R 第一版只驗證一個成功 case。

起始 configuration 直接固定為 Step 8R 找到的 `LEFT_RIM_READY`。

---

## 成功條件

```text
starts from LEFT_RIM_READY
AND
passes trailing edge
AND
returns to lower ground
AND
contact transition remains geometrically valid
AND
no invalid collision / penetration
```

第一版最後不強迫：

```text
foot rim
theta stance
```

只要成功回到 lower ground 即可。

---

## 給 Codex 的 Step 9R 指令

```text
請新增 Step 9R：
LEFT_RIM_TRAILING_EDGE_ROLL_DOWN。

起點：
必須直接使用 Step 8R 的 LEFT_RIM_READY final frame。

研究意圖：
上 obstacle 時使用 right rim。
下 obstacle 時使用 left rim。
我要測試 left rim 是否可以利用 trailing edge / corner，
以 continuous terrain contact 滾回 lower ground。

不要直接把 right-rim roll-up trajectory 做 mirror copy。
請使用目前 terrain-aware geometry / contact continuation 實際求解。

要求：

1. 起點直接使用：
   - Step 8R final hip pose
   - theta = 17 deg
   - beta
   - left-rim contact
   - alpha
   - contact point

2. hip_x 繼續往 +x 前進。

3. hip_z 第一版維持目前 Day 6–7 assumption；
   不要自行新增 body vertical planner。

4. theta / beta 允許依 descent contact continuation 做連續調整。
   如果第一版可以固定 theta=17 完成，優先先測固定 theta；
   若固定 theta 幾何上失敗，再把 theta adjustment 明確列成第二個測試，而不是偷偷改。

5. 每一步使用上一幀 state 作 local initial guess。

6. terrain surface 至少正確處理：
   - obstacle_top
   - trailing_corner
   - obstacle_back_face / descending face
   - lower_ground

7. 每一步檢查：
   - intended left-rim contact
   - active rim / alpha
   - link collision
   - rim penetration
   - illegal contact jump
   - joint limits
   - theta / beta continuity

8. 建議 phase：
   LEFT_RIM_TOP_APPROACH
   → TRAILING_CORNER_TRANSITION
   → LEFT_RIM_ROLL_DOWN
   → LOWER_GROUND_CONTACT

9. 每幀保存：
   - phase
   - hip_x, hip_z
   - theta, beta
   - active_rim, alpha
   - contact_x, contact_z
   - terrain_surface
   - collision
   - penetration
   - accepted
   - failure_reason

10. 成功條件：
    - passes trailing edge
    - obtains legal lower-ground contact
    - no invalid collision / penetration
    - trajectory remains continuous

11. 第一版不要要求最後一定是 foot rim。

12. 不要 re-extend stance。

13. 不要做 swing。

14. 不要做 obstacle sweep。

15. 不要 fallback 到 Step 6.75。

請先只找出一個人工可確認合理的成功 case。

請提供：
- full Step 9R animation
- top / corner / descent / lower-ground key frames
- CSV
- success / failure summary
- failure 時清楚指出是：
  contact loss、
  corner transition failure、
  collision、
  joint limit、
  或其他 geometry reason
```

## Step 9R 驗收

至少一個：

```text
LEFT_RIM_READY
→ trailing corner
→ lower ground
```

成功 case。

這會是目前 Day 6–7 最大的新 checkpoint。

---

# 10. Step 10R — 串成完整 Right-Up → Left-Down

## 目的

到這裡不要再增加新 motion。

只把已有的：

```text
right-rim roll-up
+
retract to 17
+
wheel-mode left-rim transition
+
left-rim roll-down
```

串成一條 trajectory。

---

## 完整 phase

```text
APPROACH
→ RIGHT_RIM_FRONT_CONTACT
→ RIGHT_RIM_ROLL_UP
→ RIGHT_RIM_TOP
→ RETRACT_TO_WHEEL
→ WHEEL_MODE_TOP_ROLL
→ LEFT_RIM_READY
→ LEFT_RIM_TRAILING_TRANSITION
→ LEFT_RIM_ROLL_DOWN
→ LOWER_GROUND_CONTACT
```

---

## 給 Codex 的 Step 10R 指令

```text
請新增 Step 10R：
FULL_RIGHT_UP_LEFT_DOWN_TRAVERSAL。

現在不要新增任何新的 recovery strategy。
只把目前已驗證的 stage 串成一個 reusable full traversal。

流程必須是：

APPROACH
→ RIGHT_RIM_FRONT_CONTACT
→ RIGHT_RIM_ROLL_UP
→ RIGHT_RIM_TOP
→ RETRACT_TO_WHEEL
→ WHEEL_MODE_TOP_ROLL
→ LEFT_RIM_READY
→ LEFT_RIM_TRAILING_TRANSITION
→ LEFT_RIM_ROLL_DOWN
→ LOWER_GROUND_CONTACT

要求：

1. reuse 現有 Step 4.5 / Step 7R / Step 8R / Step 9R functions。

2. 不要複製 leg geometry / terrain contact logic。

3. 所有 phase 使用同一套 terrain-aware collision / contact validation。

4. phase 之間必須直接傳遞上一個 stage 的 final state；
   不允許手動 teleport 到下一個理想 pose。

5. 建立 reusable API，例如：

check_right_up_left_down_traversal(
    obstacle,
    initial_state,
    theta_climb,
    constraints
) -> RollingTraversalResult

6. RollingTraversalResult 至少包含：
   - feasible
   - failure_stage
   - failure_reason
   - trajectory
   - roll_up_success
   - retract_success
   - left_rim_ready_success
   - roll_down_success
   - L_transition
   - final_state
   - minimum_collision_margin

7. full_success 的定義：
   只有從 obstacle 前方 lower ground
   完成 right-rim up
   → wheel transition
   → left-rim down
   → 回到 obstacle 後方 lower ground
   才算 success。

8. 不要做 Step 6.75 airborne reset。

9. 不要做 Bezier swing。

10. 不要做 direct swing。

11. 不要做 sweep。

請先用一個已知可行 rectangular obstacle，
產生完整 animation 與 trajectory CSV。

animation 請清楚標 phase、active rim、theta、beta、contact surface。
```

## Step 10R 驗收

得到第一條完整：

```text
GROUND
→ RIGHT RIM UP
→ θ=17° TRANSITION
→ LEFT RIM DOWN
→ GROUND
```

trajectory。

做到這裡，**rolling primitive 本身完成。**

---

# 11. Step 11R — 正式 Rolling Feasibility Sweep

## 先不要把問題弄成太多維

Day 6–7 第一張正式 quantitative result 先只掃：

```text
obstacle height
×
theta_climb
```

其餘先固定。

---

## 為什麼早期 theta sweep 不算最後結果？

早期 Step 4 的 sweep 對研究開發很有用，但 final rolling feasibility 應使用包含：

```text
hip_x motion
continuous contact
right-rim roll-up
wheel transition
left-rim roll-down
```

的完整 simulator。

---

## 給 Codex 的 Step 11R 指令

```text
請使用 Step 10R 的
check_right_up_left_down_traversal()
建立 Day 6–7 正式 obstacle-height × theta-climb feasibility sweep。

這次不要使用早期 fixed-hip Step 4 theta sweep 當 final result。

固定：
- gamma = 0
- hip_z = fixed
- initial approach condition = fixed
- obstacle top length 先設為足夠長，
  讓 right-up → wheel transition → left-down 有足夠空間完成

掃描：
- obstacle_height
- theta_climb

每一組都執行完整：

right-rim roll-up
→ retract to 17
→ wheel-mode top roll
→ left-rim ready
→ left-rim roll-down
→ lower ground

輸出 CSV 至少包含：
- obstacle_height
- theta_climb
- feasible
- roll_up_success
- retract_success
- left_rim_ready_success
- roll_down_success
- failure_stage
- failure_reason
- L_transition
- final_theta
- final_beta
- minimum_collision_margin

產生：
1. obstacle height × theta_climb full-traversal feasibility map
2. 每個 obstacle height 的 feasible theta range
3. obstacle height → minimum feasible theta_climb

failure stage 至少分：
- ROLL_UP_FAIL
- RETRACT_FAIL
- LEFT_RIM_TRANSITION_FAIL
- ROLL_DOWN_FAIL

注意：
- 只有完整回到 lower ground 才標 full traversal feasible。
- raw result 必須保留。
- 不要加入 swing fallback。
- 不要加入 Step 6.75。
- 不要做 energy optimization。
```

## Step 11R 驗收

得到：

```text
obstacle height × theta_climb
→ full right-up-left-down rolling feasibility
```

---

# 12. Step 12R — Top Length / Transition Distance

## 這一步的定位

這一步不是為了讓 planner 一定要「預先知道 obstacle 多長才敢 roll-up」。

Day 6–7 的主要 morphology capability 仍然是：

```text
upward edge roll-up
downward edge roll-down
```

`L_top` 分析只用來回答：

> **如果希望把 right-up 與 left-down 串成一個完整 continuous rectangular-obstacle rolling demonstration，中間至少需要多少 top distance 完成 rim-side transition？**

所以它是：

```text
transition requirement / characterization
```

不是 roll-up 的必要前置判斷。

---

## 核心量

```text
L_transition
```

定義：

> 從 right-rim roll-up exit state 開始，到 LEFT_RIM_READY 所需要的實際 obstacle-top forward distance。

另外可以記：

```text
leading_edge_margin
trailing_edge_entry_margin
```

---

## 給 Codex 的 Step 12R 指令

```text
請做 Day 6–7 最後一個 characterization：
TOP_LENGTH_AND_TRANSITION_DISTANCE。

目的不是把 obstacle length 變成「是否允許開始 roll-up」的必要條件。

我要量化的是：
right-rim roll-up 成功後，
從 roll-up exit state
→ retract theta to 17
→ wheel-mode forward roll
→ LEFT_RIM_READY
實際需要多少 obstacle-top forward distance。

請：

1. 使用 Step 10R 的完整 traversal simulator。

2. 固定幾組代表性的：
   - obstacle_height
   - feasible theta_climb

3. 量測每組 case 的：
   - retract forward distance
   - wheel-mode forward distance
   - total L_transition
   - leading-edge margin
   - trailing-edge entry margin

4. 如果要 sweep obstacle_top_length，
   請把它定位成：
   「full right-up-left-down continuous traversal 是否有足夠 transition space」
   而不是「能不能開始 roll-up」。

5. 輸出：
   - obstacle_height
   - theta_climb
   - L_transition
   - obstacle_top_length
   - full_traversal_success
   - failure_stage
   - failure_reason

6. 找出：
   minimum_top_length_for_full_right_up_left_down_demo

7. 不要使用舊的 foot-rim L_reset criterion。

8. 不要 fallback 到 Step 6.75。

9. 不要呼叫 swing。

10. 不要做 planner motion selection。

請另外在結果說明中清楚區分：

A. local upward-edge roll-up feasibility
B. local downward-edge roll-down feasibility
C. full rectangular-obstacle traversal transition-length requirement

避免把 C 誤解成 A 的必要先驗條件。
```

## Step 12R 驗收

得到：

```text
L_transition
```

以及一個簡單的：

```text
top length vs full traversal feasibility
```

characterization。

---

# 13. Day 6–7 到哪裡就必須停？

完成 Step 12R 後：

```text
DAY 6–7 FREEZE
```

不要再新增：

- foot-rim top reset method
- 第三種 roll-down method
- Step 6.75 + second touchdown + second liftoff
- Bezier down
- direct swing
- rolling-assisted swing
- roll-vs-swing cost
- energy optimization
- four-leg coordination
- ABAD stability

---

# 14. Day 6–7 最終應該留下什麼？

## Output A — Right-Rim Roll-Up Capability

```text
upward edge
→ right-rim rolling feasible?
```

## Output B — Left-Rim Roll-Down Capability

```text
downward edge
→ left-rim rolling feasible?
```

## Output C — Complete Demonstration

```text
RIGHT RIM UP
→ θ=17° WHEEL TRANSITION
→ LEFT RIM DOWN
```

## Output D — Feasibility Map

```text
obstacle height × theta_climb
```

## Output E — Transition-Distance Characterization

```text
L_transition
```

---

# 15. Day 8–9 怎麼接？

Day 8–9 才開始處理：

```text
ContactState
→ airborne Bezier trajectory
→ target ContactState
```

之後會有兩種來源。

## Direct Swing

rolling 根本不適合：

```text
stance
→ liftoff
→ Bezier
→ touchdown
```

## Rolling-Assisted Swing

rolling 可以幫一部分，但無法完成：

```text
right-rim rolling
→ last legal rolling state
→ liftoff
→ Bezier
→ touchdown
```

Step 6.75 可以留作「rolling state 可以進入 airborne reconfiguration」的 prototype 參考。

但是 Day 6–7 不再實作後面的 Bezier。

---

# 16. 給 Codex 的共通前置指令

之後每次做 Step 7R–12R，可以在 task 最前面附上這段：

```text
請先閱讀目前 Day 6–7 progress note、revised note，
以及現有 single_leg_rolling_scene_2d.py 與 regression tests。

目前 Day 6–7 已完成到 Step 6.75。
Step 6.5 / Step 6.75 保留為 exploratory recovery baseline，
但新版主線已固定為：

RIGHT-RIM ROLL-UP
→ RETRACT THETA TO 17°
→ WHEEL-MODE FORWARD ROLL
→ LEFT_RIM_READY
→ LEFT-RIM ROLL-DOWN

請遵守以下原則：

1. 不要重新設計整個 project architecture。
2. 優先 reuse 現有 LegWheel kinematics / rim geometry / terrain query。
3. 不要重寫 leg model。
4. 不要覆蓋 Step 1–6.75 baseline。
5. 每個新 step 使用獨立 result / CSV / visualization。
6. 每一幀都保留 failure_reason。
7. phase 之間必須傳遞真實 final state，不要 teleport。
8. hip_x 是 trajectory variable，不要固定 hip world x。
9. hip_z 與 gamma 先維持目前 Day 6–7 assumption。
10. 不要加入 swing / Bezier / four-leg planner。
11. 每次只完成我指定的這一個 step，不要提前實作下一步。
12. 修改完成後請告訴我：
    - 修改哪些檔案
    - 新增哪些 API / dataclass
    - 執行方式
    - tests 結果
    - visualization / CSV 輸出位置
    - 尚未解決的 limitation
```

---

# 17. 接下來最先要丟給 Codex 的 Task

現在不要一次把 Step 7R–12R 全丟給 Codex。

下一個 task 只做：

```text
Step 7R
RIGHT-RIM TOP
→ continuous retract
→ theta = 17°
```

確認動畫與 geometry 正確後，再做 Step 8R。

實際順序：

```text
Step 7R
↓
人工看動畫
↓
Step 8R
↓
人工確認 LEFT_RIM_READY
↓
Step 9R
↓
人工確認 left-rim roll-down
↓
Step 10R
↓
完整動畫
↓
Step 11R
↓
feasibility map
↓
Step 12R
↓
Day 6–7 freeze
```

---

# 18. 最終一句話

Day 6–7 從現在開始不要再問：

> 「還有沒有另一種 recovery / 下障礙物的方法？」

而是固定驗證這一條：

> **Right rim 負責滾上 leading edge；上到 obstacle top 後把 theta 收回 17°，利用 wheel-mode forward rolling 轉到 left-rim descent configuration；最後由 left rim 通過 trailing edge 滾回 lower ground。**

如果這條 rolling primitive 做不到的 terrain，**不要在 Day 6–7 再發明第四種方法**。

留給 Day 8–9 的 swing primitive處理。
