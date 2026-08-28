# Hybrid Gait Day 6–7 修正版：Right-Rim Climb → Wheel-Mode Transition → Left-Rim Descent

> **這份文件是 Step 6.75 之後的新版 Day 6–7 計畫。**
>
> 目的：修正先前把 trailing-edge motion 理解成「任意 rim 直接滾下去」的錯誤。
>
> **真正想做的 traversal 是：**
>
> ```text
> RIGHT-RIM ROLL-UP
> → RETRACT θ TO 17°
> → WHEEL-MODE ROLL / RIM TRANSITION
> → LEFT-RIM READY
> → LEFT-RIM ROLL-DOWN
> ```
>
> 也就是：
>
> - 上障礙物：主要利用 **right rim**
> - obstacle top 中間：把 `theta` 收回 **17° wheel-like state**
> - 在 top 上繼續 forward rolling，讓 contact / configuration 轉到 **left rim side**
> - 下障礙物：主要利用 **left rim**
>
> Day 6–7 仍然只做 single-leg、2D、known rectangular obstacle、offline kinematic / collision feasibility。

---

# 1. 先把研究意圖講清楚

Day 6–7 不是要做：

```text
right rim roll-up
→ 到 top 後隨便繼續滾
→ 任意 rim 經 trailing edge 下去
```

也不是：

```text
right rim roll-up
→ foot rim reset
→ re-extend
→ 再走別的 motion
```

目前真正想研究的是一個**有明確方向性與 rim 分工**的 traversal primitive：

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
LEFT-RIM READY
  ↓
LEFT-RIM TRAILING-CORNER ROLL-DOWN
  ↓
LOWER GROUND
```

這是一個：

> **right-side climb + wheel-mode rim-side transition + left-side descent**

的 continuous rolling traversal。

---

# 2. `theta = 17°` 在這裡的真正角色

`theta = 17°` 不是為了把 foot rim 轉到底下。

它的用途是：

> **把 leg-wheel 收回接近 wheel-like configuration，讓上方的 right-rim climbing state 可以在 obstacle top 上，透過 forward rolling 重新轉到適合 left-rim descent 的 configuration。**

也就是：

```text
right-rim climbing configuration
        ↓
retract theta
        ↓
theta = 17°
        ↓
wheel-like rolling
        ↓
left-rim descent configuration
```

因此 Step 6.5 / Step 6.75 之前針對 foot-rim recovery 的探索可以保留為開發紀錄，但不再是新版 Day 6–7 主流程。

---

# 3. 新版 Day 6–7 主線

目前已完成到 Step 6.75，但從現在開始，主線改成：

```text
Step 7R
Roll-up end → retract theta to 17°
        ↓
Step 8R
Theta=17° wheel-mode forward roll → LEFT_RIM_READY
        ↓
Step 9R
LEFT_RIM_READY → left-rim roll-down → lower ground
        ↓
Step 10R
串成完整 right-up → wheel transition → left-down trajectory
        ↓
Step 11R
Height × theta_climb feasibility sweep
        ↓
Step 12R
Top-length feasibility / minimum transition distance
        ↓
Day 6–7 FREEZE
        ↓
Day 8–9 swing planning
```

---

# 4. Step 7R — Roll-Up 後 Retract 到 17°

## 目的

從已經成功完成：

```text
right rim front face
→ leading corner
→ obstacle top
```

的合法 end state 開始，讓 `theta` 逐步減少到：

```text
theta = 17°
```

但**仍然保持 obstacle top 上的合法 forward contact**。

這一步只做 retract，不做 left-rim roll-down。

## 重要條件

- 起點必須直接使用 Step 4.5 / roll-up simulator 真正成功的 final frame。
- `hip_x` 可以繼續往 +x 前進。
- `theta` 逐步下降。
- `beta` 可以做局部連續修正。
- 不允許跳 pose。
- 不允許 penetration / illegal rim jump。
- 不要求 foot rim ready。

## 成功條件

```text
theta reaches 17°
AND
contact remains on obstacle top
AND
trajectory remains continuous
AND
no invalid collision / penetration
```

## 給 Codex 的指令

```text
請從目前已成功的 right-rim roll-up final frame 開始，實作新的 Step 7R：RETRACT_TO_WHEEL_ON_TOP。

研究意圖：
我不是要把 foot rim reset 到下面。
我要的是：right rim 滾上 obstacle top 後，保持 top 上的合法 forward contact，同時把 theta 逐步收回到 17 degrees，準備進入 wheel-like rolling。

請：
1. 起點直接使用目前 roll-up success 的 final frame，不要手動重建近似姿態。
2. theta 每一步朝 17 deg 下降。
3. hip_x 可持續往 +x 前進。
4. beta 可做局部連續修正，使 terrain contact 保持合法。
5. 每一步檢查：
   - active rim / alpha
   - terrain surface
   - contact point
   - link collision
   - geometry penetration
   - theta / beta continuity
6. 不要求 foot rim 朝下。
7. 不要呼叫 Step 6.75 airborne reset。
8. 不要做 swing。
9. 最後成功條件：
   theta == 17 deg
   AND still valid obstacle-top contact
   AND no collision / penetration.

請保存完整 trajectory 與 animation。
```

---

# 5. Step 8R — Theta=17° 後在 Top 上轉到 Left-Rim Ready

## 目的

當：

```text
theta = 17°
```

之後，保持 wheel-like configuration，讓 hip 持續往 `+x` 方向移動。

目標不是找 foot rim。

而是：

> **找到一個適合用 left rim 進入 trailing-edge descent 的 state。**

因此需要定義：

```text
LEFT_RIM_READY
```

## LEFT_RIM_READY 第一版定義

第一版可先用幾何條件：

```text
active / candidate contact belongs to left rim
AND
contact surface == obstacle_top
AND
left-rim orientation is compatible with forward trailing-edge descent
AND
no collision / penetration
```

之後如果需要，再加入 alpha range。

## 這一步要量的核心數值

定義：

```text
L_transition
```

為：

> 從 right-rim roll-up 完成後，到 `LEFT_RIM_READY` 所需的 obstacle-top forward distance。

這比之前的 `L_reset` 更符合現在研究需求。

## 給 Codex 的指令

```text
請新增 Step 8R：WHEEL_MODE_TRANSITION_TO_LEFT_RIM_READY。

背景：
Step 7R 已經把 theta 收到 17 degrees。
現在不要找 foot rim，也不要 re-extend。
我要保持 theta = 17 deg，在 obstacle top 上繼續 forward rolling，直到 configuration 變成適合用 left rim 進入 trailing-edge roll-down 的狀態。

請：
1. theta 固定在 17 deg。
2. hip_x 持續往 +x 前進。
3. beta 按 forward rolling direction 連續更新。
4. 每一步追蹤：
   - active rim
   - alpha
   - contact point
   - terrain surface
   - hip displacement
   - contact displacement
5. 請建立 LEFT_RIM_READY 判斷。
6. 第一版 LEFT_RIM_READY 至少要求：
   - left rim 為合法 top contact / candidate
   - no collision / penetration
   - configuration 可作為下一步 trailing-edge descent 起點
7. 找到 LEFT_RIM_READY 後停止。
8. 計算：
   - required_beta_rotation
   - required_hip_forward_distance
   - required_contact_forward_distance
9. 將從 roll-up end 到 LEFT_RIM_READY 的 top distance 定義成 L_transition。
10. 不要做 roll-down。
11. 不要做 swing。
12. 不要使用 foot-rim-ready 作為終止條件。

請提供一個 animation，清楚顯示：
right-rim top state
→ theta=17 wheel mode
→ forward roll
→ left-rim ready
```

---

# 6. Step 9R — Left-Rim Trailing-Edge Roll-Down

## 目的

從：

```text
LEFT_RIM_READY
```

開始，讓 left rim 成為主要下降 contact rim。

希望建立：

```text
left rim on obstacle top
→ trailing corner
→ drop-side / back-face interaction
→ lower ground
```

這是上去 motion 的「對應下降版本」，但**不要直接寫成 roll-up 的 mirror copy**。

仍要實際跑 terrain-aware continuation。

## 成功條件

```text
starts from LEFT_RIM_READY
AND
passes trailing corner
AND
returns to lower ground
AND
left-rim-led contact remains geometrically valid
AND
no invalid collision / penetration
```

## 給 Codex 的指令

```text
請新增 Step 9R：LEFT_RIM_TRAILING_EDGE_ROLL_DOWN。

起點：
必須直接使用 Step 8R 的 LEFT_RIM_READY final frame。

研究意圖：
上 obstacle 時使用 right rim。
下 obstacle 時我要使用 left rim，用和 roll-up 類似但方向對應 trailing edge 的方式滾下來。

請：
1. hip_x 繼續往 +x 前進。
2. 以上一幀 theta / beta / left-rim contact state 作 local continuation。
3. theta 可從 17 deg 開始，必要時允許連續調整。
4. beta 持續沿 forward traversal direction 變化。
5. 主要 contact sequence 預期：
   left_rim on obstacle_top
   → trailing_corner
   → obstacle drop-side / back-face interaction
   → lower_ground
6. 每一步檢查：
   - left-rim contact validity
   - illegal rim transition
   - link collision
   - geometry penetration
   - theta / beta discontinuity
7. 不要假設是 roll-up trajectory 的時間反轉。
8. 不要呼叫 swing。
9. 不要求 foot rim touchdown。
10. 成功條件：
    最後回到 lower ground 的合法 contact，
    且整段 trajectory continuous / collision-free。

請保存：
- phase
- hip_x, hip_z
- theta, beta
- active_rim, alpha
- contact_x, contact_z
- terrain_surface
- accepted / failure_reason

phase 至少分：
LEFT_RIM_TOP
→ TRAILING_CORNER_TRANSITION
→ LEFT_RIM_ROLL_DOWN
→ LOWER_GROUND_CONTACT
```

---

# 7. Step 10R — 串成完整 Right-Up → Wheel Transition → Left-Down

## 目的

正式把整條 traversal 串起來：

```text
APPROACH
→ RIGHT_RIM_ROLL_UP
→ TOP CONTACT
→ RETRACT TO 17°
→ WHEEL-MODE FORWARD ROLL
→ LEFT_RIM_READY
→ LEFT_RIM_ROLL_DOWN
→ LOWER GROUND
```

這才是 Day 6–7 的主要完整 primitive。

## 給 Codex 的指令

```text
請把目前各 stage 串成一個完整 single-leg obstacle traversal primitive：

APPROACH
→ RIGHT_RIM_ROLL_UP
→ RIGHT_RIM_TOP
→ RETRACT_TO_17
→ WHEEL_MODE_TRANSITION
→ LEFT_RIM_READY
→ LEFT_RIM_ROLL_DOWN
→ LOWER_GROUND_CONTACT

請建立 reusable function，例如：

check_right_up_left_down_traversal(
    obstacle,
    initial_state,
    theta_climb,
    constraints
) -> RollingTraversalResult

RollingTraversalResult 至少包含：
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

要求：
1. reuse 現有 stage functions。
2. 不要呼叫 Step 6.75 airborne reset。
3. 不要呼叫 swing planner。
4. full success 必須是：
   right-rim roll-up 成功
   AND retract to 17 成功
   AND left-rim ready 成功
   AND left-rim roll-down 成功
   AND final lower-ground contact 合法。
5. 請提供完整 animation，phase 標示清楚。
```

---

# 8. Step 11R — Height × Theta_climb Feasibility Sweep

## 目的

現在正式的 rolling feasibility 不再只是：

```text
Can right rim reach obstacle top?
```

而是：

> **Can the leg complete the full right-up → wheel transition → left-down traversal?**

## Sweep

```text
obstacle_height
×
theta_climb
```

其餘條件先固定：

```text
gamma = 0
hip_z = fixed
approach condition = fixed
obstacle top length = sufficiently long
```

## 輸出

至少：

```text
obstacle_height
theta_climb
full_success
roll_up_success
retract_success
left_rim_ready_success
roll_down_success
failure_stage
failure_reason
L_transition
```

## 給 Codex 的指令

```text
請使用 check_right_up_left_down_traversal() 做正式 obstacle-height × theta-climb sweep。

這次 full rolling feasible 的定義必須是完整：

right-rim roll-up
→ retract to 17
→ wheel-mode transition
→ left-rim ready
→ left-rim roll-down
→ lower ground

固定：
- gamma = 0
- hip_z fixed
- initial approach fixed
- obstacle top length 先設足夠長

掃描：
- obstacle_height
- theta_climb

輸出 CSV：
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

產生：
1. height × theta full-traversal feasibility map
2. 每個 obstacle height 的 feasible theta range
3. obstacle height → minimum feasible theta_climb

注意：
- 只有完整回到 lower ground 才算 feasible。
- 不要用早期 fixed-hip Step 4 sweep 當 final result。
```

---

# 9. Step 12R — Top Length Feasibility

## 研究問題

obstacle top 必須夠長，才能完成：

```text
right-rim roll-up exit
→ retract to 17°
→ wheel-mode forward roll
→ left-rim ready
→ trailing-edge descent entry
```

因此 top length 的核心不是 foot-rim reset，而是：

```text
L_top >= traversal transition requirement
```

實際上不要只用公式判斷，仍要跑 geometry simulation。

## 核心量

```text
L_transition
```

定義：

> 從 right-rim roll-up exit 到 LEFT_RIM_READY 所需的實際 top forward distance。

也可另外記錄：

```text
leading-edge safety margin
trailing-edge entry margin
```

## 給 Codex 的指令

```text
請加入 obstacle top length sweep，研究完整 right-up → left-down traversal 對 L_top 的要求。

固定幾組：
- obstacle_height
- feasible theta_climb

掃描：
- obstacle_top_length

每一組都執行完整：
right-rim roll-up
→ retract to 17
→ wheel-mode transition
→ left-rim ready
→ left-rim roll-down

記錄：
- obstacle_top_length
- full_success
- L_transition
- failure_stage
- failure_reason
- leading_edge_exit_x
- left_rim_ready_x
- trailing_edge_x

找出：
minimum_top_length_for_right_up_left_down_traversal

注意：
1. 不要使用舊的 foot-rim L_reset criterion。
2. top 太短時，可能是來不及完成 left-rim transition；請如實記錄 failure。
3. 不要 fallback 到 Step 6.75。
4. 不要自動呼叫 swing。
```

---

# 10. Day 6–7 到哪裡結束

完成 Step 12R 後，Day 6–7 就 freeze。

正式輸出：

## A. 一個完整 rolling primitive

```text
RIGHT-RIM CLIMB
→ WHEEL-MODE TRANSITION
→ LEFT-RIM DESCENT
```

## B. Feasibility map

```text
obstacle height × theta_climb
```

## C. Top-length requirement

```text
obstacle geometry
→ minimum top length
```

## D. Failure stage

```text
ROLL_UP_FAIL
RETRACT_FAIL
LEFT_RIM_TRANSITION_FAIL
ROLL_DOWN_FAIL
```

---

# 11. Step 6.75 要怎麼處理

Step 6.75 不刪除。

保留成：

> **當 full continuous rolling traversal 不可行時，未來可用的 airborne transition prototype。**

但不要再讓它影響 Day 6–7 主線。

未來 Day 8–9 可以使用：

```text
最後一個合法 rolling state
→ liftoff
→ Bezier / HybridSwing
```

而不是 Day 6–7 現在就把 swing 接完。

---

# 12. 和 Day 8–9 的責任邊界

## Day 6–7

回答：

```text
Can I traverse this obstacle by:

right rim up
→ wheel-mode transition
→ left rim down ?
```

## Day 8–9

回答：

```text
If rolling cannot finish,
how should I leave the terrain contact
and generate an airborne swing trajectory?
```

因此 Day 8–9 才處理：

```text
Bezier swing
rolling-assisted swing
direct swing
touchdown target
different touchdown height
```

---

# 13. 最後一句話

新版 Day 6–7 的核心 motion 應該固定成：

> **先利用 right rim 與 obstacle leading face / corner 完成 roll-up；進入 top 後逐步把 theta 收回 17° wheel-like state，在 top 上繼續 forward rolling，直到 left rim 進入適合下降的 configuration；接著利用 left rim 通過 trailing edge 並 roll-down 回到 lower ground。**

也就是：

```text
RIGHT RIM UP
→ θ = 17° TRANSITION
→ LEFT RIM DOWN
```

之後所有動畫、Codex implementation 與 feasibility 判斷，都應該以這個 motion definition 為準。
