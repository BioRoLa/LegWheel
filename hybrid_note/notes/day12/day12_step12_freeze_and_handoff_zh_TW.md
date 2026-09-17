# Day 12 Step 12：Freeze 與 Day 13-14 Handoff

規格 §19 的七項要求，逐項作答。
**這份文件不是成功報告** —— Day 12 的完成條件是「產生同步四腳軌跡**或**
結構化的不可行結果」，達成的是**平地成功、越障結構化不可行**。

---

## 0. 一句話狀態

```text
平地    Step 9 全過（0 失敗），可上機的 CSV 已產出並驗證
越障    5 項驗證失敗，其中 2 項幾何鎖死、1 項才是 ABAD 的事
規格    §17 完成條件【達成】；§20 checklist 22 項中 18 項成立
```

---

## 1. 要求 1：Day 12 的公開資料結構與 API

### 1.1 段落與鏈（Step 0）

```text
day10_11_motion_schema_2d.py
  SegmentKind            12 個成員；.pins_theta / .is_swing
                         / .is_nominal_locomotion / .is_terrain_transition
  MotionSegment2D        一個段落：兩端接觸、取樣、body 需求、frames、rolling
  MotionSequence2D       段落串
  PointContact2D / RollingContact2D
  BodyRequirement2D      kind = NONE / LOWER_BOUND / TRACK / PINNED
  RecoveryShaping2D      RECOVERY_SWING 必須自帶的四個量
  FrameRef2D             指回生成器的真實幀

day12_segment_contract_2d.py
  entry_state_2d / exit_state_2d      可串接的共同端點格式
  BoundaryKind / boundary_kind_2d     CUT / HANDOVER
  ChainTolerance2D / ChainBreak2D
  SegmentChain2D                      多來源 chain
  chain_boundaries_2d
```

### 1.2 生成器（Step 1、Day 13 擴充）

```text
day12_nominal_cycle_2d.py
  NominalPosture2D       theta / gamma / arc_samples / roll_step_m
                         / hold_hip_z_m / hold_hip_z_profile
  HipZProfile2D          依【髖部 x】索引的高度曲線（Day 13 新增）
                         .constant(z) 與 hold_hip_z_m 逐位元等價
  RecoveryConfig2D       離地半程的旋鈕
  RollStroke2D / RecoverySwing2D / NominalCycle2D / CycleFrame2D
  run_foot_rim_roll_2d   一段有限的腳掌輪弧滾動
  run_recovery_swing_2d  離地、縮到 17 度、保持前向旋轉、再伸到觸地
  run_nominal_cycles_2d  串 N 個 cycle（造一個再平移 —— 見 §7 的限制）
  theta_for_hip_z_2d     解出把髖維持在某高度的 theta；超出行程回 None
  standing_stroke_2d     單幀「站著」的 stroke
  translate_cycle_2d     平移一個 cycle
  roll_segment_2d / swing_segment_2d / cycle_segments_2d
```

### 1.3 四腳、時間軸、body（Step 2-5）

```text
day12_four_leg_state_2d.py
  LegId                  值就是專案的腿索引（0 FL、1 FR、2 RR、3 RL）
  LEG_ORDER              【閱讀順序】LF RF LH RH，不是索引順序
  leg_mounts_2d(gamma)   四個掛點；gamma != 0 會【拒絕】
  initialize_four_leg_state_2d

day12_timing_skeleton_2d.py
  GaitTiming2D           cycle_period_s / stance_duty / phase_offsets
  walk_timing_2d         讀 GAIT_LIBRARY["Walk"]（duty 0.75）
  FourLegSchedule2D / ScheduledSegment2D / LegMode
  schedule_chains_2d     依段落索引排程（平地正確）

day12_transition_mapping_2d.py
  LegPlan2D / PhasedSegment2D / TransitionPhase / FourLegPlan2D
  build_leg_plan_2d
  plan_four_legs_2d(plans, timing, *, schedule=None)
                         schedule 可外部提供 —— 位置排程由此接入

day12_body_trajectory_2d.py
  BodyTrajectory2D / BodySample2D / BodyConflict2D / BodyDriver
  merge_demands          硬需求優先；兩個硬需求不合 = 拒絕，【不平均】
  body_trajectory_2d(..., world_registered=)
  HIP_TO_BODY_Z_M        57.166 mm
```

### 1.4 支撐與穩定（Step 6-7）

```text
day12_support_stability_2d.py
  support_triangle_at / SupportTriangle2D
  swing_stability_2d     每個 swing 的最差 margin
  DEFAULT_MARGIN_FLOOR_M 10 mm（Day 12 原始）
  GAMMA_RAD              0.0

day12_top_reposition_2d.py
  TOP_REPOSITION 的解析路徑（結論：support gate 擋住）
```

### 1.5 全機軌跡與驗證（Step 8-9）

```text
day12_whole_body_trajectory_2d.py
  WholeBodyTrajectory2D / WholeBodySample2D / LegSample2D
  HandoffCheck2D         .rim_changed / .surface_changed / .is_whole_turn
                         / .is_surface_transfer
  assemble_whole_body_2d(..., use_generator_frames=)
  handoff_checks_2d

day12_whole_body_validation_2d.py
  CheckId                12 項檢查
  ValidationReport2D / ValidationFailure2D
  validate_whole_body_2d
  MOTOR_MAX_RATE_RAD_S   1980 deg/s，兩關節共用
  motor_rates_rad_s
```

### 1.6 唯一入口與世界註冊（Step 10-11）

```text
day12_terrain_generalization_2d.py
  plan_terrain_2d        【唯一入口】。terrain=None 就是平地
  HYBRID_DECISION_ORDER  feasible -> body -> roll_preference -> margin
  TerrainRun2D / TerrainFailure2D / Stage

day12_world_registration_2d.py
  leg_approaches_2d / run_approach_2d / ApproachRun2D
  crossing_rebase_2d / rebase_sequence_2d
  build_world_leg_chain_2d / world_leg_plan_2d / world_leg_plans_2d
  body_speed_m_s         cycle 髖部前進 / 週期 —— 車速【不是自由參數】
  swing_hip_advance_m    stroke * (1-duty)/duty
  world_schedule_2d      時間由【位置】決定
  resume_phases_2d / phase_start_pose_2d / run_resumed_cycles_2d
  crossing_body_profile_2d / leg_hip_z_profile_2d   （follow_body，預設關）

day12_support_margin_scan_2d.py
  hybrid_timing_2d(cycle_period_s=2.4)   duty 0.85
  hybrid_posture_2d / hybrid_body_z_m
  HYBRID_MARGIN_FLOOR_M  3 mm（推導而來，見 §6）
  frame_motor_rate_2d    與取樣無關的速率量法
```

---

## 2. 要求 2：Day 13-14 該消費哪些介面

**不要重寫這些，從這裡接上去：**

| 目的 | 介面 |
|---|---|
| 改 ABAD | `leg_mounts_2d(gamma_rad)` —— 目前 gamma != 0 會拒絕，解開這裡 |
| 重算支撐 | `support_triangle_at` / `swing_stability_2d` |
| 換姿態 | `NominalPosture2D(theta_rad=..., hold_hip_z_m=...)` |
| 換車體軌跡 | `NominalPosture2D(hold_hip_z_profile=HipZProfile2D(...))` |
| 換步態時序 | `hybrid_timing_2d(cycle_period_s=...)` |
| 跑一個地形 | `plan_terrain_2d(terrain, tables)` |
| 出硬體 CSV | `day13_hardware_export_2d.hardware_command_2d` |

---

## 3. 要求 3：gamma 固定為 0，但**有被表示出來**

已查證，兩者都成立：

```text
表示    NominalPosture2D.gamma_rad 是欄位
        leg_mounts_2d(gamma_rad) 是參數，傳進 CorgiLegKinematics
        Step 6 的輸出有 gamma_deg 欄
        硬體 CSV 有四欄 gamma（值為 0）
拒絕    NominalPosture2D.__post_init__:
          "Day 12 fixes gamma = 0; it is Day 13-14 that frees it."
        leg_mounts_2d:
          "Day 12 fixes gamma = 0; Day 13-14 is what frees it."
```

**是拒絕不是夾住** —— 傳非零值會拿到例外，不會安靜地被改成 0。

---

## 4. 要求 4：穩定度評估與 ABAD 修正是分開的

已查證：`day12_support_stability_2d.py` 裡 gamma 只出現兩次 ——
一次是 docstring，一次是把 `GAMMA_RAD` 寫進輸出欄位。
它**讀**幾何（透過 `leg_mounts_2d`），**不做**任何修正。

所以 Day 13-14 可以改 gamma 而不必改穩定度程式碼，
穩定度會自己反映新的支撐多邊形。

---

## 5. 要求 5：地形推理不在執行期馬達程式碼裡

已查證：`day13_hardware_export_2d.py` 與 `day13_motor_export_2d.py`
對 terrain / obstacle 的唯一提及是一個 import 路徑與一句 docstring，
**沒有任何地形判斷**。

匯出層只做三件事：讀軌跡的 theta/beta、重取樣到 1 kHz、加 prep ramp。
地形只活在 `plan_terrain_2d` 之前。

Step 10 的 generalization gate 另外掃過 planner 模組裡的尺寸字面值，
結果乾淨（唯一命中是 `0.10 * cycle_period_s`，一個比例不是尺寸）。

---

## 6. 要求 6：完整回歸

見 §10。結論：**1153 passed / 9 failed，day12 與 day13 零失敗**。

---

## 7. 要求 7：架構註記與**已知限制**

### 7.1 這個模型是什麼

```text
2D 矢狀面、準靜態、幾何。
BODY_BASIS = "quasi-static body-frame approximation (no whole-robot CoM model)"
```

**有的**：剛體機身（四個掛點剛性、hip_z 四腳恆等）、真實接觸幾何、
無滑動滾動、關節行程、馬達速率預算、支撐三角形與 margin。

**沒有的**：質量、慣量、接觸力、摩擦、柔度、動力學、
車體 roll/pitch/yaw（全程 0）、ABAD（gamma 鎖 0）。

### 7.2 五個越障失敗，逐項歸因

```text
at_most_one_airborne        幾何鎖死。同 mount_x 的兩隻腳髖永遠同 x，
                            障礙物在固定世界 x -> 必然同時越障
three_support_legs          上一項的後果
body_requirement_satisfied  16 個瞬間。前腳下坡、後腳上坡，差 54.281 mm
                            -> 需要 6.08 度的俯仰，2D 沒有這個自由度
motor_rate_limit            尚未在越障上重量
support_margin  -23.5 mm    ★ 只有這一項是 ABAD 的事
```

**兩條可以寫下來的式子：**

```text
兩隻腳同時在越障中  <=>  越障髖部行程(658 mm) > 軸距(510 mm)
車體高度歧異上限    =   越障序列自己的 hip_z 跨度(54.241 mm)
```

用 300 mm 頂面驗證過：越障 558 mm、超出軸距 48 mm、衝突從 16 降到 8。

### 7.3 三個「機制有了但還不能用」

```text
follow_body            stance 腿跟隨車體高度。預設【關】。
                       開了會壞：位置相依姿態破壞 run_nominal_cycles_2d
                       的平移不變性，誤差線性累積（7.56 / 15.12 mm）
                       要做成需要三處改動，見 log §1.23-5
19 cm challenge        五個策略全部 not measured，是資料缺口不是不可行
滾動天花板              只知道 >140 mm、<=160 mm；142-155 mm 沒掃過
```

### 7.4 沒有建模、但會影響上機判讀的東西

**抬腳那側會下沉 —— 完全沒有建模。**

模型裡「抬一隻腳」只是 mode 變 AIRBORNE，車體不會有任何反應。
現在擋這件事的不是模擬下沉，而是**要求靜態穩定**：
每個 swing 的三腳支撐三角形，車體中心投影離邊界 >= 3 mm。

而那 3 mm 的 floor **只涵蓋重心量測誤差**
（重心在幾何中心是專案擁有者告知；假設 ±2 mm/軸，
乘上實測靈敏度 1.412 mm/mm -> 2.824 -> 取 3）。

```text
平地 margin 4.839 mm - floor 3 mm = 1.839 mm
留給【所有沒有建模的東西】，包括下沉、柔度、動力學、地形誤差
```

要補的最小一步是**靜態載重分配**（三腳時解力矩平衡求各腳垂直力，
再配腿的等效剛度求下沉）。缺的是一個量測（腿剛度），不是模型能力 ——
與 A4 那個「重心偏移」是同一類問題。

### 7.5 其他已知限制

```text
馬達 95.0%       平地 CSV 峰值 1881.6 deg/s / 1980，餘裕 5%
                 而且 72.0% 那個舊數字是粗網格低估的（log day13 §11.4）
車速不是參數     159.763 mm/s = cycle 髖部前進 / 週期，唯一旋鈕是週期
duty 0.75 是奇異點  margin 正比於 duty - 3/4，在那裡恆為零
19 cm            從未掃過
Walk 對照        GAIT_LIBRARY["Walk"] 仍是 duty 0.75；
                 「兩邊 duty 一致」目前靠約定，不是共用來源
```

---

## 8. Day 13-14 的待辦，依「需要什麼」分類

**這不是一份 ABAD 清單。** 五個越障失敗裡只有一個是 ABAD 的。

### A. 只有 ABAD 能解

```text
A1  support_margin -23.5 mm -> 放開 gamma，加寬側向支撐，重算支撐多邊形
```

### B. 需要新的自由度或新的生成能力（不是 ABAD）

```text
B1  車體俯仰               解 16 個 body 衝突的唯一途徑
B2  越障序列可重新生成      讓它跑在指定的車體高度，而不是重播 Day 6-7 的幀
                           （能同時解 B1 與姿態不匹配）
B3  越障序列可參數化落點     或車體在越障時停下 -> 解同時離地
B4  輪模式地面滾動          出口過渡前讓每隻腳多滾一段，錯開離地
B5  頂面上不縮成 17 度      ★ 最高價值。目標值已量出來，見下
```

### B5：頂面上用伸展姿態，而不是縮成 17 度輪模式

**動機不是美觀，是量出來的。** 40 mm x 400 mm 下，只把決策順序從
`roll_preference` 優先換成 `margin` 優先（**只改一個參數，下游同一份程式**）：

```text
                        ROLL_ROLL（現行）   SWING_SWING
min margin              -23.5046 mm         +3.7217 mm    <- 由負轉正
body conflicts          16                  0             <- 完全消失
max airborne            2                   4             <- 變糟
每腳段數                17-21               9-13
失敗項                  5                   6（多了 body_continuity、
                                              segment_chaining）
```

**兩件事被證實了：**

1. **「全程接觸」就是 margin 與 body 衝突的元兇。** 縮成 17 度貼著頂面滾，
   把車體拖到 143.8-198.0 mm；不那樣做，那 54.241 mm 的高度跨度就消失，
   16 個衝突歸零、margin 由負轉正。
2. **但 SWING 的代價是四腳同時離地**（max airborne 2 -> 4）。那不是步態。

```text
ROLL   輸在  margin、body 衝突
SWING  輸在  同時離地 4 隻、軌跡連續性
```

**B5 要的是兩者之間**：像 SWING 一樣不把車體拖低（腿保持伸展），
像 ROLL 一樣保持接觸（不要多腳同時離地）——
也就是**在頂面上用伸展姿態滾一段、轉一圈、再接觸**，
和平地的 nominal locomotion 同一種動作。

**目標值（Day 13-14 可以直接拿來對）：**

```text
min margin       >= +3.7217 mm    （SWING 已證明做得到）
body conflicts   0                （SWING 已證明做得到）
max airborne     <= 1             （ROLL 目前 2，SWING 4，兩個都不合格）
```

**已量到的幾何限制：**

```text
伸展姿態一個 stroke 的接觸前進   202.458 mm
伸展姿態一個完整 cycle           383.430 mm
現行 17 度輪模式頂面滾           225.870 mm（髖前進）

400 mm 頂面   完整 cycle 放得下，只剩 16.6 mm 餘裕
300 mm 頂面   只放得下一個 stroke，放不下完整 cycle
200 mm 頂面   兩個都放不下
```

**所以 17 度輪模式不只是「舊策略」，它是塞得進短頂面的那一個。**
B5 若要成立，頂面長度必須進入策略選擇 —— 這正是 §8 B6 要的東西。

### B5 的三個前提，已經量過（2026-09-06）

**掃 `theta_wheel_rad`：17 度不是「舊策略」，是幾何上唯一過得去的值。**

```text
theta_wheel   結果
   17 度      唯一成立（現行）
   20-40 度   RIM_SEAM_NOT_CLOSED_AT_THETA_TARGET
   45 度以上  ValueError: theta_target_rad 不得超過 roll-up 結束時的 theta
```

原因在 `single_leg_rolling_scene_2d.py`：在頂面上把接觸從 `right_rim`
交給 `left_rim` 時，兩個輪緣之間有一道接縫，程式每一步檢查它的寬度，
**太寬就繼續縮 theta 直到接縫閉合**。`RIM_SEAM_NOT_CLOSED_AT_THETA_TARGET`
的意思是「theta 已經停在目標值、不能再縮，但接縫還沒閉合」。

**腿越伸展，接縫越寬。縮到 17 度不是為了變成輪子，是為了換手。**

**但接縫限制只存在於「接觸中換手」。** 專案擁有者指出的做法是
**在空中換 rim**：離地、縮 17 度、順轉到 left_rim、再張開落地 ——
和 nominal 的 recovery swing 是同一個動作。實測 nominal recovery：

```text
起飛  foot_rim  theta 72.49 度
空中  縮到 17.00 度，順轉 280.3 度
觸地  foot_rim  theta 72.49 度
```

差別只有「轉回同一個 rim」對「轉到不同的 rim」，
而 `run_recovery_swing_2d` 已經吃 `beta_target_rad`。

**空中換 rim 會不會刮到頂面：不會，餘裕 74 mm。**

```text
                旋轉相位最小淨空    要求
平地                74.448 mm      10 mm
40 mm 頂面上        74.448 mm      10 mm
100 mm 頂面上       74.448 mm      10 mm
```

每個高度完全相同，因為淨空只取決於「髖相對於它站的那個面」，
把整個場景抬高不改變那個相對關係。

> **量這三個數字時踩到兩個坑，都值得記：**
>
> 1. `min()` 取到起飛與觸地幀 —— 那兩幀的淨空**依構造為零**，
>    所以平地基準一度讀成「0.000 mm 對要求 10 mm 卻 success=True」。
>    要求只適用於 `RECOVERY_ROTATE` 相位，真值是 74.448 mm。
> 2. 「站上頂面」不是只調 `hold_hip_z_m`（那是蹲下），
>    也不是只調 `ground_height_m`（那是把地面抬上來壓扁腿）。
>    **兩者要一起抬。** 前兩次都得到 `STROKE_START_IN_COLLISION`，
>    而那個訊息讀起來像「站不上去」，其實是「我把腿塞進地板裡」。

**所以 B5 的三個前提都成立**：接縫限制可以繞開、淨空足夠、
頂面長度限制解除（空中轉不需要頂面長度，只有落地點需要）。

### B5 續：空中換 rim 的量測（2026-09-06，**未完成**）

**確定的三件事：**

```text
1  接觸中換手必須縮到 17 度
   掃 theta_wheel：17 度是唯一成立的值，20-40 度全部
   RIM_SEAM_NOT_CLOSED_AT_THETA_TARGET。腿越伸展、輪緣接縫越寬。
   -> 17 度不是「舊策略」，是接觸中換手的幾何必然。

2  頂面淨空 74.448 mm（對 10 mm 要求）
   而且平地、40 mm 頂、100 mm 頂完全相同 ——
   淨空只取決於「髖相對於它站的那個面」，整個場景抬高不改變它。

3  alpha 是【環狀】的：right_rim 高端 180 == left_rim 低端 -180
   所以 right -> left 順著轉【不經過 foot_rim】：
       right 低端 40 -> left 高端 -40   =  80 度
   遠低於 ROTATION_CLEARANCE_LOST 的約 120 度門檻。
```

**還不確定的：空中換 rim 到底行不行。**

理由是這一輪的探測**起點就錯了**：全部用平地的 `nominal_stroke_2d` 當
起飛姿態，而它從 **foot_rim** 起飛。所以量到的是「foot_rim 出發能落到
哪」，不是 B5 的「right_rim 出發能不能到 left_rim」。

```text
量的是      foot_rim -> ?          （起點錯）
要量的是    right_rim -> left_rim  （差 80 度）
```

### 這一輪踩到的四個量測錯誤，全部同一族：**過度指定 / 用錯座標**

```text
1  min() 取到起飛與觸地幀
   那兩幀淨空【依構造為零】，於是平地基準讀成
   「0.000 mm 對要求 10 mm 卻 success=True」。
   要求只適用於 RECOVERY_ROTATE 相位，真值 74.448 mm。

2  「站上頂面」只調 ground_height_m
   那是把地面抬上來壓扁腿，不是站上去。地面與髖高要【一起】抬。
   失敗訊息 STROKE_START_IN_COLLISION 讀起來像「站不上去」，
   其實是「我把腿塞進地板裡」。

3  同時指定落地 theta 與落地 hip_z
   兩者互相決定，過約束。於是每一列都
   TOUCHDOWN_LEVELLING_UNREACHABLE，連本來會過的 0d 也失敗。
   hip_z 必須【由】落地 theta 在該 beta 下推出來。

4  把環狀的 alpha 當線段相減      <- 最嚴重
   算出 right -> left 要 220 度，差點寫成「幾何不可能」放進 handoff。
   實際上 +180 與 -180 是同一點，順著轉只要 80 度。
   （這與 Day 12 早就記過的「beta 是圈數計數器，不能直接相減」
     是同一個教訓，換個變數又踩一次。）
```

> **共通點：每一次的失敗訊息都讀起來像幾何結論**
> （「站不上去」「不是有效接觸」「淨空不足」），
> 實際上都是呼叫方式或座標用錯。
> **一個「不可行」的結論，要先排除是自己問錯問題。**

### 下一步（起點要對）

腿站在頂面上、踩在 right_rim 上的姿態，**不需要自己造**：
`SWING_SWING` 在 40 mm 是 feasible，那條序列裡就有「腿已經上到頂面」
的幀。從那裡取起飛姿態去量，才是 B5 真正的問題。

### B5 續二：改用「SWING 上去」當起點，問題大幅簡化（2026-09-06）

**專案擁有者的提議：既然 SWING_SWING 在 40 mm 是 feasible，
就拿「擺上去之後那個點」來量，不必自己造姿態。**

量出來的落地狀態：

```text
SWING_UP 落地在頂面
   theta    60.000 deg     <- 伸展姿態，【不是】17 度
   beta      0.000 deg
   alpha     0.000 deg  -> foot_rim 正中央（foot 弧是 -40..+40）
   surface  day10_11_obstacle_top
   contact  x = 260.00 mm, z = 40 mm
```

**這把 B5 從「要不要縮 17 度」變成「根本不必縮」：**

1. 擺上去之後腿站在 **foot_rim、theta 60 度**，那正是平地 nominal
   locomotion 的接觸狀態 —— 頂面上可以直接做 nominal，
   不必換 rim，也不必縮 17 度。
2. 先前那些「foot_rim 出發」的量測**不是白做的**：對這條路而言
   foot_rim 出發正是對的起點。之前量到 -30d/-60d/-80d/-90d
   都能落地、旋轉淨空 11.9-16.2 mm，那些數字對這條路有效。
3. alpha=0 在 foot 弧正中央，離兩側邊界各 40 度 ——
   要換到 left_rim 只需再轉 40 度多，不是從 right_rim 邊界起算。

**先前卡住的 right_rim -> left_rim 空中換手，在這條路上根本不需要**：
那是 ROLL 上去才會留下的姿態，是自找的麻煩。

### 頂面長度決定頂面上能做什麼（已量）

```text
一個 stroke 接觸前進   202.458 mm
一個完整 cycle         383.430 mm
SWING_UP 落地點        x = 260.00 mm（固定，不隨頂面長度變）

頂面      剩餘長度     放得下
200 mm      —         連 SWING 都沒有序列（資料缺口，不是不可行）
300 mm    140.00 mm   0 個 stroke  -> 落地即到邊緣，只能直接下來
400 mm    240.00 mm   1 個 stroke  -> 滾得了一段，放不下完整 cycle
500 mm    340.00 mm   1 個 stroke
600 mm    440.00 mm   2 個 stroke / 1 個完整 cycle
```

**專案擁有者設想的三種頂面行為，分界點就是這張表：**

```text
<= 300 mm   落地即到邊緣，只能直接下來（swing 或換 rim 滾）
400-500 mm  滾一段 stroke，但轉不回來（完整 cycle 要 383.4 mm）
>= 600 mm   才真的能「像 nominal 一樣在上面走」
```

**注意現行評估地形是 400 mm** —— 正好落在「滾得了一段但轉不回來」那一格。

### B5 續三：SWING 的四腳離地是【全程離地】造成的（已量）

先立假設再量：`SWING_SWING` 只有兩段、中間沒有接觸，所以一隻腳
**越障期間全程離地**，不只是一個 swing window。前後腳對又必然同時越障，
於是疊起來。量測證實：

```text
6.9701 .. 8.3672   4 腳離地   持續 1.397 秒
```

不是瞬間的接縫問題，是持續近 1.4 秒。每隻腳的最長連續離地：

```text
  LF  2.113 s   SWING_DOWN + 出口過渡
  RF  2.113 s   同
  LH  2.197 s   NOMINAL_RECOVERY_SWING + 入口過渡 + SWING_UP
  RH  2.113 s   SWING_DOWN + 出口過渡

對照  一個 swing window   0.360 s
      一段頂面 stroke     2.040 s（325.916 mm / 159.763 mm/s）
```

**所以頂面 stroke 正是對症的**：它在兩個 swing 中間插入接觸，
把「全程離地 2.113 s」切成兩段各約 1.0 s。

**但要誠實：這不會讓「最多一腳離地」成立。**

```text
切開後每段仍約 1.0 s，遠超過 0.360 s 的 swing window
前後腳對【仍然】同時越障（那是 mount_x 的幾何，與頂面策略無關）
-> 重疊時間會大幅縮短，但不會歸零
```

而且 400 mm 頂面只放得下一段 stroke（剩餘 240 mm vs 需要 202.5 mm），
用掉之後腿停在距邊緣 37.5 mm 處，下山怎麼接**尚未量**。

### B5 目前的總結（誠實版）

```text
已排除的死路
  縮 theta 掃描        17 度是接觸中換手的唯一可行值
  right -> left 空中換手  ROLL 上去才需要，SWING 上去根本不需要
  頂面放完整 cycle      400 mm 放不下（需 383.4 mm，剩餘僅 240 mm）

已確定的路
  SWING 上去落在 foot_rim / theta 60 / alpha 0  -> 頂面可直接做 nominal
  頂面淨空 74.448 mm 對 10 mm 要求
  頂面長度表決定能做什麼（<=300 / 400-500 / >=600 mm 三段）

已量到的效益上限
  頂面 stroke 能把 2.113 s 的連續離地切成兩段各約 1.0 s
  但【解不掉】前後腳對同時越障，那是 mount_x 的幾何

尚未做
  序列生成程式碼一行都還沒寫（只有型別骨架）
  下山接法未量
  未整合進 plan_terrain_2d，五項失敗一項都還沒改善
```

### B5 附註：轉速、轉角、路徑是三件事（2026-09-06）

專案擁有者問：空中旋轉速度可以自己調，那不是想要哪個 rim 觸地都辦得到嗎？

**「調轉角就能決定落在哪個 rim」這句是對的。** 但要和「調轉速」分開：

```text
轉多少（總角度）   可以自由選  -> 決定落在哪個 rim        <- 專案擁有者說的
轉多快（角度/秒）  = 總角度 / swing window(0.360 s)，吃馬達預算
路徑經過哪裡       由起飛與落地姿態決定，【與快慢無關】
```

`beta_step_rad` 是**幾何解析度**（每步轉幾度），不是速度；
每段的時間由位置排程給定。所以「轉快一點」實際上是改總角度或改週期。

**先前四個失敗，沒有一個是轉速問題：**

```text
RIM_SEAM_NOT_CLOSED          靜態幾何：某個 theta 下兩輪緣差多遠。與速度無關。
ROTATION_CLEARANCE_LOST      路徑經過哪裡。轉快轉慢【都刮】，時間不改變幾何。
TOUCHDOWN_IS_NOT_VALID       落地姿態對不對，不是多久到那裡。
TOUCHDOWN_ON_RIGHT_RIM       落在哪個 rim，由【總轉角】決定 —— 這個失敗
                             其實正好印證「調轉角真的能換 rim」，
                             只是生成器有 foot_rim 的硬檢查擋著。
```

唯一與速度有關的是 `motor_rate_limit`：`|dtheta|+|dbeta| <= 1980 deg/s`，
而平地峰值已經 **1881.6 deg/s = 95.0%**，往快的方向幾乎沒有空間。

**所以先前「空中換 rim 走不通」講得太滿。** 正確版本是：

```text
落到指定 rim   可以，調總轉角就行
但要同時滿足   落地姿態有效 + 旋轉全程不刮地 + 生成器接受該 rim
三者一起       在 foot_rim 出發時只有 -90d..+30d 這個窗口成立
```

不是「不能選 rim」，是「能選的 rim 被淨空與落地條件夾住」。
而這對 B5 已不構成阻礙：SWING 上去落在 foot_rim / theta 60 / alpha 0，
頂面直接做 nominal 即可，**根本不需要換 rim**。

### B6：上下半場分開決定，卡在中間的交接

架構**已經**支援（`STRATEGY_HALVES` 就是上下半場的笛卡兒積，
`ROLL_SWING` / `SWING_ROLL` 都在，頂面長度也已經是 `compose_2d` 的輸入）。

但掃過每一個高度，這兩個混合策略**全部**是：

```text
DIRECT_HANDOFF_INFEASIBLE
```

**混合策略從來沒有出現過，不是沒人做，是上半場結束的姿態
不是下半場能接手的姿態。** 要「先決定怎麼上去、再看頂面長度決定怎麼下來」，
要做的是那個**中間狀態**，不是決策層。

### C. 缺一個量測（不是缺模型）

```text
C1  腿的等效剛度      -> 才能估下沉，才知道 1.839 mm 餘裕夠不夠
C2  重心實際偏移      -> 現在假設 ±2 mm/軸
C3  330 rpm 是不是關節側、有沒有減速比 -> 現在當關節側用
```

### D. 補掃描（純機器時間）

```text
D1  142-158 mm   夾緊滾動天花板 —— 論文要的那個數字
D2  20 mm        低端到底是滾的還是擺的（現在是資料缺口，不是結論）
D3  190 mm       challenge terrain
```

### E. 工程整理

```text
E1  follow_body 的三處平移不變性修正（log §1.23-5）
E2  Walk 與 Hybrid 共用 duty 常數
E3  Day 12 的 notebook 仍是 duty 0.75 的舊圖
```

---

## 9. 凍結清單：Day 13-14 不應重寫

```text
four-leg state representation      day12_four_leg_state_2d.py
timeline representation            day12_timing_skeleton_2d.py
FOOT_RIM_ROLL semantic             day10_11_motion_schema_2d.py（SegmentKind）
per-leg segment integration        day12_transition_mapping_2d.py
body requirement merge             day12_body_trajectory_2d.py（merge_demands）
support triangle API               day12_support_stability_2d.py
stability margin API               同上（swing_stability_2d）
TOP_REPOSITION resolution path     day12_top_reposition_2d.py
whole-body trajectory schema       day12_whole_body_trajectory_2d.py
validator                          day12_whole_body_validation_2d.py
```

---

## 10. 回歸結果（2026-09-05）

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/ -q \
  --ignore=tests/test_plotly_csv_viewer.py \
  --ignore=tests/test_plotly_robot.py \
  --ignore=tests/test_multi_rim_hybrid_gait_2d.py \
  --ignore=tests/test_single_leg_hybrid_gait_2d.py

1153 passed, 9 failed, 3 warnings in 6704.84s (1:51:44)
```

（那四個 ignore 是既有的收集錯誤，Day 12 開始前就存在。）

### 九個失敗，逐一歸因：**沒有一個在 day12 / day13**

```text
test_generate_csv_tui.py                 5 個   TUI 測試領先實作
test_lateral_stance_symmetry.py          1 個   3D 站姿高度 -254.98 vs -250 mm
test_toroidal_contact_and_checker.py     2 個   CLI 措辭不符
test_flat_walk_baseline_regression.py    1 個   專案擁有者正在改的 Walk 基準
```

前八個與 2026-09-02 基準（1079 passed / 8 failed）**完全相同**。
第九個是 Walk 基準，出現在專案擁有者修改 Walk 步態期間；
曾把本方唯一動到的 `legwheel/` 檔案（`terrain_query_2d.py`）還原後重測，
**失敗依舊**，所以不是本方造成的。

### 獨立性證據

```text
那四個測試檔【沒有任何一個 import hybrid_note】
而本輪所有改動都在 hybrid_note/ 底下
唯一的例外是 legwheel/planners/hybrid/terrain_query_2d.py，
它有 9288 列 golden 資料做逐位元等價測試
```

通過數 1079 -> **1153**（+74，本輪新增的測試）。

### 本輪新增的測試

```text
tests/test_day12_world_registration_2d.py        11（此模組原本一個都沒有）
tests/test_day12_nominal_cycle_2d.py             +7（HipZProfile2D）
tests/test_day12_whole_body_validation_2d.py     +2（面轉移 vs 腿瞬移）
tests/test_day12_support_margin_scan_2d.py       34（A4 排查）
tests/test_terrain_query_2d_equivalence.py        7（+ 9288 列 golden）
```

---

## 11. Step 12 完成聲明

規格 §19 的七項要求全部完成：

```text
1  公開資料結構與 API 清單          §1
2  Day 13-14 該消費的介面           §2
3  gamma 固定為 0 但有表示          §3   已【查證】：是拒絕不是夾住
4  穩定度評估與 ABAD 修正分離        §4   已【查證】：只讀幾何，不做修正
5  地形推理不在執行期馬達程式碼       §5   已【查證】：零地形判斷
6  完整回歸                        §10  1153 passed，day12/day13 零失敗
7  架構註記與已知限制               §7   含「下沉未建模」與 5% 馬達餘裕
```

**沒有開始實作 ABAD 最佳化**，符合規格 "Do not start implementing ABAD
optimization in this step"。

**Day 12 到此凍結。**
