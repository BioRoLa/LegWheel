# Day 13 實作紀錄（接續 Day 12）

> Day 12 的紀錄在 `../day12/day12_implementation_log_zh_TW.md`。
> **那一份仍然完全有效** —— Day 13 加的每一項都是**預設關閉的 opt-in**，
> Day 12 所有凍結的數字都是在關閉狀態下量的。

---

# 1. 起點：Day 12 收尾時的七個未解結論

```text
Step 4  時間放不下   0.6 s 的窗口要裝 1.2 s（2.000 x）
Step 5  高度對不起來 三隻站立腳的 body 高度要求差 15.329 mm -> INFEASIBLE
Step 6  沒有餘裕     margin 最小 0.000 mm，五個 swing 全 unstable
Step 7  撐不住       兩個 TOP_REPOSITION 都在 support gate 停住
Step 9  beta guard   recovery 超出舊 planner 的 ±40 deg
Step 9  判不了       沒有關節速度極限，10.553 倍無法裁決
Step 10 資料缺口     19 cm 從來沒有被掃過
```

# 2. 使用者提供的兩個硬體事實（2026-09-01）

**repo 裡沒有這兩個事實**，所以它們被明確記成「來源是專案擁有者」，
而不是從程式推論出來的。

```text
1. 腿旋轉關節【可以連續轉】
   -> BETA_MAX_DEG = 40 是把 beta 當「有界擺動」的那些 planner 的慣例，
      和 Hybrid 的「圈數計數器」在講不同的量。
      記成 BETA_IS_CONTINUOUS = True，是一個【開關】不是刪除：
      切回 False，240 個失敗會回來（有測試證明）。

2. 單顆馬達 330 rpm = 1980 deg/s
   -> 轉換就在 legwheel/utils/utils.py：
        phi_r =  theta + beta - theta_0
        phi_l = -theta + beta + theta_0
      偏移是常數，所以速率是 phi_r' = theta' + beta'、phi_l' = beta' - theta'。
      【兩個關節共用一個預算】：|theta'| + |beta'| <= 1980 deg/s。
```

**這兩件事把 Step 9 從 7 過 4 失變成 9 過 3 失**（失敗記錄 251 -> 11）。

**注意 330 rpm 被當成關節側轉速。** 若那是無載值或後面有減速比，
真正的預算更小 —— 那個數字在 `MOTOR_MAX_RATE_RAD_S` 一個常數一個地方。

# 3. theta 補償滾動（解掉 Step 5）

## 3.1 問題不是機器人的，是 Step 1 的建模選擇

foot rim 滾動時 hip 高度走一段弧：兩端 202.161 mm、中間 219.448 mm，
起伏 **17.287 mm**。三隻站立腳在弧的不同位置，就要求三個不同的 body 高度，
而它們全是 hard requirement -> Step 5 回 INFEASIBLE。

**那個弧是「在整個 stroke 裡把 theta 釘死在 60 度」的後果**，不是物理限制。
腿有 theta 這個自由度可以抵消它。

## 3.2 先做便宜的可行性探測，再動機器

```text
d(hip_z)/d(theta) = +1.796 mm/deg（50-70 度之間幾乎線性）
補償 17.287 mm 需要 ~9.6 度 -> 實際解出來 12.49 度
theta 60.00 -> 72.49 度，殘差 < 0.001 mm，全在 [17, 160] 內
theta 角速度 ~32 deg/s，佔 1980 的 1.7%
```

**而且「維持弧的最高點」這個選擇是自己浮出來的**：
`219.4486 - ABAD_AXIS_OFFSET(57.166) = 162.2826 mm`
= **Step 2 註冊的那個 nominal body 高度**。不需要重新註冊任何東西。

## 3.3 實作：接進 no-slip 的公式裡，不是外掛

`_flat_roll_template` 算的 arc 是**新舊兩個姿態的輪緣弧長平均**——
也就是說它**本來就**正確處理「輪緣形狀在這一步之間改變」。
所以 theta 補償可以直接放進去：解完 beta 旋轉之後，
用 `theta_for_hip_z_2d` 解出保持 hip 高度的 theta，
再用那個 theta 重跑一次 `_flat_roll_template` 拿真正的 arc。

```text
+ NominalPosture2D.hold_hip_z_m: float | None = None   （預設 None = Day 12 行為）
+ theta_for_hip_z_2d()   在取樣幾何上解，超出關節行程回 None【不夾住】
                         夾住的 theta 會安靜地停止保持高度，然後回報成功
```

## 3.4 結果

| | 固定 theta | theta 補償 |
|---|---|---|
| hip 起伏 | 17.2871 mm | **0.000176 mm** |
| theta 掃掠 | 0 | 60.000 -> 72.486 度 |
| 接觸前進 | 202.458 mm | **202.458 mm**（完全相同） |
| 接觸輪緣 | foot_rim | **foot_rim**（沒跑掉） |
| 幀數 / 停止原因 | 49 / RIM_ARC_EXHAUSTED | 49 / 同 |

**唯一可能翻盤的事（theta 一變，接觸點跑掉腳輪緣）沒有發生。**

下游：
```text
Step 5 body conflicts       227 -> 0
  worst disagreement    15.329 -> 0.000 mm
  usable body samples    1/121 -> 121/121
  body feasible           False -> True
```

# 4. 連續接線（解掉 Step 8 的 297 mm）

## 4.1 第一層：兩段 nominal run 各自生成

`build_leg_plan_2d` 的 before / after 是兩次獨立的 `run_nominal_cycles_2d(1)`，
都從 hip_x = 0 起算 -> 接縫差 297.065 mm。

`run_nominal_cycles_2d` **內部**的 cycle 之間是接好的（實測 0.000 mm），
所以修法是：**一次要 `cycles_before + cycles_after`，再依 phase 切開**。

```text
+ build_leg_plan_2d(..., continuous_nominal=False)   預設關閉
```

## 4.2 第二層：recovery 落在錯的 theta

改成連續生成之後，斷點從 297.065 mm 降到 **14.425 mm**，沒有歸零。

原因是 `theta_touchdown = float(posture.theta_rad)` —— recovery 落在 **60 度**，
但補償版的下一個 stroke 從 **72.486 度** 起步。

有意思的是那段程式的註解本來就寫對了方向：

> the touchdown posture is what the *next* contact needs, not the compact one...
> it is **read**, not assumed equal to `theta_compact`.

只是在補償模式下，「read」要從**保持 hip 高度的約束**解出來。修好之後：

```text
cycle 邊界 contact gap        0.0000 mm
兩個 cycle 的站立 hip_z 飄動  0.000176 mm
Step 9 segment_chaining        4 -> 0
```

**這也證明 Step 8 的 handoff 檢查是有價值的** ——
14.4 mm 這種殘差用看的看不出來，是它報出來的。

# 5. 從真實幀組軌跡（上機的硬前提）

## 5.1 為什麼端點內插不能上機

Step 8 是在 segment 的**兩個端點之間內插**，而 `RECOVERY_SWING` 的頭尾
theta 相同 —— 所以組好的取樣裡，**縮到 wheel mode 再伸回來的過程根本不存在**。

```text
端點內插    theta 72.486 .. 72.486 deg   sweep  0.000 deg   <- 腿全程不縮
生成器幀    theta 17.000 .. 72.486 deg   sweep 55.486 deg
```

直接匯出去，機器人會被命令在整個 recovery 維持伸展 -> 腿會拖地。

## 5.2 幀本來就在那裡，只是沒被接上

`MotionSegment2D` 刻意只帶 `FrameRef2D`（spec 5.4），幀本體要放在別處。
Day 12 沒有把它放在任何地方 —— 現在放進 `LegPlan2D.frames`。

**一個差一格的陷阱**：`NominalCycle2D.frames` 會把 recovery 的第一幀
當成 stroke 最後一幀的重複而去掉，但 `cycle_segments_2d` 索引的是
`n_roll + n_rec`（保留）。用去重的清單會把每一個 recovery 幀位移一格。

```text
+ LegPlan2D.frames: dict[source_id, frames]
+ assemble_whole_body_2d(..., use_generator_frames=False)   預設關閉
+ LegSample2D.from_generator_frame / .is_segment_boundary_frame
```

## 5.3 真實幀修正了兩個數字

```text
              peak theta   peak beta   peak motor
端點內插          0.00      467.19      467.19  = 23.6%
生成器幀        477.57      951.48      951.48  = 48.1%
```

**先前的 23.6% 是下界。** 另外驗證器原本用「兩個峰值相加」估馬達速率，
那假設 theta 與 beta 同時到峰值，把 951 高估成 1429。
改成**逐步實測**，並保留上界當作保守估算用的欄位。

> **【2026-09-02 更正】上面這張表的 951.48 / 48.1% 本身也是錯的。**
> 排查 Day 12 的 A4 時發現：`joint_rates_2d` 是對**重取樣後**的軌跡做差分，
> 而 `leg_sample_at` 取的是「最近的 frame」，所以那條訊號是**階梯**。
> 差分階梯得到的是「一個 frame step ÷ 一個取樣間隔」，會隨取樣數變：
>
> ```text
>   samples   重取樣 deg/s    util      frame-to-frame deg/s    util
>       241        951.48    0.481                   855.41    0.432
>       481       1266.01    0.639                   855.41    0.432
>       961       1264.70    0.639                   855.41    0.432
>      1921       2528.08    1.277                   855.41    0.432
> ```
>
> 峰值就是 `RecoveryConfig2D.beta_step_rad = 4°` 那一步的離散化。
> **真值是 855.41 deg/s = 43.2%**，量法是
> `day12_support_margin_scan_2d.frame_motor_rate_2d`：
> 相鄰生成 frame 之間、在它們被排定的時刻。這個值與網格無關。
>
> 方向上是好消息（比原本以為的寬鬆），但**理由是錯的**，
> 而且同一個量法在 1921 取樣下會讀出 127.7%（超標）。
> 詳見 `../day12/day12_implementation_log_zh_TW.md` §1.6 與陷阱 61。

## 5.4 一個真實幀暴露出來的語意差別

`stance_contact_valid` 多出一個失敗，在 `RECOVERY_SWING` 的 fraction 0.0 ——
那是**離地瞬間**那一幀（本來就還在接觸）。

`mode` 是**段層級的標籤**，`in_contact` 是**幀層級的事實**，
在共用的離地 / 觸地幀上兩者**本來就會不同**。
修的是檢查（排除段的邊界幀），不是資料。

# 6. 馬達匯出

```text
+ hybrid_note/scripts/experiments/day13_motor_export_2d.py
    motor_command_2d()    -> 4 x n 的 theta / beta，【專案的腿索引順序】
    write_motor_csv_2d()  -> 透過專案自己的 create_command_csv_phi
    NotExportable         兩個拒絕
```

**兩個拒絕都是學來的**：

```text
端點內插的軌跡   直接拒絕。retraction 不在裡面。
沒被指名的失敗   直接拒絕。要匯出一條 margin = 0 的步態是一個【決定】，
                 它應該寫在呼叫端的原始碼裡讓人看得到，不是靠這裡的沉默。
```

實際產出（平地）**——以下是 2026-09-02 之前的版本，已被取代，見 6.1**：

```text
samples          241        dt 0.012448 s   ->  80.33 Hz
duration         2.9876 s
peak motor       951.48 deg/s  =  48.1% of 1980   <- 取樣產物，見 5.3 更正
accepted         support_margin   （唯一還在失敗的檢查）

leg  idx   theta (deg)        beta (deg)          peak motor
LF     0   17.00..72.49    -680.16..-39.84         951.48
RF     1   17.00..72.49    -373.17..  13.48        951.48
LH     3   17.00..72.49    -346.52..  39.84        951.48
RH     2   17.00..72.49    -399.84.. -13.17        951.48
```

## 6.1 2026-09-02 重做：duty 0.85 ＋ frame 內插

Day 12 A4 排查完之後，專案負責人選定 `stance_duty = 0.85`
（見 `../day12/day12_implementation_log_zh_TW.md` §1.6）。同時修掉一件
比「量錯」更嚴重的事：**匯出的 CSV 本身是階梯**。

`leg_sample_at` 取最近的 frame，所以 CSV 裡有整段一模一樣的姿態、
中間夾一個 4° 跳階。以 80.33 Hz 播放 = 要求機器人在**一個播放週期內**走完 4°。
這不是量測誤差，是**指令不可執行**。改成 frame 之間**內插**
（連續量內插；`rim` 與 airborne 是類別量，取較近的一幀）。

修好之後，兩個獨立的量測第一次對上：

```text
the two rate measurements, which must now agree
  between generator frames    1425.69 deg/s   ( 72.0%)
  across the command's dt     1425.69 deg/s   ( 72.0%)
  the command asks for      1.000x the planned rate
```

新的產出：

```text
stance duty          0.850       swing window 360 ms of 2.4 s
liftoff order        LF RH RF LH （與 GAIT_LIBRARY 同序，只換 duty）
support margin       4.8394 mm   （duty 0.75 時是 0.000 mm）
samples              241         dt 0.012448 s -> 80.33 Hz
duration             2.9876 s
peak motor           1425.69 deg/s = 72.0% of 1980
Step 9               11/12 通過，只剩 support_margin
accepted             support_margin

leg  idx        theta (deg)           beta (deg)     peak motor
 LF    0    17.00..72.46      -680.16..-30.43           1425.69
 RF    1    17.00..72.37      -366.42.. 16.63           1425.69
 LH    3    17.00..72.49      -342.88.. 39.84           1425.69
 RH    2    17.00..72.31      -389.94.. -6.91           1425.69
```

`support_margin` 仍然 fail，但**意義變了**：
不再是「零裕度、正好在翻覆邊界」，而是「4.84 mm 正裕度，低於一個構不到的 floor」。

## 6.2 同日稍晚：floor 定案，Step 9 變成 12/12

負責人提供事實：「之前量過，重心就真的是在機器人的中心。」
據此把 floor 從「沒依據的 10 mm」換成**推導出來的 3 mm**
（詳見 `../day12/day12_implementation_log_zh_TW.md` §1.6 A4-8）：

```python
COM_UNCERTAINTY_PER_AXIS_M = 0.002   # 假設的量測殘餘精度
MARGIN_LOST_PER_COM_OFFSET = 1.412   # 實測 mm/mm（兩軸同時偏）
-> derived 2.824 mm -> HYBRID_MARGIN_FLOOR_M = 3 mm
```

```text
Step 9 on the exported trajectory
  checks failed  0   []
  failures       0
accepted_failures                （空的）
```

`ACCEPTED` 從 `(CheckId.SUPPORT_MARGIN,)` 變成 `()`。
**平地 Hybrid 步態現在完整可行，不帶任何 accepted failure。**

上機警告改成五條，第一條換掉：

```text
1. margin 最差 4.839 mm，對 3 mm floor。
   那個 floor 只涵蓋「重心不在量測位置」，其他什麼都沒涵蓋。
2. ±2 mm/軸 是【假設】的量測精度。實際更粗的話乘 1.412 再跟 4.839 比。
3. 重心兩軸各偏 3.43 mm，裕度就歸零。
4. 沒有動力學／接觸力／摩擦：no-slip 是幾何性質不是驗證過的物理性質。
5. writer 會前置一段 0 到第一姿態的 ramp，要對照實際起始姿態。
```

> **內插的代價**：腿會經過一些沒有被逐一碰撞檢查過的姿態。
> 生成器步長（θ 2°、β 4°）界定了這些姿態離已檢查姿態最遠有多遠。

# 7. 上機前必須知道的三件事

```text
1. support margin 最差 4.839 mm（duty 0.85）——【正的】，但低於 10 mm
   的 planning floor。那個 floor 從來沒有被量過，而且 §1.6 證明這個幾何
   在任何 duty、任何速度下都到不了它。floor 該定多少【還沒決定】。
2. Day 12 【沒有任何質量模型】。投影點是 body 中心不是 CoM，
   所以「多少裕度才夠」現在無論如何都答不出來。
3. Day 12 沒有動力學、沒有接觸力、沒有摩擦。
   no-slip 是輪緣幾何的性質，【不是】驗證過的物理性質。
4. create_command_csv_phi 會在前面接一段從 0 到第一個姿態的 ramp，
   要對照機器人實際的起始姿態檢查那一段。
5. 內插引入了沒有被逐一碰撞檢查過的中間姿態（見 6.1）。
```

# 8. 現在的狀態

```text
平地      Step 9  【12/12 全過】（2026-09-02）
          duty 0.85、margin 4.839 mm vs 3 mm floor、馬達 72.0%
          馬達 CSV 不帶任何 accepted failure（但仍有上面五個警告）
越障      還有 652 mm 的 world-x 缺口（各段沒有放進共同座標）—— 還沒動
19 cm     資料缺口 —— 還沒掃
```

# 9. 下一步

```text
1. support_margin  ← 唯一還沒解的檢查，也是上機安全的關鍵
                     零 margin 是【對角對稱】造成的：交接瞬間對角兩隻支撐腳
                     量到在 (+239.2, -211.7) 與 (-239.2, +211.7) mm，
                     互為相反數 -> 連線通過 body 中心。
                     【對稱地加寬站姿沒有用】（乘上任何倍率仍然互為相反數）。
                     無 ABAD 的選項：duty > 0.75、較短 stroke、
                     不同 phase pattern、body 前後偏移。
2. 越障的 world-x 註冊
3. 19 cm 的 rolling traversal 掃描
```

# 10. 檔案索引

```text
程式  ../../scripts/experiments/day12_nominal_cycle_2d.py       + hold_hip_z_m
                                                                + theta_for_hip_z_2d
      ../../scripts/experiments/day12_transition_mapping_2d.py  + continuous_nominal
                                                                + LegPlan2D.frames
      ../../scripts/experiments/day12_whole_body_trajectory_2d.py
                                                                + use_generator_frames
      ../../scripts/experiments/day12_whole_body_validation_2d.py
                                                                + MOTOR_RATE_LIMIT
                                                                + BETA_IS_CONTINUOUS
      ../../scripts/experiments/day13_levelled_rolling_2d.py    A/B/C 對照
      ../../scripts/experiments/day13_motor_export_2d.py        匯出 + 兩個拒絕
      ../../scripts/experiments/day13_step2_motor_driver.py     產生 CSV
      ../../../tests/test_day13_motor_export_2d.py              19 tests

資料  day13_motor_command.csv    馬達指令（透過專案自己的 writer）
      day13_motor_summary.csv    每隻腳的行程與峰值速率
      ../day12/day13_levelled_rolling.csv   三個變體的對照
```

---

# 11. Walk 契約的硬體 CSV，以及一個必須更正的馬達數字（2026-09-04）

## 11.1 為什麼不能直接用 §6 那份 phi 檔

拿 §6 的 `day13_motor_command.csv` 和使用者自己的
`Walk_..._dt0.001.csv` 逐欄比對，有三個實質差異：

```text
                     Walk / trot                  phi 匯出
欄 0-7               (theta, beta) x 4 腿         (phi_r, phi_l) x 4 腿
欄 8-11              gamma，0.0                   flags，-1.0
取樣率               1 kHz，PCHIP 重新取樣        planner 自己的，沒有記錄
prep                 5000 列 cosine，從 theta=17  5000 列線性，從馬達零位
                     的折疊姿態                   ＋另外 2000 列 hold
```

`legwheel/planners/obstacle_walk/export.py` 寫得很明白：
`corgi_csv_control consumes exactly 5000 transform rows at 1 kHz`。
所以那份 phi 檔若照 1 kHz 播，241 列會在 **0.241 s** 內放完 ——
**快 12.4 倍**，而且多出來的 2000 列 hold 會被當成軌跡的前 2 秒。

## 11.2 `day13_hardware_export_2d.py`：重用契約，只放寬一個前提

重用 Walk pipeline 自己的 `CONTROLLER_DT_S`、`CONTROLLER_TRANSFORM_ROWS`、
`build_prep_rows`，所以兩邊不會默默漂開。

唯一刻意放寬的：`resample_for_csv_controller` 要求 planner dt 是 1 ms 的
**整數倍**。Walk 的 planner 自己挑 dt，所以成立；這條軌跡的取樣時間來自
位置排程（span = 髖部行程 ÷ 車速），幾乎不可能落在整毫秒上。
非整數倍時走本地的 PCHIP 重取樣，插值法／末點夾住／phase 是標籤不內插
三件事完全照抄。

## 11.3 產出（`--metres 3.0`）

```text
day13_hybrid_flat_hardware.csv          24796 列 x 12 欄，無表頭
day13_hybrid_flat_hardware_phase.csv    逐列對齊的 stance(0)/swing(1)
day13_hybrid_flat_hardware_summary.csv

列    0 .. 4999    cosine prep，從 theta=17 度折疊姿態（5.0 s @ 1 kHz）
列 5000 .. 24795   步態軌跡 19.796 s
9 個 cycle   車速 159.763 mm/s   走 3.1627 m
Step 9  0 項失敗
```

驗過：第 0 列 theta 全 17 度、beta 全 0（與 `generate_hardware_csv.py` 的
`home_pose` 相同）；第 4999 與 5000 列完全相同；gamma 四欄全 0。

## 11.4 【更正】馬達利用率不是 72.0%，是 95.0%

§6 報的 **72.0%（1425.69 deg/s）是在 planner 的粗網格上量的**。
在控制器真的會送出去的 **1 kHz 相鄰列之間**量，峰值是 **1881.6 deg/s**。

是不是插值造出來的？把 planner 網格加密，看它收不收斂：

```text
 planner 50 Hz   峰值 1888.93 deg/s   95.4%
 planner 100 Hz  峰值 1883.97 deg/s   95.1%
 planner 200 Hz  峰值 1881.61 deg/s   95.0%
```

**單調收斂到 1881，不是發散也不是消失 —— 所以 95.0% 是真的，
72.0% 是粗網格把它平均掉了。**

```text
峰值位置        軌跡第 16.5 s，LH 的 swing 內部（不是段落交界）
超過 1980 的列  0
99 / 99.9 百分位 1461 / 1764 deg/s
prep 段峰值      30.0 deg/s
```

**可以執行，但餘裕只剩 5.0%。**

> 這是陷阱 A8（「速率是取樣相依的」）的第三次出現，而且這次是**反過來**咬：
> 前兩次是細網格把速率算得太高，這次是**粗網格把它藏起來**。
> 判斷方法一樣：**加密網格，看數字往哪裡收斂**，不要相信任何單一網格上的值。

## 11.5 §6 那些數字要怎麼讀

§6 的 72.0% 不是寫錯，是**量在別的地方**——它量的是 planner 幀之間，
而那是「規劃出來的動作有多快」。95.0% 量的是「控制器每毫秒之間被要求動多少」。
**上機看的是後者。**

## 11.6 參數提到 CLI，以及一個集中的 notebook

`day13_step3_hardware_driver.py` 現在收八個旗標：

```text
--metres        走多遠（cycle 數自己算）
--period        週期；車速 = cycle 髖部前進 / 週期，唯一的直接速度旋鈕
--duty          stance duty（預設 0.85；0.75 是臨界值，margin 恆為 0）
--theta-deg     姿態 theta（預設 60）
--hold-hip-z-mm 守住的髖高（預設 = 滾動弧的最高點）
--roll-step-mm  每個生成步的接觸前進（預設 4.0，直接乘上建構時間）
--planner-hz    重取樣前的 planner 網格（預設 200；粗網格會低估馬達速率）
--out           輸出檔名
```

`--period` 實測（`--metres 1.0`）：

```text
週期 2.4 s   車速 159.763 mm/s   峰值 1881.6 deg/s   95.0%
週期 3.0 s   車速 127.810 mm/s   峰值 1489.3 deg/s   75.2%
```

**週期與馬達峰值幾乎成反比**，兩點連線外推：要回到 80% 以下，週期大約要 2.85 s 以上。

集中入口：`hybrid_note/notes/gait_csv_workbench.ipynb`。
它**不重新實作任何 planner** —— 只是呼叫 driver、讀回三個檔、畫圖，
並且把契約當成斷言檢查（row0 的 theta 是不是 17 度、prep 有沒有接上軌跡、
gamma 是不是全 0）。新的步態要加進來的規則寫在最後一節。

---

# 12. Day 13 開工：先確認 C 類是不是我能量的，然後 D1 的前提檢查（2026-09-06）

## 12.1 讀完三份文件後的第一個判斷：C 類我量不了

`day13_plan_zh_TW.md` §4 建議的順序是 C1-C3 先做（最便宜，且會改變
怎麼讀現有所有 margin 數字）。**但 C1/C2/C3 三項全部是「事實輸入」，
repo 裡推導不出來。** 已查證：

```text
C1 腿的等效剛度   legwheel 沒有質量、沒有慣量、沒有勁度
                  全 repo 只有四個檔提到 stiffness/compliance，
                  沒有一個是腿的結構剛度模型
C2 重心實際偏移   day12_support_margin_scan_2d.py:103-111 自己寫著
                  「這是專案擁有者的量測，repo 沒有任何質量模型」
C3 330 rpm        day12_whole_body_validation_2d.py:67-72
                  「as given for this robot」，當關節側用是一個【假設】
```

`examples/self_righting/analysis/stability_analysis.py` 確實出現 mass，
但它是把質量當**自由變數**掃 5-20 kg，不是量到的值。

**所以 C 類要問專案擁有者，不是跑程式。** 已列成三個問題。
在等回答的同時，先做 D 類（純機器時間）。

## 12.2 D1 的前提要先查：「>140 mm、<=160 mm」可能不是幾何

D1 寫的是「掃 142-158 mm，夾緊滾動天花板」。**但在花八分鐘一格
去掃之前，先看現有那 70 格的失敗長什麼樣**——這正是 §6 說的
「一個『不可行』的結論，要先排除是自己問錯」。

`day6_7_step11r_feasibility_sweep.csv`，h=120/140/160 三列：

```text
h=120   th 40 ✗  45 ✓  50 ✗  55 ✓  60 ✗  65 ✗  70 ✗  75 ✗  80 ✗  85 ✗
h=140   th 40 ✓  45 ✓  50 ✗  55 ✓  60 ✗  65 ✗  70 ✓  75 ✗  80 ✗  85 ✗
h=160   全 ✗
```

**h=140 的 feasible 在 theta 上是跳的：45 ✓、50 ✗、55 ✓、60 ✗、70 ✓。**
真的幾何天花板不會這樣交替。交替是【解析度】的特徵，不是極限的特徵。

失敗原因幾乎全是 `COUPLED_RESET_COLLISION_BLOCKED`，來自
`single_leg_rolling_scene_2d.py:4219`。讀那段程式：它是一個
**以 `beta_step` 為格點的離散局部搜尋**，所有候選都不合法才報這個。
也就是說它是「在這個格點上找不到」，不是「不存在」。

### 12.2.1 一個我原本猜錯、查了才更正的方向

原本懷疑是輪緣接縫 guard（記憶裡有「arc_samples 低於約 140 會
假性拒絕右->左換手」）。**查了程式，不是。**
`right_up_left_down_sweep_2d.py:101` 的 `seam_bridge_for_sampling_m`
已經讓 guard 隨 arc_samples 縮放：

```text
arc_samples=121 -> max(5mm, 3 x 5.83mm) = 17.5 mm
```

所以那 70 格的接縫 guard 是**放寬過的**，接縫不是元兇。
（這條記下來是因為它示範了「先查再說」：憑記憶會寫出錯的歸因。）

### 12.2.2 真正沒被測過的旋鈕：beta_step

`TraversalConstraints2D` 的搜尋解析度全部固定：

```text
pivot_beta_step_rad     1.0 deg
wheel_beta_step_rad     1.0 deg
roll_up_beta_step_rad   1.0 deg
theta_step_rad          1.0 deg
```

而 `SweepSettings2D` **沒有把這些暴露出來**——那 70 格全部跑在 1 度。
所以「>140 mm」這個天花板，是在 1 度搜尋網格上量到的。

**做法照 §11.4 對馬達速率那次：加密網格，看數字往哪裡收斂。**
如果 h=140 那些 ✗ 在 0.5/0.25 度下變成 ✓，天花板就不是 140，
而 D1 掃 142-158 的前提（在 1 度網格上）就是錯的。

## 12.3 C1-C3：專案擁有者的回答（2026-09-06）

```text
C3  330 rpm 是【關節側】，無減速比
    -> 現行程式的假設是對的。1980 deg/s 就是關節能動的速度，
       平地峰值 95.0% 的判讀【不變】，不需要重算。
       這一項【結案】，可以從待辦移除。

C1  腿的等效剛度：沒量過
    -> 下沉量仍然無法估。維持現況：完全沒建模，
       擋它的只有靜態 margin >= 3 mm floor。
       平地 4.839 - 3 = 1.839 mm 要涵蓋下沉、柔度、動力學、地形誤差。
       這是【上機前的已知風險】，不是可以算掉的東西。

C2  重心 ±2 mm/軸：是保守猜測，不是量測
    -> 於是 margin floor 3 mm 的推導鏈是
         猜測 2 mm x 實測靈敏度 1.412 mm/mm = 2.824 -> 取 3
       **floor 本身建立在一個猜測上。**
```

### 12.3.1 這三個答案合起來改變了什麼

**C3 是好消息，C1/C2 把 margin 的判讀往下修。**

現在可以把「平地 margin 4.839 mm」這個數字的成色寫清楚：

```text
4.839 mm   量出來的（幾何，可信）
-3.000 mm  floor，其中的 2 mm 是【猜的】，1.412 mm/mm 是量的
=1.839 mm  留給下沉（沒量）、柔度（沒量）、動力學（沒建模）、地形誤差
```

**也就是說：4.839 mm 裡面，只有「4.839」這個數字本身是硬的；
它要對抗的門檻與餘裕兩邊都不硬。** 論文與上機都應該這樣講，
不能只寫「margin 4.839 mm > floor 3 mm，STABLE」。

C1 與 C2 都是**量測**缺口而不是模型缺口（Day 12 §7.4 已經這樣分類過），
現在有專案擁有者的確認：兩者都還沒量。所以 Day 13 不該再等它們，
**它們不阻擋 D 類與 B 類的工作**，只是限制結論能講多滿。

## 12.4 D2 的前提查證：20 mm 確實只是資料缺口（已確認）

比對兩邊的掃描高度：

```text
day10_11_step2_swing_onto_sweep.csv   20 40 60 80 100 120 140 150 160 180 200
day10_11_step3_swing_off_sweep.csv    20 40 60 80 100 120 140 150 160 180 200
day6_7_step11r_feasibility_sweep.csv     40 60 80 100 120 140 160
                                      ^^
                                      滾動【沒有】20 mm 這一格
```

**所以 20 mm 選 SWING 不是因為 ROLL 輸了，是因為 ROLL 沒有參賽。**
`Availability.NOT_MEASURED` 忠實地表示了這件事（decision map 模組
開頭第 2 點就在講這個），但論文不能停在這裡。

而且滾動在 40 mm（它掃過的最低點）是**大致可行**的：

```text
h=40   th 40 ✓ 45 ✓ 50 ✓ 55 ✓ 60 ✗ 65 ✓ 70 ✓ 75 ✗ 80 ✗ 85 ✗   (6/10 可行)
       失敗全是 APPROACH_DID_NOT_REACH_THE_FRONT_FACE
```

注意那個失敗是**接近階段**的（腿還沒碰到障礙物前面），
與高處那個 `COUPLED_RESET_COLLISION_BLOCKED`（換手階段）**不同族**。
低端失敗與接近淨空 0.04 m 的設定有關，不是越障本身不可行。

**低端滾動很可能是可行的**，值得量。D2 因此和 D1 是同一個實驗，
只是換個高度：往 `step11r` 那張表加幾列而已。

## 12.5 還有一個沒被測到的解析度軸：arc_samples

讀 `right_up_left_down_traversal_2d.py:1305-1310` 的角落樞轉：
它要求接觸點維持在**同一個輪緣取樣索引**上
（`abs(probe_candidate.sample_index - pin) > sample_match_tolerance` 就跳過）。

**那是一個依 `arc_samples` 而定的條件。** h=160 的
`NO_LEGAL_CORNER_PIVOT_CONTINUATION` 因此對輪緣取樣密度敏感，
而那 70 格全部跑在 arc_samples=121。

本輪的收斂測試**只動搜尋步長（beta_step 族），沒有動 arc_samples**，
所以它能回答「1 度網格夠不夠細」，**不能**回答
「121 個輪緣取樣夠不夠密」。這是誠實的範圍界定，
第二個軸留給後續。

## 12.6 收斂測試怎麼跑的（方法，可重現）

```text
腳本   scratchpad/d1_converge.py
條件   SweepSettings2D() 原封不動（含 arc-matched 的 17.5 mm 接縫 guard）
變數   把所有搜尋步長一起乘上 factor
       approach_coarse / roll_up / roll_up_top / theta_step
       / wheel_beta / pivot_beta
格點   h = 120 / 140 / 160 mm  x  theta 40..85  x  factor 1.0 / 0.5 / 0.25
```

**先驗證再相信：** factor=1.0 必須重現已發表的那張表。
抽兩格對過，完全一致：

```text
h=140 th=45  feasible=True                                   （表上 ✓）
h=140 th=50  False LEFT_RIM_TRANSITION_FAIL
             COUPLED_RESET_COLLISION_BLOCKED                 （表上 ✗）
```

後續跑到的 1.0 格也持續吻合（h=140 的 50 ✗ / 55 ✓ / 60 ✗ / 65 ✗）。
**所以這個 harness 問的是跟原本一樣的問題**，
factor 變細之後若有變化，那個變化是真的訊號。

（踩到一個環境坑，記下來：用 `nohup ... &` 丟背景，
shell 一結束就把整個 ProcessPoolExecutor 帶走，90 格只跑完 1 格就
「正常結束」。改用 `setsid ... & disown` 才活得下來。
**背景工作要確認它真的還活著，不要看到 exit 0 就當成跑完。**）

## 12.7 【本輪的量測錯誤，第五次同一族】細化步長會餓死步數預算

收斂測試跑到 factor 0.5 時，失敗原因**換了一種**：

```text
factor 1.0（30 格）   0 個步數預算失敗
                     17 COUPLED_RESET_COLLISION_BLOCKED
                      6 可行
                      5 NO_LEGAL_CORNER_PIVOT_CONTINUATION
                      2 NO_SLIP_REQUIRES_NEGATIVE_X_MOTION

factor 0.5（前 9 格） 9 格【零可行】
                      5 MAX_FORWARD_STEPS_BEFORE_TOP_ROLL_COMPLETE   <- 新的
                      2 MAX_FORWARD_STEPS_BEFORE_TOP_SUPPORT         <- 新的
                      2 COUPLED_RESET_COLLISION_BLOCKED
```

**九格裡七格是「步數用完」，不是幾何。**

原因很單純：那些上限全部是**次數**，不是距離。

```text
approach_max_steps          200
roll_up_max_forward_steps   100
每個 stage 的 frame 上限     800 / 1200 / 500
```

步長減半，同一段實體距離就要走**兩倍**的步數，但預算沒有跟著加。
於是迴圈在還沒走到終點前就用完次數，回報
`MAX_FORWARD_STEPS_BEFORE_TOP_ROLL_COMPLETE`。

> **這正是 §6 那一族的第五次：訊息讀起來像幾何結論
> （「滾不完」「上不了頂面」），實際上是我自己把預算調到不夠。**
> 而且這次特別危險：如果沒注意到失敗【原因】換了種類，
> 只看「0.5 度全部不可行」，會直接寫出
> 「加密網格後更不可行，所以 140 mm 天花板是真的」——
> **一個完全相反的錯誤結論。**

**修正**：`factor` 同時縮步長、放大預算（grow = 1/factor），
並把 `budget_scale` 寫進輸出，讓被餓死的格子事後可辨識。

```text
approach_step_m           0.005 * factor
roll_up_dx_m              0.002 * factor
approach_max_steps        200 * grow
roll_up_max_forward_steps 100 * grow
```

grow=1 時與原設定完全相同，所以 factor 1.0 仍必須重現已發表的表。

**已作廢的部分結果**保留為 `d1_converge_INVALID_partial.csv`，
它的 factor 0.5 那 9 列**不可引用**。

## 12.8 附帶確認：Day 12 相關測試仍然全過

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  tests/test_day12_whole_body_validation_2d.py \
  tests/test_day12_support_margin_scan_2d.py -q
-> 76 passed in 600.76s
```

本輪到目前為止**沒有動任何專案程式碼**，只在 scratchpad 寫量測腳本。

## 12.9 套用「指標變好不等於問題解決」到收斂測試本身

專案擁有者提供的五個常犯錯誤清單裡，第三項（指標變好不等於解決，
要看有沒有東西悄悄消失）**直接適用於我正在跑的這個收斂測試**。

原本的腳本只記錄 `feasible` 與 `failure_reason`。**那不足以判斷。**
如果換網格之後某個 stage 悄悄被截斷，我只會看到
「可行格數變了」，卻分不出那是真的改變還是有東西不見了
——正好就是 16 -> 0 那個 body conflict 的同一種陷阱。

於是加記「走到多遠」的證據，才有辦法事後稽核：

```text
frame_count    整條軌跡幾幀
phases         phases_visited 串起來
n_phases       走過幾個相位
approach_ok / roll_up_ok / retract_ok / left_rim_ok / roll_down_ok
               五個 stage 各自的成敗
```

這樣「factor 0.5 下可行格數變了」才能拆成兩種可能：
**真的判定改變**（五個 stage 走得一樣遠，只是最後結論不同）
vs **有東西消失**（某個 stage 根本沒被執行到）。

**沒有這些欄位的話，這個收斂測試本身是不可稽核的。**
前一版（v2）因此作廢重跑，v3 才是要引用的那份。

## 12.10 本輪至今，方法上的三個修正（都在下結論之前）

```text
1  背景工作用 nohup & 會被 shell 帶走      -> 改 setsid + disown，並驗活著
2  細化步長餓死步數預算，失敗訊息像幾何    -> 預算隨 grow=1/factor 同步放大
3  只記 verdict 無法辨識「stage 消失」     -> 加記 frame_count / phases / 五個 stage
```

三個都不是幾何問題，都是**量測方式**的問題。
而且三個都會導向看起來合理的錯誤結論。

# 13. D1 的結論：那個「天花板」在 1 度網格上【不穩定】（2026-09-06）

資料：`day13_d1_grid_sensitivity.csv`（60 格，v3，步數預算已同步放大）

## 13.1 先確認沒有東西消失

```text
exceptions              0
步數預算失敗（餓死）      0        <- v2 的病已治好
frame_count 在 0.5 度變多  24 / 30 格
在 0.5 度走到【更後面】的 stage  4 格
```

**細網格是做了更多工，不是更少。** 所以接下來的差異不是
「有東西被截斷」，是真的判定不同。（這就是 12.9 加那些欄位的用途。）

## 13.2 主結果：判定不是收斂，是【重新洗牌】

```text
兩個網格都可行     2 / 30 格
只有 1.0 度可行     4 格
只有 0.5 度可行     1 格
--------------------------------
判定不一致          5 / 30 格 = 17%
```

**而且是雙向的**，這是關鍵：

```text
h=140 th=40   可行  -> 不可行
h=140 th=55   可行  -> 不可行
h=140 th=65  不可行 ->  可行     <- 反方向
h=120 th=45   可行  -> 不可行
h=120 th=55   可行  -> 不可行
```

如果細化只是「更嚴格」，改變會是單向的（可行變不可行）。
**h=140 th=65 由不可行變可行，證明不是變嚴格，是換了搜尋路徑。**

`COUPLED_RESET_COLLISION_BLOCKED` 是一個**離散局部搜尋**找不到合法後繼，
換個步長就換一組候選點——所以它報的是
「在這個格點上找不到」，不是「不存在」。

## 13.3 這對 D1 代表什麼

**D1 原本的問法（掃 142-158 mm 夾緊天花板）建立在一個不成立的前提上：
它假設某個高度的可行性是一個定值。實測 17% 的格子會隨網格翻面。**

```text
原本以為   天花板在 (140, 160] 之間，掃中間就能夾緊
實際上     140 mm 那一列本身就有 3/10 格會隨網格翻面
           在 1 度網格上「140 可行、160 不可行」這件事
           【本身】不是一個穩定的量測
```

所以在 142-158 掃出來的任何一條界線，**都會帶著同樣 17% 的不確定性**，
論文不能拿它當「夾緊的天花板」。

**這也解釋了原本那個可疑的交替**（h=140 的 45 ✓ 50 ✗ 55 ✓ 60 ✗ 70 ✓）：
它不是幾何在交替，是搜尋在某些格點上運氣好、某些運氣不好。

## 13.4 誠實的範圍界定：這個測試【不能】回答什麼

```text
能回答   1 度的 beta_step 網格夠不夠細   -> 不夠，17% 會翻面
不能回答 121 個 arc_samples 夠不夠密
         h=160 的 NO_LEGAL_CORNER_PIVOT_CONTINUATION 依賴
         「接觸維持在同一個輪緣取樣索引」，那是 arc_samples 的條件，
         本輪固定沒動。
不能回答 真正的天花板在哪
         要先有一個【對網格穩定】的可行性判準，才談得上量天花板。
```

## 13.5 建議：D1 應該改成什麼

**不要直接去掃 142-158。** 先解決判準不穩定：

```text
1  找出 COUPLED_RESET 的搜尋為什麼對步長這麼敏感
   （它是離散局部搜尋 + beta_search_window，可能是窗口太窄）
2  或者改用「多個網格都可行才算可行」的保守判準，
   並把 17% 的不一致當成論文裡的已知不確定度誠實報出來
3  在有穩定判準之後，才掃 142-158
```

**在那之前，「滾動天花板 >140 mm、<=160 mm」這個敘述應該加註
「在 1 度搜尋網格上量得，該判準有 17% 的網格敏感度」。**

## 13.6 找到重新洗牌的【機制】，不只是統計現象

13.2 只說了「判定會隨網格翻面」。追下去找到了原因，
在 `single_leg_rolling_scene_2d.py:2262-2276`：

```python
desired_beta = previous_beta + top_beta_step
beta_values = [desired_beta]
beta_search_count = floor(beta_search_window_rad / beta_search_step_rad)
for index in range(1, beta_search_count + 1):
    beta_values.append(desired_beta + index * top_beta_step)   # <- 關鍵
```

**候選解是以 `top_beta_step` 的整數倍產生的。**
而我這次縮的正是 `roll_up_top_beta_step_rad`。

所以把步長減半，**不只是把同一條路徑走得更細，
而是換掉了整組候選解**——搜尋在完全不同的一組點上找落腳處。

```text
以為在做   同一個問題，網格更細 -> 答案應該收斂
實際在做   換一組候選解 -> 答案沒有理由收斂
```

**這就是為什麼判定是雙向洗牌而不是單向收斂。**
`COUPLED_RESET_COLLISION_BLOCKED` 因此不是幾何結論，
它是「這一組候選點裡沒有合法的」。

### 13.6.1 這讓 13.3 的結論更強，但也改了它的性質

原本寫「判準有 17% 的網格敏感度」——那是把它當成**雜訊**。
現在知道是**機制**：步長同時扮演兩個角色
（路徑解析度 ＋ 候選解產生器），這兩個角色本來就該分開。

**真正該做的下一個實驗**：固定 `top_beta_step`（路徑不變），
只改 `beta_search_window_rad` / `beta_search_step_rad`（候選密度）。
那才是「候選夠不夠密」的乾淨測試。

**但那需要把 `beta_search_window_rad` 接到 `TraversalConstraints2D` 上**
（目前只到 stage 函式，沒有上到 constraints，所以 sweep 改不到，
全程固定 5 度）。那是動到專案程式碼，**不在本輪量測的範圍內，
需要專案擁有者同意再做。**

## 13.7 Day 13 本輪的狀態小結

```text
C1  腿剛度        沒量過（擁有者確認）—— 仍是上機前的已知風險
C2  重心 ±2 mm    是猜測（擁有者確認）—— margin floor 3 mm 建立在猜測上
C3  330 rpm       關節側、無減速比（擁有者確認）—— 【結案】，95.0% 判讀不變

D1  滾動天花板    前提不成立。可行性判準本身對網格敏感（5/30 格翻面，雙向），
                  且已找到機制：候選解以 top_beta_step 的整數倍產生。
                  **在有穩定判準之前不應該掃 142-158。**
D2  20 mm         前提已確認（滾動掃描確實沒有 20 mm 那格，是資料缺口）。
                  腳本已寫好（d2_low_end.py，含 20/30 mm 與 142-158 mm），
                  **但因為 D1 揭露判準不穩定，先不跑**——
                  用一個會翻面的判準去填資料缺口，只會產生一張不可信的表。
D3  190 mm        未動
```

**本輪沒有修改任何專案程式碼**，所有腳本在 scratchpad。
唯一寫進 notes 的資料檔是 `day13_d1_grid_sensitivity.csv`。

---

# 14. 把 `beta_search_window_rad` 接到 constraints 上（2026-09-06，擁有者已同意）

## 14.1 改了什麼（純加法）

```text
right_up_left_down_full_traversal_2d.py
  + TraversalConstraints2D.wheel_beta_search_window_rad = deg2rad(5.0)
      預設【就是】現有所有掃描實際跑的值
  + Stage 3 傳給 run_wheel_mode_transition_to_corner_2d
      （原本沒傳，所以一直吃該函式的預設 5 度）
  + __post_init__ 加非負驗證，與既有風格一致
```

**驗證預設行為沒變**（這是唯一重要的事）：

```text
h=140 th=45   feasible=True   frames=305     <- 與 v3 逐項相同
h=140 th=50   False COUPLED_RESET  frames=99  <- 與 v3 逐項相同
```

回歸：

```text
tests/test_right_up_left_down_full_traversal_2d.py
tests/test_right_up_left_down_traversal_2d.py
tests/test_right_up_left_down_sweep_2d.py
-> 68 passed in 178.76s
```

## 14.2 為什麼「只改 window」才是乾淨的測試

關鍵在 `single_leg_rolling_scene_2d.py:4170-4180`：

```python
search_count = floor(beta_search_window / beta_step)
signed_steps = [ +-count for count in range(2, search_count + 1) ]
search_beta  = previous_beta + signed_step * beta_step
```

兩個旋鈕的角色是**不同**的：

```text
beta_step     候選點的【間距】   —— 同時也是路徑解析度（所以 §13.6 才不收斂）
beta_search_window  搜尋能伸多遠的【範圍】 —— 透過 search_count 決定候選【數量】
```

**固定 step、只放寬 window ＝ 在同樣的間距上多給候選點，路徑解析度完全不動。**

於是兩個假設可以被分開檢定：

```text
若放寬 window 讓判定穩定下來
    -> 失敗是「搜尋太早放棄」，不是幾何
若放寬 window 判定不動
    -> 5 度的搜尋範圍已經夠，失敗確實在幾何那一側
       （此時 §13 的網格敏感度就要歸因到 beta_step 換候選集這件事上）
```

這是 §13.6 說的那個乾淨測試，現在做得到了。

## 14.3 實驗設定

```text
腳本    scratchpad/d1_window.py
格點    h 120/140/160 mm x theta 40..85 x window 5/10/20 度 = 90 格
不變    所有步長、預算、容差都是已發表的值
基準    window 5 度【必須】重現原表，否則接線接錯了
```

# 15. Window 掃描結果：**放寬搜尋範圍完全沒用**（2026-09-06）

資料：`day13_d1_search_window.csv`（90 格 = 30 格 x window 5/10/20 度）

## 15.1 基準先過

```text
window 5 度（＝已發表設定）  6 格可行  <- 與原表相同
```

## 15.2 主結果：判定【完全不動】

```text
window  5 度   6 可行
window 10 度   6 可行
window 20 度   6 可行

逐格比對   可行性 30/30 格【完全相同】
frame_count  28/30 格【逐位元相同】
```

**把搜尋範圍放大四倍，判定一格都沒有改變。**

有兩格「變了」，但變的只是失敗**原因**，兩格都還是不可行：

```text
h=120 th=80   NO_SLIP_REQUIRES_NEGATIVE_X -> COUPLED_RESET   frames 99 -> 132
h=160 th=45   NO_SLIP_REQUIRES_NEGATIVE_X -> COUPLED_RESET   frames 79 ->  90
```

也就是「多走了幾步，然後撞到同一面牆」。

### 15.2.1 這兩格同時也是「參數真的有生效」的證據

必須排除「參數根本沒接到」這種可能（否則結果沒有意義）：

```text
1  那兩格走得【更遠】（99->132、79->90 幀）並換了失敗模式
   -> 參數確實到達了那段程式
2  三組 window 的執行時間不同（總和 2747 / 2611 / 2367 秒）
   -> 不是同一份快取結果
```

## 15.3 我先前的假設【被推翻了】

§13.5 與 §14.2 我寫的是「可能是搜尋窗口太窄」。**量出來不是。**

```text
先前假設   COUPLED_RESET 是「搜尋太早放棄」-> 放寬 window 應該會改善
實測結果   放寬四倍，判定零改變
結論       5 度的搜尋範圍【已經夠了】。失敗不在「找得不夠遠」這一側。
```

## 15.4 於是網格敏感度的歸因收斂到唯一一個機制

兩個實驗合起來，把原因夾出來了：

```text
改 beta_step（§13）    5/30 格判定翻面，【雙向】
改 window  （§15）     0/30 格判定改變
```

**同樣是「讓搜尋看到更多候選點」，一個會翻面、一個完全不會。**
差別就是 §13.6 那個機制：

```text
放寬 window   在【同樣的間距】上多給候選點 -> 候選集是原本的超集
              原本找得到的還是找得到，找不到的多找幾個也還是找不到
縮 beta_step  候選點【落在不同位置】(desired_beta + index * top_beta_step)
              -> 換掉整組候選集，不是超集，所以判定會雙向跳
```

**所以「滾動可行性對網格敏感」這件事，原因不是搜尋不夠努力，
而是候選解的【位置】被步長綁死。** 這是一個比 §13 更精確的結論。

## 15.5 對 D1 的最終建議（比 §13.5 更明確）

```text
不要   掃 142-158 mm —— 判準本身會隨步長雙向翻面
不要   靠放寬搜尋範圍來修 —— 已證實無效
要     若要一個對網格穩定的判準，得讓候選解【不隨步長平移】，
       例如把候選集定義成絕對角度網格，或對 beta 做連續求解
       而不是「desired_beta 的整數倍」
```

**在那之前，論文引用滾動天花板時必須附上：
「在 beta_step = 1 度、beta_search_window = 5 度的搜尋設定下量得；
該判準對步長敏感（5/30 格會翻面），對搜尋範圍不敏感（0/30 格）。」**

這比原本的「>140 mm、<=160 mm」誠實得多，而且**兩個數字都是量出來的**。

## 15.6 本輪收尾狀態（2026-09-06）

### 程式碼變更（唯一一處）

```text
right_up_left_down_full_traversal_2d.py    +14 行、-0 行
  純加法，預設值 = 現有所有掃描實際跑的 5 度
```

回歸（五個 `right_up_left_down_*` 測試檔）：

```text
103 passed in 329.94s
```

加上先前的 `test_day12_whole_body_validation_2d.py` +
`test_day12_support_margin_scan_2d.py` = **76 passed**。

### 今天實際動到的檔案（已逐檔用 mtime 稽核）

```text
程式碼   right_up_left_down_full_traversal_2d.py           （+14 行）
資料     day13_d1_grid_sensitivity.csv                     （步長，60 格）
         day13_d1_search_window.csv                        （範圍，90 格）
notebook hybrid_gait_day13_grid_stability_dashboard.ipynb  （24 格，已執行）
文件     day13_implementation_log_zh_TW.md、day13_README_zh_TW.md
```

（`day12/` 那兩個檔與 `day13_plan_zh_TW.md` 的 mtime 是 11:25-11:26，
**早於本輪第一個動作**，是專案擁有者自己的編輯，不是本輪改的。）

### Day 13 待辦的最新狀態

```text
C1  腿剛度      沒量過（擁有者確認）—— 上機前已知風險，非模型缺口
C2  重心 ±2mm   是猜測（擁有者確認）—— margin floor 3 mm 建立其上
C3  330 rpm     關節側、無減速比    —— 【結案】

D1  滾動天花板   前提不成立，且原因已定位到唯一機制（候選解位置被步長綁死）
                 對步長敏感 5/30、對範圍不敏感 0/30，兩個數字都量出來了
                 【建議暫緩，直到有不隨步長平移的候選解】
D2  20 mm       前提確認成立（是資料缺口），腳本已寫好，
                 【建議暫緩】—— 判準不穩定時填缺口只會得到不可信的表
D3  190 mm      未動

B/E 類          未動
```

### 下一輪可以直接接手的

```text
1  arc_samples 軸（第三個、還沒檢定）
   h=160 的 NO_LEGAL_CORNER_PIVOT_CONTINUATION 依賴
   sample_match_tolerance（輪緣取樣索引），固定 121 沒測過
2  讓候選解不隨步長平移（絕對角度網格 / 對 beta 連續求解）
   —— 這是讓 D1、D2 變得可做的前置條件
3  B3（錯開前後腳對的越障時機）—— 唯一能解 at_most_one_airborne 的
   與 D 類無關，不受本輪結論影響，可平行進行
```

---

# 16. B3 前置分析：為什麼前後腳對會同時越障（2026-09-06）

B3 是**唯一**能解 `at_most_one_airborne` 的（Day 13 plan §2）。
先把「為什麼現在會同時」問清楚，再談怎麼錯開。

## 16.1 掛點確認：同一對的兩隻腳髖 x 恆等

```text
LF  x=+255.00 mm    RF  x=+255.00 mm
LH  x=-255.00 mm    RH  x=-255.00 mm
```

剛體、yaw 全程 0 -> **同一對的兩隻腳髖 x 在任何時刻都相同**。
障礙物在固定世界 x，所以兩隻腳**同時抵達**它。
freeze 文件 §7.2 那句話是對的，已查證。

## 16.2 但「同時抵達」不等於「同時離地」——平地就沒有這個問題

平地的四個 swing window **本來就錯開**，剛好鋪滿一個 cycle：

```text
LF  t/T in [0.00, 0.25)
RH  t/T in [0.25, 0.50)
RF  t/T in [0.50, 0.75)
LH  t/T in [0.75, 1.00)
```

LF 與 RF 差了半個 cycle。**所以步態本身不會讓一對腳同時離地。**
問題不在 `phase_offsets`，在越障怎麼被排程。

## 16.3 真正的原因：越障的時間【由位置決定】，不看相位

`world_schedule_2d`（`day12_world_registration_2d.py:1456`）的設計是
**時間來自位置**：一個段落的時長 = 車體把它的髖從段落起點帶到終點所需的時間。

```python
body_at_start = segment.start_contact.hip_xz_m[0] - mount
start_s = (body_at_start - origin) / speed
```

這對「四腳共用一個時鐘」是正確的設計（log §1.14 解掉 602.6 mm 的分歧）。
**但副作用是：`mount_x` 相同的兩隻腳，其越障段落落在【完全相同】的時間區間。**

```text
平地   時間由相位決定 -> LF 與 RF 差半個 cycle -> 錯開
越障   時間由位置決定 -> LF 與 RF 的 mount_x 相同 -> 完全重疊
```

**這才是 `at_most_one_airborne` 失敗的機制**，不是「幾何鎖死」那麼不可救。

## 16.4 具體的阻擋點：`target_hip_x` 是在迴圈【外面】算的

`leg_approaches_2d`（同檔 :359）：

```python
target_hip_x = terrain.x_start_m - landing_contact_offset_m   # <- 迴圈外
out = {}
for leg in LEG_ORDER:
    distance = target_hip_x - start_hip                       # <- 四隻腳同一個目標
```

`landing_contact_offset_m` 是一個**純量**，四隻腳共用。
**所以「每隻腳在不同的 x 越障」目前表達不出來。**
B3 要的就是讓它變成 per-leg。

## 16.5 錯開需要多大？（已量）

```text
車速          135.798 mm/s   （duty 0.85、週期 2.4 s）
swing window  0.360 s -> 車體只前進 48.9 mm
軸距 510 mm   後腳走到前腳位置要 3.756 s = 1.56 個 cycle
```

**關鍵數字：一個 swing window 車體只走 48.9 mm。**
要讓一對腳的離地區間不重疊，左右腳的越障落點大約要差
**這個量級**（越障段落比 swing window 長很多，所以實際需要更大）。

而 §3 已量過越障的連續離地是 **2.113 s**，
車體在那段時間走 **287 mm** —— 那才是真正要錯開的量。
287 mm 遠大於一個 stroke 的接觸前進（202.458 mm），
**所以錯開一對腳的代價是至少多一個 stroke 的行程差**，
這會直接改變 §3 那張「頂面長度能放下什麼」的表。

## 16.6 下一步（尚未做）

```text
1  把 landing_contact_offset_m 改成可 per-leg（dict 或 4-tuple）
   —— 這是 B3 的最小接口改動，但會動到 day12_world_registration_2d.py，
   而該檔【不在】凍結清單上（凍的是四腳整合層），所以可動，
   但仍應先問過專案擁有者。
2  量：左右腳差多少 x 才能讓離地區間不重疊
3  查：錯開之後支撐三角形變成什麼（可能改善也可能惡化 margin）
```

**注意 §16.5 那個 287 mm 已經預告了 B3 不是免費的**：
它會讓一對腳在越障期間相距近 300 mm，支撐多邊形會明顯變形，
而 `support_margin` 本來就是越障五項失敗之一。
**B3 與 A1（ABAD）可能是耦合的**，這與 Day 13 plan §0
「A1 的效果會被其他改動改變」是同一個道理，只是方向相反。

## 16.7 B3 接口實作（2026-09-06，擁有者已同意）

### 為什麼不是重用 `landing_contact_offset_m`

第一直覺是「把那個純量改成 per-leg 就好」。**查了它的來源之後放棄這個做法。**

`world_leg_plans_2d:1365` 是這樣算它的：

```python
entry = composed.sequence.segments[0].start_contact
landing_offset = entry.point_world_xz_m[0] - entry.hip_xz_m[0]
```

**它是【推導】出來的，不是自由參數**——是「越障序列第一個接觸點相對於
它自己的髖」的幾何量。四隻腳共用它，是因為四隻腳跑**同一條**越障序列。
把它挪作錯開之用，等於讓一個有物理意義的量身兼二職，
下次讀的人會分不清哪一半是幾何、哪一半是排程選擇。

### 實際做法：新增一個獨立的 per-leg 量

```text
day12_world_registration_2d.py
  + LegApproach2D.crossing_stagger_m = 0.0        （欄位，附完整說明）
  + target_hip_x_m 改成  obstacle_x - landing_offset + stagger
  + leg_approaches_2d(..., crossing_stagger_m: dict[LegId, float] | None)
  + world_leg_plans_2d(..., crossing_stagger_m=...)  往下傳
  ~ target_hip_x 由【迴圈外】移到【迴圈內】，因為它現在依腿而異
```

`None` / 空 dict = 完全沒有錯開 = **重現 Day 12**。

### 語意上的分工（寫進 docstring，避免以後混淆）

```text
landing_contact_offset_m   【推導】越障序列自己的幾何，四腳相同
crossing_stagger_m         【選擇】這隻腳要多滾多遠才開始越障，per-leg
```

---

# 17. 第三個軸：arc_samples（輪緣取樣密度）—— 敏感度【最大】的一個（2026-09-06）

資料：`day13_d1_arc_samples.csv`（90 格 = 30 格 x arc 121/181/241）

## 17.1 動手前先拆掉一個和步數預算同族的陷阱

`sample_match_tolerance = 3` 是**索引數**，不是角度。實測角解析度：

```text
arc_samples=121   每格 0.4435 度   -> 3 索引 = 1.330 度
arc_samples=181   每格 0.2957 度   -> 3 索引 = 0.887 度
arc_samples=241   每格 0.2217 度   -> 3 索引 = 0.665 度
```

**同樣的「3」，在 241 下容許量只剩一半。**
所以直接調高 arc_samples 會**偷偷收緊**角落樞轉的約束，
量到的會是「容許量變嚴」而不是「輪緣變細」。

修正：容許量固定成**角度**（tol 隨 arc 等比放大 -> 3/4/6 索引）。
並把 `descent_sample_match_tolerance` 用純加法接到 `TraversalConstraints2D`
（預設 3 = 現值）。

> 這是本輪第三次遇到「count vs quantity」：
> 步數預算、`sample_match_tolerance`，以及先前的接縫 guard（那個已經處理好了）。
> **這個 codebase 裡凡是整數上限，都要先問「它是次數還是量」。**

## 17.2 主結果：22/30 格會變，只有【一格】三種取樣都可行

```text
arc_samples=121   6 格可行   <- 與原表相同（基準先過）
arc_samples=181  10 格可行
arc_samples=241   4 格可行
```

**非單調（6 -> 10 -> 4），完全沒有收斂的樣子。**

```text
判定改變         22 / 30 格 = 73%
某個取樣下可行    13 格
三種取樣都可行     1 格   <- 只有 h=120 th=45
```

**「可行」這件事幾乎完全由輪緣取樣密度決定。**

對照三個軸：

```text
beta_step（§13）        5/30 格改變  = 17%
beta_search_window(§15) 0/30 格改變  =  0%
arc_samples（本節）     22/30 格改變 = 73%   <- 最大
```

## 17.3 但有一個【穩健】的結果：h=160 mm 三種取樣全部不可行

```text
h=160 可行格數    121 -> 0    181 -> 0    241 -> 0
```

**這是整份分析裡唯一站得住的結論。** 160 mm 不可行不隨取樣改變，
所以「滾動天花板 < 160 mm」是**真的**。

不成立的是另一半：「> 140 mm」。140 mm 的可行格數隨取樣在
4 / 2 / 2 之間跳，而且是不同的格子。

```text
可以講    滾動在 160 mm 不可行（三種取樣一致）
不能講    滾動在 140 mm 可行（取樣一換就換格子）
所以      天花板【上界】160 mm 成立，【下界】沒有量到
```

## 17.4 為什麼 arc_samples 的影響比 beta_step 還大

`arc_samples` 同時決定三件事，比 `beta_step` 還糾纏：

```text
1  接觸點的位置解析度      —— 哪個 sample 是最低點
2  接縫寬度               —— 已由 seam_bridge_for_sampling_m 補償
3  sample_match_tolerance 的實際角度 —— 本節已補償
```

補償掉 2 和 3 之後**還有 73% 會變**，所以主因是 1：
**「哪一個取樣點是接觸點」本身就會隨密度改變**，
而整條軌跡是靠「釘住同一個 material sample」推進的。
換句話說，接觸點的身分是離散的，而判定對那個身分敏感。

## 17.5 三個軸合起來的最終結論

```text
真正穩健的        h=160 mm 不可行（arc 三種一致）
判準對什麼敏感     arc_samples 73% > beta_step 17% > window 0%
機制             候選解與接觸點身分都被離散化綁死，
                 不是「搜尋不夠努力」（window 已證明無效）
```

**論文能寫的最強版本：**

> 滾動越障在 160 mm 於所有測試的取樣密度下皆不可行；
> 140 mm 以下的可行性判準對輪緣取樣密度高度敏感
> （30 格中 22 格隨密度改變，僅 1 格在三種密度下皆可行），
> 因此本文不宣稱一個夾緊的滾動高度上限。

這比「>140、<=160」誠實，而且**每個字都有量測支撐**。

---

# 18. B3 量測結果：`at_most_one_airborne` 解掉了，但不能就此宣告成功（2026-09-06）

資料：`day13_b3_stagger.csv`（40 mm x 400 mm，stagger 0/100/200/300/400 mm）

## 18.1 基準先過

```text
stagger 0 mm   26 個失敗、5 個 check
               at_most_one_airborne 6、three_support_legs 6、
               support_margin 10、body_requirement 2、motor_rate 2
               max_airborne 2.0、worst margin -23504.57 um = -23.5 mm
```

**與 Day 12 凍結的數字完全相同**（五項失敗、margin -23.5 mm），接線沒接錯。

## 18.2 主結果：兩個 check 消失了

```text
stagger        0      100    200    300    400   (mm)
at_most_one_airborne   6      0      0      0      0   <- 消失
motor_rate_limit       2      0      0      0      0   <- 也消失
max_airborne         2.0    1.0    1.0    1.0    1.0
失敗的 check 數        5      3      3      3      3
```

**`at_most_one_airborne` 從 6 個違規歸零，max_airborne 由 2.0 降到 1.0。**
這證實了 §16.3 的機制診斷：那一項不是「幾何鎖死」，
是**越障排程只看位置不看相位**造成的，錯開落點就能解。

而且 **100 mm 就足夠**，不需要 §16.5 估的 287 mm ——
那個估計是「連續離地時間 x 車速」，過度保守了，
因為只要兩隻腳的離地區間**不重疊**即可，不需要完全分離。

## 18.3 【但是】總失敗數從 26 漲到 44

這正是專案擁有者列的第三個陷阱
（「指標變好不等於問題解決，要看有沒有東西悄悄消失」）。

```text
              stagger 0    stagger >=100
總失敗數          26            44        <- 變【多】
three_support_legs 6            33        <- 暴增
support_margin    10             9
body_requirement   2             2
```

**`three_support_legs` 從 6 暴增到 33。** 在宣告 B3 成功之前，
必須先確認這 33 個是什麼——兩種可能性完全不同：

```text
可能 A  錯開之後某些瞬間真的只剩 2 隻腳支撐（那 B3 是失敗的，
        只是把「兩腳同時離地」換成「支撐腿不足」的另一種說法）
可能 B  是【四腳都著地】(support=4) 而非不足——
        那是排程留白，不是穩定性問題，嚴重性完全不同
```

`three_support_legs` 的判準是 `len(support_legs) != 3`，
**兩個方向都會觸發**，所以光看失敗數分不出來。

## 18.4 另外一個必須解釋的：`support_margin` 的值不見了

```text
stagger 0     worst_margin_mm = -23504.57 (um)
stagger >=100 worst_margin_mm = None        <- 有 9 個 support_margin 失敗，
                                               但沒有一個帶 value
```

`ValidationFailure2D.value` 是 `float | None`。
9 個失敗都沒有數值，表示它們是**另一種** support_margin 失敗
（可能是「算不出三角形」而非「margin 太小」）。

**這也支持可能 A/B 要先分清楚**：如果支撐腿不足 3 隻，
支撐三角形根本構不成，margin 自然沒有數值。
**若真是如此，那 9 個 margin 失敗與 33 個 support 失敗是【同一件事】，
而不是兩個獨立問題。**

## 18.5 尚未回答，正在量

已寫 `b3_detail.py` 去印出**實際的支撐腿數分佈**（而不是失敗數）、
每個 sample 的腿數、離地腿數、軌跡總時長。
在那個結果出來之前，**B3 只能說「解掉了 at_most_one_airborne，
但引入了尚未釐清的 three_support_legs 問題」，不能說成功。**

## 18.6 診斷結果：兩隻腳為什麼消失（2026-09-06）

**先更正 §18.5 的措辭。** 我先前寫「approach 失敗會【靜默】吞掉一隻腳」——
**那是錯的，Day 12 沒有這個坑。**
`world_leg_plan_2d:1212` 的 docstring 明講：

```text
A chain that failed to build comes back as a plan carrying its refusal --
not as an exception and not as a shorter chain that looks complete.
```

**失敗原因【有】被記錄**，在 `LegPlan2D.refusal`，而且是刻意的介面設計。
問題出在**我的量測腳本沒有去讀它**，只數腿數就下結論。
（我也連續三次猜錯屬性名：`validation`/`stage`、`value` 非 None、
`approach_run`。同一族錯誤：對沒讀過的資料結構做假設。）

### 真正的原因

```text
LF  segments=11  refusal=None
RF  segments=0   refusal=world registration failed -- the approach overshot:
                 it would need a hip advance of -291.260 mm,
                 and the body does not reverse
LH  segments=12  refusal=None
RH  segments=0   refusal=... -296.896 mm
```

**而且我的假設方向是【反的】。**
我猜「stagger 把目標推太遠、走不到」，實際是**走過頭了**，
需要**倒退** 291 mm。

原因：`crossing_stagger_m` 只改了「approach 要瞄準哪裡」，
但**越障序列本身**仍然被釘在相對於障礙物的固定位置
（`world_leg_plans_2d` 的 `landing_offset` 是從 `composed.sequence`
的第一個接觸點算出來的，與 stagger 無關）。

```text
approach 目標    往前挪了 300 mm     <- 我改的
越障序列位置      沒有跟著挪          <- 我沒改
=> 兩者對不起來，註冊時要倒退 291 mm 才能接上，而車體不能倒退
```

**所以 100/200/300/400 mm 四組結果完全相同，是因為它們【全都同樣地失敗了】** ——
不是「錯開量不影響結果」，是四組都只剩兩隻腳。

### 結論

B3 的失敗**不是**「錯開時機行不通」，是**我的實作只做了一半**：
改了瞄準點卻沒有把越障序列一起搬過去。
要做對的話，`crossing_stagger_m` 必須同時平移**越障序列的世界註冊位置**。

**但這條路仍然不採用** —— 見 §19。

---

# 19. B3（錯開落點）不採用，改走事件驅動排程（2026-09-06）

## 19.1 為什麼 B3 不採用 —— 與實作對錯無關

§18.6 查清楚了 B3 失敗只是我實作做了一半（改了瞄準點沒搬越障序列），
**修得好**。但即使修好也不採用，理由是需求層面的：

```text
B3 的本質      讓 RF/RH 在【不同的 x】越障
專案擁有者要求  走直線越障
=> 直接衝突，與實作正確與否無關
```

`crossing_stagger_m` 三個參數**保留但預設關閉**（`None` = 重現 Day 12），
並在此標記為**已評估、不採用**。保留而不刪除，是因為
下次有人想到同樣的點子時，可以直接看到已經量過了、以及為什麼不用。

## 19.2 專案擁有者提出的替代方案：事件驅動

> 「不能是假設 a 揮腳，然後要等到 a 任一個 rim 會觸地了，
>   才揮動下一個 b 腳嗎」

**這個提法繞開了 B3 撞的牆**：

```text
B3      改【位置】來換取錯開時間  -> 必然斜著走，違反直線越障
事件驅動 直接約束【時間】，位置不動 -> 四腳仍走同一條直線、同一個 x 越障
```

## 19.3 現在為什麼做不到（已查證）

`world_schedule_2d` 的時間完全由位置推出：

```python
start_s = (body_at_start - origin) / speed
end_s   = (body_at_end   - origin) / speed
```

**四隻腳之間沒有任何互相參照**，沒有一行在問「另一隻腳落地了嗎」。
而 `body_speed_m_s`（:1450）的 docstring 明講：

```text
One nominal cycle advances the hip by the stroke's advance plus the
recovery's, and the gait says a cycle takes cycle_period_s.
That is the whole definition -- there is no separate speed to choose.
```

**車速是單一純量，「等一下」在現在的架構裡沒有地方可以表達。**

### 這不是疏忽，是為了修另一個 bug

Day 12 Step 3 原本按**段落索引**排程，平地正確（四腳序列相同）。
越障時四腳序列長度不同（17/17/21/21 段），照索引排會把一隻腳的
7.6 s 拉成另一隻的 14.2 s，**四腳對車體位置的認知差 602.6 mm**（log §1.14）。
改成位置驅動解決了那個問題，代價就是腳與腳之間不再協調。

## 19.4 要插入多長的等待（已量）

資料：`day13_b3b_airborne_overlap.txt`

```text
LF & RF   0.033 s   擦邊
LF & RF   0.360 s   [7.897,8.257] vs [7.897,8.257]  <- 起訖【完全相同】
LH & RH   0.019 s   擦邊
LH & RH   0.360 s   [11.090,11.450] 完全相同

最大重疊 = 0.360 s = 【剛好一個 swing window】
```

**只有 4 次重疊，不是全程。** 其他時段四腳本來就錯開良好
（LF 5 個窗口、RF 4 個，多數完全不碰）。

而兩次嚴重重疊是**起訖時間一模一樣**——這正是 §16.3 診斷的機制：
`mount_x` 相同 -> 位置相同 -> 位置驅動下時間必然相同。

### 代價估算

```text
要插入的等待   0.360 s x 2 次 = 0.72 s
現有軌跡長度   13.269 s
總時間增加     +5.4%
車速           平均降約 5%（專案擁有者已確認可接受）
馬達預算       【會變好】—— 走得慢速率需求下降，而平地峰值已 95.0%
```

**代價遠小於 B3，而且不動落點，直線越障保持。**

## 19.5 動手前要先確認的（避免重蹈今天的覆轍）

```text
1  先確認那兩個 0.360 s 重合【真的是越障造成的】
   目前只知道時間完全相同，還沒確認該時刻 LF/RF 在做什麼段落
2  停頓期間支撐三角形仍要滿足 margin（車體停住、四腳著地，應為靜態）
3  停頓前後的接線不能有速度不連續（否則變成馬達的階躍指令）
4  【最重要】做完先檢查四隻腳都還在（refusal 全 None、每個 sample 四隻腳）
   【再】看 at_most_one_airborne —— 今天已經被這個順序坑過一次
```

## 19.6 這會動到凍結清單

事件驅動要讓車體軌跡能插入停頓，也就是把 `body_speed_m_s`
從單一純量變成可分段的速度曲線。這動到 freeze 清單上的
**timeline representation**（`day12_timing_skeleton_2d.py`）。
已向專案擁有者說明並取得同意，且**仍在 Day 13 做**
（Day 14 保持只做 ABAD）。

## 19.7 §19.5 第 1 項已驗證：重疊【是】越障造成的，而且更精確

不憑「讀起來像越障」下結論，直接印出那兩個窗口在做什麼段落：

```text
window [7.897, 8.257]  LF & RF
  LF  [7.756,7.897] STANCE   ROLL_DOWN       LOWER_GROUND_CONTACT
  LF  [7.897,8.257] AIRBORNE RECOVERY_SWING  CROSSING_EXIT_TRANSITION
  RF  [7.756,7.897] STANCE   ROLL_DOWN       LOWER_GROUND_CONTACT
  RF  [7.897,8.257] AIRBORNE RECOVERY_SWING  CROSSING_EXIT_TRANSITION

window [11.090, 11.450]  LH & RH
  LH/RH 兩隻都是 AIRBORNE RECOVERY_SWING  CROSSING_EXIT_TRANSITION
```

**兩次重疊都是 `CROSSING_EXIT_TRANSITION` 的 `RECOVERY_SWING`
—— 是【下障礙物之後的出口過渡】，不是爬上去。**

而且連前一段 `ROLL_DOWN` 也是逐位元同步的
（LF 與 RF 都是 [7.756, 7.897]）。這再次確認機制：
`mount_x` 相同 -> 位置相同 -> 位置驅動下所有段落的時間都相同，
不只 swing。

### 這對事件驅動的設計有直接影響

```text
不必改「怎麼爬上障礙物」——爬上去的部分沒有重疊
只需要在【出口過渡】處插入等待
每對腳一次，共兩次，各 0.360 s
```

**範圍比原本估計的更小、更明確。**
這也是為什麼要先驗證：如果照假設以為「越障全程都要錯開」，
會做成一個大得多、而且不必要的改動。

---

# 20. 事件驅動排程：`at_most_one_airborne` 解掉了，代價是零（2026-09-06）

專案擁有者提的規則：**「a 揮腳，等到 a 任一個 rim 觸地了，才揮 b」。**
資料：`day13_b3b_event_driven_schedule.txt`

## 20.1 實作：只動時鐘，不動位置

因為 §19.7 驗證出重疊只發生在**出口過渡**、各 0.360 s，
不需要把 `body_speed_m_s` 改成分段速度曲線（那是原本估的大改動），
改成**排程層的後處理**就夠：

```text
day12_world_registration_2d.py
  + airborne_overlaps_2d(schedule)
      把「哪兩隻腳在哪段時間同時離地」報成【區間】而不是失敗計數，
      這樣修正才能對著它量
  + delay_overlapping_swings_2d(schedule) -> (new_schedule, waits)
      把較晚揮的那隻腳【剩餘排程整段後推】，直到前一隻落地
```

**關鍵性質：所有髖位、接觸點、姿態完全不變。**
車體暫停，沒有任何腳被送到新的地方 -> **越障仍然是直線**。
這也避開了 B3 出事的地方（B3 動的是世界註冊，一動就對不上）。

**預設不啟用**：`world_schedule_2d` 仍回傳 Day 12 的時間，
不呼叫這個函式就重現所有凍結數字。

## 20.2 結果

```text
                    之前          之後
同時離地重疊         4 次          0        <- 解掉了
腿數                4             4
段落數              77            77
總段落時長           44.2425 s     44.2425 s   逐位元不變
軌跡總長             13.269 s      13.269 s
越障路徑             直線          直線
```

插入的等待只有三次，共 0.8878 s，**每次都恰好在「要開始揮腳之前」**：

```text
LF  +0.166 s   FOOT_RIM_ROLL -> RECOVERY_SWING
RH  +0.362 s   FOOT_RIM_ROLL -> RECOVERY_SWING
LF  +0.360 s   ROLL_DOWN     -> RECOVERY_SWING   <- 出口過渡
```

## 20.3 為什麼總時間【沒有】增加（查證過，不是 bug）

一開始看到 `end 13.269 -> 13.269 (+0.000)` 覺得說不通——
插了 0.888 s 怎麼會不變？查了逐腿的結束時間：

```text
LF   9.149 ->  9.675   +0.526
RF  10.287 -> 10.287   +0.000
LH  13.269 -> 13.269   +0.000   <- 最後結束的是它，而它【沒被延遲】
RH  12.130 -> 12.492   +0.362
                       ------
                       +0.888   全部對得起來
```

**0.888 s 完全在帳上**，只是最後結束的 LH 從未被延遲，所以 `max()` 沒動。
也就是說**等待被其他腳原本就有的餘裕吸收了**。

**代價比 §19.4 估的（+5.4%、車速降 5%）好得多：代價是零。**
專案擁有者說「車速變慢沒關係」，結果根本不用變慢。

## 20.4 那些小間隙是【原本就有的】（查證過）

```text
間隙數     之前 36 個、共 0.5168 s     之後 39 個、共 1.4045 s
差值       0.8877 s  ==  插入的等待 0.8878 s
>50 ms     之前 0 個                   之後 3 個（全部是我插的那三次）
```

**36 個小間隙（0.004-0.027 s）是 Day 12 原本的行為**，不是本次造成的，
而且一個都沒有被改動。本次只加了 3 個間隙，數值完全對得上。

## 20.5 這推翻了 freeze 文件對這一項的分類

Day 12 freeze §7.2 把 `at_most_one_airborne` 歸為：

```text
at_most_one_airborne   幾何鎖死。同 mount_x 的兩隻腳髖永遠同 x，
                       障礙物在固定世界 x -> 必然同時越障
```

**「同時抵達」是對的，「必然同時離地」是錯的。**
兩者之間隔著一個假設：**時間必須由位置唯一決定**。
那個假設來自 log §1.14 的修正（位置排程解掉 602.6 mm 的分歧），
是為了解另一個問題而引入的，不是幾何事實。

```text
正確的敘述   在【位置驅動排程】下，同 mount_x 的兩隻腳必然同時離地。
             放寬成「位置決定順序、事件決定時刻」之後就不必然。
```

而且解法**既不需要 ABAD，也不需要改變任何位置**。

## 20.6 尚未做（下一輪）

```text
1  接進 plan_terrain_2d，跑完整 Step 9 驗證
   目前只驗證了【排程層】：重疊歸零、腿沒少、段落沒少。
   還沒看 support_margin / body_requirement 在新時間軸下如何。
2  停頓期間的靜態穩定
   車體停住、四腳著地，應為靜態，但要量 margin 是否仍滿足
3  停頓前後的速度不連續
   軌跡取樣是照時間內插的，插入停頓可能造成階躍指令 -> 要查
4  平地不受影響的確認
   平地走 schedule=None 的路徑，理論上完全不經過這裡，要驗證
```

**在 1-4 做完之前，只能說「排程層解掉了重疊」，
不能說「越障少了一項失敗」。**

---

# 21. 【更正】車體【沒有】暫停，而且產生嚴重的速度不連續（2026-09-06）

資料：`day13_b3b_pause_body_speed.txt`

## 21.1 收回 §20.1 的說法

§20.1 寫「**車體暫停**，沒有任何腳被送到新的地方」。
**前半句是錯的。** 量了等待區間內的車體速度：

```text
等待期間   v = 159.76 mm/s   <- 與平常【完全相同】
```

車體照常前進。所以那不是「暫停等待」，
只是**一隻腳被排在旁邊不動，而車體繼續走**。

後半句仍然成立（沒有腳被送到新的 x），但前半句必須收回。

## 21.2 更嚴重的：第三次等待產生速度跳變

```text
t=8.003->8.083   v = -104.23 mm/s   <- 【倒退】
t=8.083->8.162   v =  462.55 mm/s   <- 2.9 倍正常速度
t=8.162->8.242   v =  213.72 mm/s
t=8.242->8.322   v =  468.55 mm/s
```

**-104 到 +462 是 566 mm/s 的跳變。** 第二次等待（RH）也有較輕微的
異常（159.76 -> 118.42 -> 34.35 -> 159.76）。

**這條軌跡不能上機。**

## 21.3 機制

`body_trajectory_2d:392-430` 的車體 x 是從**還在 stance 的腿**推出來的：

```python
active = [s for s in scheduled_all if s.start_s <= time_s <= s.end_s]
if not active:
    previous_hip_x.pop(leg, None)
    continue                      # <- 等待中的腿【被跳過】
...
absolute.append(hip_x - mounts[leg])   # 車體 x 由剩下的腿決定
```

把一隻腳的排程整段後推 -> 它在等待期間沒有 active 段落 -> 被跳過
-> **車體位置改由剩下的腿決定**，而那些腿對「車體在哪」的看法不同
-> 腿進出的瞬間車體位置重新解算 -> 跳變。

這也解釋了 §20.6 那個 `legs per sample {4:106, 3:14, 2:1}`：
**14 個少一隻腿的 sample 就是等待區間**，每次腿的進出都是一次重新解算。

## 21.4 根本問題：我只改了時間軸，沒有改車體

```text
我做的      把一隻腳的【時間】往後推
沒做的      讓【車體】知道要一起等
```

時間軸與車體軌跡是兩件事。§19.1 因為看到重疊很小（只有兩次 0.360 s），
判斷可以用排程層後處理避開「`body_speed_m_s` 改成分段速度曲線」
那個大改動——**那個判斷是錯的**。

等待要成立，車體必須真的停住，也就是速度曲線必須能表達「這段時間 v=0」。

## 21.5 目前仍然成立的部分

```text
成立   at_most_one_airborne 6 -> 0（排程層確實沒有重疊了）
成立   沒有腿消失（refusals 空、四腳都在）
成立   段落數與總段落時長逐位元不變
成立   越障路徑仍是直線（沒有任何 x 被改變）
不成立 「車體暫停」
不成立 這條軌跡可上機
存疑   失敗 26->18、margin 改善 2.6 mm
       —— 數字是真的，但它們是量在一條含速度跳變的軌跡上，
          等車體修好之後必須【重量】
```

---

# 22. 修正等待的實作：`BODY_HOLD` 段落（2026-09-06）

## 22.1 先量掉 B'：rim 弧餘裕是【零】

§21.4 之後提了三個修法，先量 B'（延長前一段 stance）：

```text
nominal stroke 的 stop_reason      RIM_ARC_EXHAUSTED
要求多 17.7% -> 實際多給           0.000 mm
要求 1000 mm -> 實際               202.458 mm（餘裕 0.000 mm）
```

**nominal stroke 已經把整個 foot arc 用光了。**
一個 stroke 的長度不是設計選擇，是 rim 弧的物理極限。

**所以 B' 不可行**，而我推薦它時說的
「要延長的量遠小於現有段落長度」**是錯的推理**：
段落的【時間】很長（0.936 / 1.176 s），但它的【rim 弧】已經用完。
**時間長不等於還能滾。**

需要多少也已經量出來（供日後參考）：

```text
等待 0.166 s -> 髖 26.521 mm -> 接觸點 16.475 mm （一個 stroke 的 8.1%）
等待 0.360 s -> 髖 57.515 mm -> 接觸點 35.728 mm （17.6%）
```

## 22.2 A 的實作比預期乾淨：不必改速度曲線

原本以為 A 要把 `body_speed_m_s` 改成分段曲線。讀 `_hip_x_at` 之後發現不必：

```python
fraction = (time_s - start_s) / span
return start + fraction * (end - start)
```

**起點與終點是同一個姿態的段落，在整段時間內回傳常數。**
而車體 x 是 `median(stance 腿的 hip_x - mount)`，
所以只要等待期間那隻腳**仍有一個「原地保持」的段落**在，
它就不會從 median 裡消失 —— §21.3 那個跳變的根因（腿進出導致重新解算）
就消失了，而且**不消耗任何 rim 弧**（起訖同姿態）。

## 22.3 新增 `SegmentKind.BODY_HOLD`，三處刻意處理

```text
1  is_terrain_transition 明確排除它
   否則新成員會 fall through 成 True，
   把「站著不動」算進論文的地形轉換計數
2  MotionSegment2D.__post_init__ 加專屬分支
   原本的 else 要求 RollSampling2D 與 rolling 描述子，hold 兩者都沒有；
   同時【強制檢查起訖髖位相同】——讓「這是不是真的 hold」
   變成機器可驗的事實，而不是註解
3  查過所有下游
   is_rolling 唯一的消費者要求 is_rolling AND is_terrain_transition，
   BODY_HOLD 第二項是 False -> 論文指標不受影響
```

已驗證 13 個 kind 的旗標，**其他 12 個完全沒變**。

## 22.4 兩個測試失敗，都是【真的】，已據實修正

```text
test_nominal_and_terrain_transition_partition_everything_but_wheel_mode
  斷言除 WHEEL_ROLL 外每個 kind 非 nominal 即 terrain transition。
  BODY_HOLD 刻意兩者皆非 -> 例外從一個變兩個。
  改法：把例外【列出來】而不是放寬斷言 ——
        「不小心兩者皆非」正是這個測試要抓的東西。

test_the_semantics_table_covers_every_kind
  要求每個 kind 都有 SEGMENT_SEMANTICS 條目，我漏了。
  同時 terrain transition 計數由 len-3 改為 len-4。
```

## 22.5 【設計比預期大】schedule 層插不進去，必須進 plan

原本想只在 schedule 插入 hold。**查了之後不行**：

```text
ScheduledSegment2D.segment_index 是 leg_plan.phased 的索引，
而【十個】呼叫點會 phased[segment_index] 解參考：
  day12_body_trajectory_2d.py:416      day12_support_stability_2d.py:255
  day12_whole_body_trajectory_2d.py:427/428
  day12_transition_mapping_2d.py:490   day12_obstacle_registration_2d.py:284/325
  day12_whole_body_animation_2d.py:178/286  ...
```

**只在 schedule 插一個沒有 phased 後盾的 hold，會讓這十處全部爆掉。**
所以 hold 必須同時進 `LegPlan2D.phased`。

`PhasedSegment2D` 只是薄包裝（segment / phase / source_label），
但 `MotionSegment2D` 需要真的接觸點與 `FrameRef2D`，
而 `FrameRef2D` 至少要一個 frame index ——
**hold 就引用它所保持的那一幀**（前一段的最後一幀），
這是一致的做法，不是憑空捏造。

## 22.6 尚未做

```text
1  產生 BODY_HOLD 段落並插進 plan + schedule（兩邊一致）
2  重跑 §20.6 的四項驗證（速度連續性是重點）
3  §21.5 標為【存疑】的數字（失敗 26->18、margin 改善 2.6 mm）必須重量
```

---

# 23. `BODY_HOLD` 的結果：結構成功，但速度跳變是【我造成的】（2026-09-06）

## 23.1 結構上成功

```text
                    OFF          ON + BODY_HOLD
BODY_HOLD 段落        0            3            <- 都插進去了
refusals            []           []            <- 沒有腿消失
legs per sample     4:119,2:2    4:117,3:4      <- 比 §21 的 3:14 改善
失敗數               26           13
at_most_one_airborne  6            0
```

`insert_holds_2d` 同時改 plan 與 schedule（`phased` 與 `scheduled` 一起，
後續 `segment_index` 一併後移），十個解參考點都沒出事。

## 23.2 但速度跳變【沒有】解決

```text
Day 12 baseline      min 159.76  max 159.76   異常步數 0     <- 完全平順
event_driven+HOLD    min -167.01 max 507.52   異常步數 6
```

**基準是完全平順的，所以跳變是本次改動造成的，不是既有缺陷。**

## 23.3 我在這件事上連續判斷錯兩次，都靠量測更正

```text
第一次   看到 -167/+507 比 -104/+462 更糟
         -> 判斷「hold 沒用，可能還有害」
         實際上：定位後發現 hold 區間【正是最平順的】
                 t=8.003->8.083 車體 -2.48 mm/s，接近靜止，正是要的效果

第二次   看到跳變發生在沒有 hold 的時刻（四隻腳都在 ROLL_UP/ROLL_DOWN）
         -> 判斷「這是 Day 12 既有的缺陷」
         實際上：基準量出來 0 個異常步，完全平順。是我造成的。
```

**兩次都是先有結論再找解釋。** 正確的做法是先量基準——
那只花了一次 8 分鐘的量測，卻可以直接排除掉一半的可能性。

## 23.4 目前對機制的理解（尚未證實）

跳變都發生在**越障期間**，而且都在 hold 區間**之外**：

```text
t=7.764->7.844  v=-107.60   LF ROLL_DOWN, LH ROLL_UP, RF ROLL_DOWN, RH ROLL_UP
t=8.242->8.322  v=+419.49   LF BODY_HOLD, RF 由 RECOVERY_SWING 轉 FOOT_RIM_ROLL
t=8.401->8.481  v=+474.54   LF 由 BODY_HOLD 轉 RECOVERY_SWING
t=9.277->9.357  v=+507.52   LF 由 FOOT_RIM_ROLL 轉 RECOVERY_SWING
```

`body_x = median(stance 腿的 hip_x - mount)`。四個值取中位數，
**當「誰是中間那個」換人時，中位數會跳。**
延遲一隻腳改變了各腿的相對時序 -> 換人的時刻改變 ->
在越障期間（四隻腳分別在不同高度、不同段落，`absolute` 四個值本來就分散）
跳幅被放大。

**注意 §21 的 world_x_spread 本來就是被監控的量**
（`body_trajectory_2d` 回傳 `world_x_spread_m`，
註解寫 "the spread is what says whether trusting it was warranted"）。
Day 12 自己就知道這個 median 只有在四腳一致時才可信。

## 23.5 下一步

```text
1  量 world_x_spread_m：基準 vs event_driven
   Day 12 已經在監控它，直接比對就知道「四腳的分歧」是不是變大
2  若是分歧變大 -> 延遲一隻腳確實破壞了四腳對車體位置的共識
   那 A 的正確做法就不是延遲【一隻】腳，
   而是讓【四隻腳一起】hold（整台車真的停住），
   那樣 median 的四個輸入同時凍結，不會換人
```

**§21.5 標為存疑的數字（失敗 26->13、margin）在跳變修好前仍然不可引用。**

## 23.6 【專案擁有者澄清】規則不是「停下來」，是「不要兩隻同時懸空」

2026-09-06，在 §23.5 提議「四隻腳一起 hold」之後，專案擁有者澄清：

> 「我的意思不是揮 a 的時候一定要停下來，如果其他往前滾也沒關係，
>   但最重要的是要確認 a 有 contact 無論 obstacle 或地面的時候才揮 b，
>   以免同時兩隻腳懸空」

**所以約束只有一條：任何時刻至多一隻腳懸空。**

```text
不要求   車體停止
不要求   其他腳停止（它們可以繼續往前滾）
只要求   b 開始揮之前，a 必須【已經有接觸】——地面或障礙物都算
```

### 這推翻了 §23.5 的提議

§23.5 想讓「四隻腳一起 hold」來凍結 median 的四個輸入。
**那超出需求了**，而且會付出不必要的代價（整台車停住）。

### 而且這解釋了為什麼 §23 的做法會產生跳變

我做的是「把 b 的**整段剩餘排程**往後推」——那不只延後了揮腳，
**也延後了 b 後面所有的 stance 段落**，於是 b 這隻腳對「車體在哪」
的貢獻整條時間軸都被平移，四腳的共識被破壞 -> median 換人 -> 跳變。

**真正需要的只是「b 的揮腳晚一點開始」，不是「b 的一切都晚一點」。**
b 在等待期間應該**繼續它原本的 stance 動作**（往前滾），
只是 swing 延後——那樣它對車體位置的貢獻是連續的，median 不會跳。

### 但這帶回 §22.1 的 rim 弧問題

b 要「繼續滾」就要 rim 弧，而 §22.1 量到 nominal stroke 的餘裕是 **0.000 mm**。

```text
所以 b 在等待期間不能【多滾】
但它可以【慢一點滾】—— 同樣的 rim 弧、拉長的時間
```

**這是還沒被測過的第三種可能**：不改位置、不改 rim 消耗，
只改那一段的**時間長度**（把 b 的 stance 段落拉長，swing 相應延後）。
這樣 b 全程都有接觸、車體位置的貢獻連續、rim 弧不變。

## 23.7 下一步（依澄清後的規則）

```text
1  量 world_x_spread_m 基準 vs 現況（確認 median 分歧是不是元兇）
2  改實作：只延後 swing 的【開始】，把前一個 stance 段落【拉長】
   而不是把整段剩餘排程平移
   —— 拉長 stance 的【時間】不需要額外 rim 弧（同樣的弧、更久的時間）
3  重新驗證：at_most_one_airborne、速度連續性、四腳都在
```

---

# 24. 為什麼只能整台車停：`world_x_spread_m` 給了決定性證據（2026-09-06）

## 24.1 Day 12 自己監控的那個量，直接指出問題

```text
Day 12 baseline    world_x_spread_m = 2.2e-16  ->   0.000 mm   四腳完全一致
延後整段排程                                        84.068 mm
拉長 stance                                        84.068 mm   <- 一模一樣
```

**兩種做法產生【完全相同】的分歧。**

`body_trajectory_2d` 的註解早就寫著：
"the legs are supposed to agree; the median is what to use and
the spread is what says whether trusting it was warranted."

**Day 12 知道這個 median 只在四腳一致時可信，而我把一致性破壞了。**

## 24.2 逐段比對確認了原因

拉長確實照設計運作（`FOOT_RIM_ROLL [2.809,3.745] -> [2.809,3.911]`，
起點不動、終點延後）。**但看四隻腳各改了幾段：**

```text
LF  19 -> 19 段，【15 段改變】
RH  21 -> 21 段，【15 段改變】
RF  17 -> 17 段，   0 段改變
LH  20 -> 20 段，   0 段改變
```

**兩隻腳的整條後續時間軸被推遲，另外兩隻完全沒動。**
於是 LF/RH 對「車體在哪」的說法與 RF/LH 差了 84 mm，
`median()` 在換人時就跳。

## 24.3 這是位置驅動架構的必然，不是實作缺陷

```text
在 world_schedule_2d 下：一隻腳的【時間】就是它對【車體位置】的說法
   start_s = (hip_x - mount - origin) / speed
=> 只動一隻腳的時間，它就必然與其他三隻脫節
```

**所以「延後整段」和「拉長 stance」失敗的原因是同一個，
而且是架構層級的，不是我寫錯。** 兩者都只動一隻腳。

唯一不產生分歧的做法：**四隻腳一起延後**。
四個輸入同時凍結 -> 分歧無法改變 -> median 不會換人。

## 24.4 實作：整台車暫停

```python
if seg.end_s <= cut:          # 暫停前：不動
elif seg.start_s >= cut:      # 暫停後：整段平移 delay
else:                         # 跨越暫停點：起點不動、終點延後
    if seg.mode is not LegMode.STANCE:
        raise ValueError(...)  # 【拒絕】把腿停在半空
```

第三個分支的拒絕是刻意的：如果暫停瞬間有腿在空中，
把它默默拉長就等於「讓一隻腳停在半空」——
而這整個 pass 存在的理由正是防止腿在空中重疊。

## 24.5 已接受的代價（專案擁有者同意）

```text
車體會【停下來】，總時間增加約 0.888 s
```

專案擁有者原本的要求是「至多一隻腳懸空」，不要求停車；
拉長那條路試過了（§23.6 的構想），量出來不可行。
擁有者已同意「先停下來，之後要優化再優化」。

---

# 25. 「整台車暫停」不收斂 —— 這是結構性衝突，不是調參問題（2026-09-06）

資料：`day13_b3b_pause_nonconvergence.txt`

## 25.1 逐次追蹤：重疊【寬度不變，只是被平移】

```text
iteration 0   LF&RF [3.7450,3.7778] 0.0329s    cut=3.6116  delay=0.1662
iteration 1   LF&RF [3.9112,3.9440] 0.0329s    cut=3.7778  delay=0.1662
iteration 2   LF&RF [4.0774,4.1103] 0.0329s    cut=3.9440  delay=0.1662
...
iteration 11  LF&RF [5.5733,5.6061] 0.0329s    <- 十二輪之後【一模一樣】
```

**四個重疊的寬度（0.0329 / 0.0189 / 0.3600 / 0.3600）從頭到尾沒有變過**，
每一輪整條時間軸被平移 0.1662 s，重疊原封不動跟著走。

## 25.2 為什麼 —— 一句話

**暫停整台車 = 對所有腿加同一個常數。**
而重疊是**腿與腿的相對關係**，加同一個常數不改變任何相對關係。

```text
暫停整台車    保住四腳共識（world_x_spread 不變）  但【解不掉重疊】
只延後一隻腿  能解重疊                              但【破壞共識】(84.068 mm)
```

**兩個需求在位置驅動架構下直接衝突。** 這不是我試錯的方式不對，
是「時間由位置唯一決定」這個前提下，兩者不可兼得：

```text
start_s = (hip_x - mount - origin) / speed
=> 腿的時間 ≡ 腿對車體位置的說法
=> 要改變腿【之間】的相對時序，就必然改變它們對車體位置的說法
```

## 25.3 已經量過的四種做法，全部失敗，原因同源

```text
1  B3 錯開落點            兩隻腳消失（approach 倒退 291 mm）  + 違反直線越障
2  延後一隻腿的整段排程    world_x_spread 0 -> 84.068 mm，車體速度 -167/+507
3  拉長一隻腿的 stance     world_x_spread 0 -> 84.068 mm（與 2 完全相同）
4  暫停整台車             共識保住了，但重疊寬度完全不變 -> 不收斂
```

**1 動位置、2/3 動一隻腿的時間、4 動所有腿的時間。**
三個維度都試過了，這不是還沒找到對的變體。

## 25.4 這代表什麼（誠實版）

`at_most_one_airborne` **在現行架構下不能只靠排程層解決**。

§16.3 的機制診斷仍然成立，而且更精確了：

```text
freeze 文件說   「幾何鎖死」                    -> 不精確
§16.3 說        「位置驅動排程的副作用」        -> 對，但我當時以為改排程就能解
現在知道        位置驅動排程【本身】就是那個約束，
                在它之內無論怎麼改時間都解不掉
```

要解它，必須動的是**產生那個排程的前提**，也就是：

```text
(a) 讓越障序列本身錯開       -> 改 Day 6-7 生成器（B2），不是排程層
(b) 放棄「時間由位置唯一決定」-> 那正是 log 1.14 為了修 602.6 mm 分歧引入的，
                                要動它得先確認不會把那個 bug 放回來
(c) 讓一對腳的 mount_x 不同   -> 硬體改動，不在範圍內
```

## 25.5 本輪的方法論教訓

在這個問題上**連續四次憑推理下結論、四次被量測推翻**
（其中兩次被自己加的 `raise` 擋下）：

```text
1  「hold 沒用甚至有害」        -> 定位後發現 hold 區間是最平順的
2  「跳變是 Day 12 既有的」      -> 量基準：0 個異常步，是我造成的
3  「cut 那一刻四腳都在地上」    -> RF 從 3.6116 就在空中，被 raise 擋下
4  「暫停整台車能解重疊」        -> 平移不改變相對關係，十二輪原地踏步
```

**共通點：每次都是先有一個看起來合理的因果故事，再去實作。**
而每一次量測都只花幾分鐘就給出確定答案。

> **下次的規則：對「時間軸／排程」這類全域結構做改動之前，
> 先寫一個十行的模擬去掃一遍，不要直接改生產程式碼再跑八分鐘。**
> 本輪 §b3t（掃描四腳著地區間）與 §b3w（逐次追蹤）都是十行腳本，
> 而它們給出的答案比前面四次修改加起來還多。

---

# 26. 越障 CSV 匯出、四個被專案擁有者指出的錯誤（2026-09-06 深夜 ~ 09-07）

## 26.1 專案擁有者對五項失敗的三點質疑，逐項查證

### (1) `motor_rate_limit` —— 「實際上是兩顆馬達在控制」

**查證：程式【已經】是對的。** `day12_whole_body_validation_2d.py:647`：

```python
for name, rate in zip(("phi_r", "phi_l"), motor_rates_rad_s(theta_rate, beta_rate)):
    if abs(rate) > MOTOR_MAX_RATE_RAD_S:
```

`phi_r = θ̇+β̇`、`phi_l = β̇−θ̇`，兩顆**各自**對 1980 deg/s 比較。

**但本方先前的敘述「|dθ|+|dβ| ≤ 1980」是錯的簡寫**，而且
`day13_step3_hardware_driver.py:241` 印出來的峰值**用的正是那個錯誤公式**
（`np.abs(np.diff(theta)) + np.abs(np.diff(beta))`）。
兩關節反向運動時那個和會**高估**。新的越障 driver 已改用正確的每馬達算法。

### (2) `support_margin` —— 「跟實際機器人物理資訊有關，不確定要不要一直卡在這」

**同意，而且理由比擁有者說的更強。** 推導鏈：

```text
重心在幾何中心         擁有者告知
誤差 ±2 mm/軸          【擁有者今日確認：是保守猜測，不是量測】
× 靈敏度 1.412 mm/mm   這個是量的
= 2.824 → 取 3 mm      floor
```

**floor 建立在一個猜測上**，而模型沒有質量、慣量、接觸力、摩擦、動力學。
本方先前寫「−23.5 mm 意味著機器會翻」——**那句話說得太滿，已收回**。
正確說法：它違反了本專案自己設定的準靜態判準，而該判準是目前唯一
在擋「抬腳那側會下沉」的東西（下沉完全沒建模）。

### (3) `body_requirement_satisfied` —— 「上下坡本來腳就不可能同高」

**擁有者是對的，本方的描述完全錯誤。** 檢查的是
`merge_demands`（`day12_body_trajectory_2d.py:333`）：

```python
hard = [d for d in demands if d.is_hard]
conflicts = ... if abs(other.body_z_m - hard[0].body_z_m) > HARD_AGREEMENT_M
```

**比的是兩隻腳各自要求【車體】在哪個高度，不是腳的高度。**
腳不等高完全正常。

`HARD_AGREEMENT_M = 1e-3`（1 mm），而實測衝突 54.28 mm = 容差的 54 倍，
所以也**不是**「容差太緊」（本方另一個猜錯的方向）。

## 26.2 【最重要】擁有者指出模型鎖死車體姿態

> 「因為我的身體不是可以傾斜嗎？為什麼會有這個衝突限制，
>   只要可以規劃到四個點差不多在同一個平面就好了吧」

查證 `day12_body_trajectory_2d.py:241`：

```python
body_rpy_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)
```

**車體 roll/pitch/yaw 是一個常數欄位，整條軌跡不變。**
不是「算出來是 0」，是**根本沒有被當成變數**。

```text
機器人   可以傾斜（剛體）
模型     假設不傾斜
=> 那 16 個「衝突」是【模型假設】造成的，不是機器人做不到
```

而且 Day 12 文件裡「需要 6.08 度俯仰」那個數字，正是
54.28 mm ÷ 510 mm 軸距 —— **模型其實已經把答案算出來了，
只是沒有那個自由度去用它。**

**所以這一項的正確定位是「模型限制」而非「物理限制」。**
擁有者補充：機身是剛體，只要不要嚴重扭轉都還可以運作 ——
用工程語言就是「共平面殘差夠小就吃得下」。

## 26.3 擁有者的第四個發現：CSV 方向反了

> 「你前後又反了，機器人現在是倒退走」

`day13_hardware_export_2d.py:117` 的 docstring 本來就寫著：

```text
Which sign is forward **on the robot** is a fact about the hardware that
the 2D model does not contain ... Reversing was confirmed by driving the robot.
```

**本方寫的越障 driver 根本沒傳 `reverse` 參數**，用了預設 `False`。
已改成**預設 `reverse=True`**（要原始符號才加 `--forward`）。

## 26.4 擁有者的第五個發現：右後與左前「爆開」

> 「感覺右後會整個爆開欸，還有左前的也是的樣子。是不是下的 command 有問題」

**是真 bug，而且是 Day 12 就存在的。** 量測：

```text
CSV 裡 theta 掉到 0.00 度（合法下限 17 度）的列數：
  FL/FR   235 / 31438 列   t=11.67-11.91s
  RR/RL   470 / 31438 列   t=24.51-24.74s   <- 兩段，所以後腳看起來最誇張
```

根因在 `day13_hardware_export_2d.py:124`：

```python
rows = np.zeros((len(samples), 12))          # 先全部填 0
for leg, leg_sample in sample.legs.items():  # 只寫【存在】的腿
    rows[row, 2*index] = leg_sample.theta_rad
```

**缺席的腿保持 0，被匯出成「完全收合」。**

而軌跡確實會缺腿：

```text
legs present per sample: {4: 118, 2: 3}
  sample  45  t=11.830  只有 ['LH','RH']    <- 前腳兩隻【成對】消失
  sample  94  t=24.667  只有 ['LF','RF']
  sample 101  t=26.501  只有 ['LF','RF']
```

**與 CSV 的折腿時段完全對應。** 缺腿是成對的，因為同對腳 `mount_x` 相同、
時序完全一致 —— 又是今天那個根本問題的表現。

平地從不觸發（每個 sample 都有四隻腿），所以這個 bug 一直沒被發現。

> **教訓**：本方今天稍早看到 `legs per sample {4:119, 2:2}` 時
> 判斷「那是 Day 12 原本就有的，無害」。**它不是無害的**，
> 它會被匯出成折斷的腿。「原本就有」不等於「沒問題」。

## 26.5 那個 8827 deg/s (446%) 的峰值

峰值出現在 **row 31590**，正落在上述折腿區間內 ——
**不是軌跡真的要那麼快，是 theta 從 0 跳回正常值造成的**。

過程中本方又猜錯一次：以為速度分段造成取樣不等距，
量出來 `min = max = 261.977 ms`（**等距**）。假設錯誤。

## 26.6 速度分段（擁有者要求，已實作）

> 「我整趟完全不用同一個速度阿，在還沒到障礙物以前可以走跟走平地一樣的速度，
>   不然超慢感覺很容易傾斜」

新增 `SpeedZone2D` 與 `time_at_body_x_2d`（`day12_world_registration_2d.py`）：

```text
原本   t = (x − origin) / speed        全程單一速度
現在   t = ∫ dx/v(x)                   分段積分，逐段累加
```

**關鍵：它套用在【車體的 x】上**，所以四隻腳讀同一個時鐘、
對車體位置的看法完全一致 —— 不會產生 §24 那個 84 mm 分歧。
這是「暫停整台車不破壞共識」那個性質的推廣（從「停」到「變慢」）。

已驗證：

```text
無 zone   t(x) == (x−origin)/speed   逐位元相同 -> 重現 Day 12
有 zone   t(1.4)=11.25、t(2.0)=15.0  與手算相符，邊界連續
```

實測生效：接近段 159.763 mm/s、越障段 39.941 mm/s，失敗數 29 → 21。

## 26.7 另一個本方引入的 bug：`--planner-hz` 傳進 `samples=`

```text
period  samples   peak deg/s
   2.4      121     16359      週期減半→速率加倍，物理正確
   4.8      121      8180
   4.8      200     25432      【同週期】改取樣數，暴增 3 倍  <- bug
   9.6      121      4090
```

`plan_terrain_2d(samples=)` 是**整條軌跡的取樣點數**（一個 count），
不是取樣率。已改用明確的 `--samples`（預設 121）。

**這是「速率是取樣相依的」（log §11.4）第四次咬人。**

## 26.8 【關鍵】擁有者問：一定要滾完整個 rim 嗎

> 「我一定得強迫其他非揮腳的腳滾完整個 rim 嗎？這樣對於過障礙物會不會有問題」
> 「也可以根據揮腳的狀況決定我到底應該要滾多少」

**不是強迫的。機制早就存在，只是 nominal cycle 從不使用。**

`run_foot_rim_roll_2d` 的 docstring（`day12_nominal_cycle_2d.py:557`）：

```text
``max_distance_m`` stops the stroke early instead.  That is what a leg
approaching a transition needs, and it is why this is a **distance request
rather than a hard-wired full arc**.
```

`leg_approaches_2d` 已經在用它（接近障礙物的腿靠它對齊落點），
但 nominal stroke 不傳，所以每次都 `RIM_ARC_EXHAUSTED`、餘裕 0.000 mm。

### 量出來的代價與收益（`d1_reserve.py`）

```text
餘裕     多出來的弧      車速
  5%      8.436 mm      96.0%   (−4%)
 10%     16.871 mm      92.1%   (−8%)
 20%     37.961 mm      82.9%   (−17%)
 30%     59.050 mm       —

stop_reason 全部變成 REQUESTED_DISTANCE_REACHED（不再被 rim 弧強制中止）
```

**對照今天量到的錯開需求：**

```text
等待 0.166 s → 接觸點需多滾 16.475 mm   <- 10% 餘裕（16.871）【剛好夠】
等待 0.360 s → 需多滾 35.728 mm         <- 20% 餘裕（37.961）【夠】
```

**10-20% 的餘裕就足以應付，而擁有者已表明沒有速度需求 —— 代價幾乎是免費的。**

### 這推翻了 §25 的結論

§25 寫「`at_most_one_airborne` 在現行架構下無解，需改生成器或放棄位置驅動時序」。

**下得太早。** 本方測的四種方法**全部在時間軸上動手腳**，
因為以為腿在空間上沒有餘裕 —— 而那個「沒有餘裕」是
**nominal cycle 自己選擇滾到弧盡頭造成的，不是幾何限制**。

```text
本方一直在問   時間上怎麼錯開？   -> 四種方法全失敗（破壞四腳共識）
擁有者提的是   空間上留餘裕       -> 四腳一致的調整，不產生分歧
```

**而且 `max_distance_m` 一直都在，docstring 明說它就是為此存在。**
是擁有者問「一定要滾滿嗎」才促使本方去查的。

## 26.9 待辦（下一輪的順序）

```text
1  修缺腿 bug
   (a) 匯出層：planner_rows_2d 遇到缺腿的 sample 拋 NotExportable，
       不要靜默填 0 —— 折斷的腿不該長得像合法指令
   (b) 上游：處理「為什麼會有空隙」
2  把 stroke reserve 做成【固定】參數（預設 0 = 重現 Day 12）
3  用 10-20% 餘裕重跑越障，看 at_most_one_airborne 是否改善
4  依擁有者要求調速度（平地 0.1 m/s、越障更慢）產新 CSV
5  尚未完成的量測：c3/c4（衝突是 pitch 還是 roll、換算角度與共平面殘差）
                   c7（峰值隨網格是否收斂）
```

### 2b【擁有者提的，比 2 更完整】依揮腳需求反推該滾多少

> 「其實也可以根據揮腳的狀況決定我到底應該要滾多少，因為過障礙物有點難預期，
>   感覺都一定要滾一定角度的 rim 會卡住揮腳的那隻」

**這與第 2 項不同，不要混為一談：**

```text
第 2 項    固定餘裕      每個 stroke 都少滾 10%，事前決定，與情況無關
第 2b 項   動態決定      先看揮腳需要什麼時機，再反推這個 stroke 該滾多少
```

現在的因果是**單向**的，揮腳沒有話語權：

```text
滾動滾到弧用完 -> 才換腳揮 -> 揮完再滾滿 -> ...
```

擁有者指出的是把因果反過來：**揮腳的需求決定滾動的長度**。
`max_distance_m` 就是那個接口，`leg_approaches_2d` 已經在對
「接近障礙物的腿」這樣用了 —— 缺的是對**一般 stance 腿**也這樣用。

**為什麼越障特別需要這個**：越障時每隻腿何時能揮很難事前預期
（取決於它在障礙物的哪個階段），固定餘裕不一定剛好夠。
動態決定則是「需要多少給多少」。

**做 2b 的前提是先有 2**（要先能傳 `max_distance_m` 進 nominal cycle），
但 2 不是 2b 的替代品。**若只做 2 就收工，等於把擁有者的提議做掉一半。**

## 26.10 本輪本方的錯誤清單（供下一個對話參考）

```text
1  「|dθ|+|dβ| ≤ 1980」            錯誤簡寫，實際是兩顆馬達各自檢查
2  「margin −23.5 mm 會翻車」       說太滿，模型答不出來
3  「前腳下坡後腳上坡，腳不等高」    完全講錯，比的是【車體】高度要求
4  「可能只是容差太緊」              1 mm vs 54.28 mm，差 54 倍
5  越障 driver 沒傳 reverse         擁有者實機驗證才發現
6  --planner-hz 傳進 samples=       速率暴增 3 倍的假數字
7  「缺腿無害」                      會被匯出成折斷的腿
8  「速度分段造成取樣不等距」         量出來是等距，假設錯誤
9  §25「架構上無解」                 下得太早，忽略了空間餘裕這個維度
```

**共通點與 §25.5 相同：先有一個看起來合理的因果故事，再去實作。**
而擁有者的每一次質疑（兩顆馬達、車體可傾斜、方向反了、腳爆開、
一定要滾滿嗎）**都指向了一個真實的問題**。

---

# 27. 缺腿 bug 的根因：段落之間有【位置】不連續（2026-09-07）

## 27.1 匯出層已修（第 1 項 a）

`planner_rows_2d` 遇到缺腿的 sample 現在拋 `NotExportable`，
不再靜默填 0。註解寫明為什麼是拒絕而非補值：
**誠實的選項只有「真實的姿態」或「沒有檔案」**，
在這裡編一個姿態出來，等於把捏造的指令放在應該是量測的地方。

## 27.2 根因（第 1 項 b）：不是鏈斷掉，是段落沒接上

```text
sample 55  t=4.1812  LF/RF 缺席
   前一段 APPROACH 結束於 4.170857
   下一段 APPROACH 開始於 4.190659    空隙 19.8 ms
sample 61  t=4.6328  同樣是 LF/RF
   ROLL_UP -> ROLL_UP                 空隙 17.3 ms
```

**不是鏈結束**（`chain spans [0.0000, 9.1490]` 涵蓋全程），
是**同一種段落之間**接不起來。成對缺席是因為 LF/RF 時序完全一致。

## 27.3 時間空隙的來源是【位置】不連續

`world_schedule_2d` 每個段落的時間都由它自己的端點推出：

```python
start_s = t(segment.start_contact.hip_x - mount)
end_s   = t(segment.end_contact.hip_x   - mount)
```

所以有時間空隙 ⟺ `前一段 end_contact.hip_x ≠ 下一段 start_contact.hip_x`。
量出來（LF，19 個段落）：

```text
 i  kind                end hip_x    next start   jump mm
 6  APPROACH             1062.7896    1065.9533    3.1637
 7  APPROACH             1065.9533    1068.6832    2.7299
 8  ROLL_UP              1136.2734    1139.0315    2.7581
 9  ROLL_UP              1158.3218    1159.2056    0.8838
10  WHEEL_TRANSITION     1175.1074    1175.7686    0.6612
11  WHEEL_TRANSITION     1401.6335    1404.1425    2.5091
12  WHEEL_TRANSITION     1534.6421    1538.9590    4.3169
13  ROLL_DOWN            1551.4782    1553.9709    2.4928
14  ROLL_DOWN            1634.4044    1635.5434    1.1390
                                      總計         20.6544 mm
```

**每個 0.66-4.32 mm，一隻腿累計 20.65 mm 的髖部「瞬移」。**

## 27.4 為什麼 `segment_chaining` 檢查【通過】—— 它沒有錯

`ChainTolerance2D.handover_hip_jump_m = 10 mm`，而它的註解說明了校準依據：

```text
Same calibration on the body side: the largest hip motion the traversal
makes in one step is 6.6 mm (the APPROACH cut).  Ten admits that and
refuses a body teleport.
```

**這些跳躍全部 < 4.32 mm，遠在容差內，所以檢查通過是正確的** ——
它們是「生成網格的一步」，不是瞬移。Day 12 的判斷沒有問題。

**問題出在下游對它的解讀：**

```text
鏈檢查說   跳躍 3 mm 在容差內，這是同一個動作   <- 正確
時間軸說   3 mm ÷ 車速 = 19.8 ms 的空白        <- 也「正確」，但那段時間【沒有腿】
匯出層說   沒有腿就填 0                        <- 錯，而且是災難性的
```

**三層各自都合理，合起來產生一條折斷腿的軌跡。**

## 27.5 修在哪一層

```text
不修生成器   3 mm 是生成網格的解析度，要它精確接合等於要求無限細的網格
不修鏈檢查   10 mm 容差是量出來的，改小會誤判正常的越障交接
修時間軸     段落之間的空白不該存在 —— 一隻腳在任何時刻都在【做某件事】，
             即使那件事是「從上一段的終點移動到下一段的起點」
修匯出層     已修（27.1），作為最後一道防線
```

**正確的修法是讓排程不要留白**：把每個段落的 `end_s` 延伸到下一段的
`start_s`（同一隻腳、同一個位置區間內），因為那 3 mm 的位移**確實發生了**，
只是生成器沒有為它產生獨立的幀。時間軸把它記成空白是不對的 ——
應該記成「上一段的最後一格再多持續 19.8 ms」。
