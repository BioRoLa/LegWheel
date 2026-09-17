# Day 14 計畫：以全機為單位的越障步態規劃器（gait-first）

## 1. Context：為什麼要重整

**已經成立的東西（不重寫）**

| 層 | 狀態 | 位置 |
|---|---|---|
| 平地 nominal（FOOT_RIM_ROLL + RECOVERY_SWING，duty 0.85） | 完成、實機驗證、Step 9 12/12 | `day12_nominal_cycle_2d.py`、`hybrid_flat_v1.csv` |
| 滾動越障 primitive（右輪緣上 → 17° 輪式 → 左輪緣下） | 單腳完成 | `right_up_left_down_full_traversal_2d.check_right_up_left_down_traversal` |
| 揮腳越障的**端點**（approach clearance、落點、起跳點） | 完成 | Day 8–9 / Day 10–11 決策表 `decide_2d`、`compose_2d` |
| 用 nominal 揮法上障礙（B5） | 單腳完成，5 tests | `day13_b5_nominal_ascent_2d.run_nominal_ascent_2d` |
| 全機軌跡 schema、驗證器、硬體 CSV 匯出 | 完成 | `day12_whole_body_trajectory_2d`、`day12_whole_body_validation_2d`、`day13_hardware_export_2d` |

**卡住的地方（Day 12 §1.14 → Day 13 §33.5）**：四腳整合採「單腳先各自規劃 → 事後用位置推時間」（`world_schedule_2d`：`start_s = (hip_x − mount − origin)/v`）。同 `mount_x` 的兩腳髖 x 恆等 → 時間恆等 → 必然同時揮。五種修法（錯開落點、延後單腿、拉長 stance、暫停整台、延後車體時鐘）全部失敗，最後一次反而讓四腳對車體位置的分歧從 0 變 122.5 mm。結論已寫在 log §33.5：**必須讓四腳軌跡在生成階段就互相知道，那是重寫不是後處理。** Day 12 討論摘要（2026-09-04）也早已建議「shared body trajectory → 各 stance leg 自己維持 rolling contact」，但一直沒有實作。

**擁有者這次的要求**

1. 全機規劃不要以單腳出發，四腳的位置要同時考慮，不能再出現兩腳同時離地。
2. 越障分兩種：(a) 用**平地 nominal 的揮法**（縮到 17° → 順向旋轉 → 伸開落地）跨上／跨下；(b) 用 Day 6–7 的滾上／滾下。Day 10–11 負責選擇。
3. 頂面**不要求全程接觸**：頂面太短或太長時，可以完全離地旋轉（空中重新定位／直接揮下）。
4. 過去很多「不可行」來自一開始那個不符合真實物理的模型，不要再讓模型假設當硬牆。

## 2. 重整：哪些限制是真的，哪些只是模型的

新規劃器只把「真」的當硬約束；「模型」的一律**計算並回報，不擋輸出**。

| 限制 | 層級 | Day 14 處置 |
|---|---|---|
| 兩顆馬達 `phi_r = θ̇+β̇`、`phi_l = β̇−θ̇` 各 ≤ 1980 deg/s | **硬體** | 硬約束；由揮腳時間長度滿足（`motor_rates_rad_s`） |
| θ ∈ [17°, 160°]、β 連續、無折腿 | **硬體** | 硬約束 |
| 接觸幾何、穿透、垂直面碰撞（2D 取樣） | **幾何** | 硬約束 |
| 同一時間最多一腳離地 | **擁有者要求** | 由排程**構造保證**（事件序列化），不再事後檢查修補 |
| 段落串接不瞬移、時間軸不留白 | **正確性** | 硬約束 |
| `body_rpy = (0,0,0)` | 模型（已刪） | 唯一要求是**四個髖點共平面**（機身是剛體）；pitch 大小不設硬上限，`_pitched_body_2d` 只保留共平面殘差檢查，25° 上限降為 fit 防呆註記 |
| `support_margin ≥ 3 mm`（floor 建立在猜的 ±2 mm） | 模型、無質量無動力學 | **只回報** signed margin 與是否在三角形內，不當失敗 |
| 每隻 stance 腳各自對車體高度提 TRACK 硬需求 | 模型（因果反了） | 車體軌跡是**輸入**；stance 腳用 `hold_hip_z_profile` 跟隨 |
| 時間由髖位置決定 | 架構自找的 | **廢除**。gait clock 是唯一時鐘 |
| 頂面必須 17° 貼著滾、接觸中換 rim | 模型（接觸中換手的接縫） | 全程接觸假設拿掉：頂面上可以 nominal stroke、空中換 rim／重定位、或直接揮下。**但任何離地動作都是一個離地事件，必須排進「同一時間只有一腳離地」的序列**（擁有者 2026-09-07 確認） |
| 決策表只在掃過的高度有答案 | 已修（bracketing） | 沿用 |

## 3. 新架構：gait-first 全機規劃器（Day 14）

一句話：**車體時鐘 → 四腳當下位置 → 每隻腳依自己腳下的地形決定下一步；離地事件依 wave 順序序列化。**

```text
TerrainSpec + GaitTiming(duty 0.85, T 2.4 s, FL→RR→FR→RL) + NominalPosture(hold hip z)
        ↓
WholeBodyState(t): body_x, body_z, pitch, 每腳 (θ, β, contact, surface, mode)
        ↓  事件迴圈：依揮腳順序處理「下一個要離地的腳」
  1. 三／四腳滾動：車體以 v 前進，直到該腳到達它的起跳點
     （stance 腳的 stroke 長度 = 車體在它輪到之前會走的距離 → D1「依揮腳需求反推該滾多少」）
  2. 地形規則（per-leg Day 10–11）：這隻腳現在該做什麼
        RECOVERY（平地）| SWING_UP | SWING_DOWN | TOP_REPOSITION | SWING_OVER
        或 stance 內的 ROLL_UP / ROLL_DOWN（Day 6–7 幀依髖 x 取樣）
  3. 生成空中動作：run_recovery_swing_2d + 明確端點
        duration = max(馬達極限最短時間, 髖前進量 / v)
  4. 其餘三腳在這段時間內繼續滾（車體前進）；車體 z / pitch 由 stance 腳接觸高度擬合平面
  5. 落地 → 下一個事件
        ↓
WholeBodyTrajectory2D（既有 schema）→ 驗證（硬／註記分開）→ hardware_command_2d → CSV
```

**關鍵設計決定**

1. **時間是自變數。** `body_x(t) = ∫v dt`，`v` 沿用 `body_speed_m_s`（159.76 mm/s），允許 `SpeedZone2D` 在越障區減速；擁有者無速度需求。
2. **stance 腳逐段生成、不平移複製。** 用 `run_foot_rim_roll_2d(posture, start_beta_rad, hip_x_m, max_distance_m)`，`max_distance_m` = 車體到這隻腳輪到離地前會走的距離。平地區段仍可用 `translate_cycle_2d`（回歸不變），越障視窗內逐段生成（避開 Day 13 `follow_body` 的平移不變性問題）。若弧在輪到前就用完 → 插入減速區並記錄，不靜默。
3. **所有空中動作同一個生成器。** `run_recovery_swing_2d(stroke, config, beta_target_rad, theta_touchdown_rad, hip_z_touchdown_m)`：
   - RECOVERY：平地（現況）
   - SWING_UP：B5（已完成）
   - SWING_DOWN：B5 的鏡像——站在頂面（obstacle 進 stroke posture）、落地 `hip_z = hip_z_for_flat_stance + 0`
   - TOP_REPOSITION：頂面 → 頂面（解掉 Day 10–11 留下的 `requires_external_support`，因為現在有四腳 context）
   - SWING_OVER：地面 → 地面，跨過整個障礙
   端點（approach clearance c、landing 160 mm、takeoff）沿用 Day 10–11 表與 `standing_scene_2d`。
4. **滾動越障拆成三段，中間那段可以離地。** ROLL_UP（Step 4.5，stance）→ TOP_TRANSITION → ROLL_DOWN（Step 9R，stance）。stance 段的幀依 `hip_x` 重新參數化，在事件迴圈中以 `hip_x(t) = body_x(t) + mount_x` 取樣；它的 `hip_z(hip_x)` 是對車體的 TRACK 需求，車體 z/pitch 由此決定，其餘 stance 腳跟隨。TOP_TRANSITION 有兩種做法，由頂面長度與排程決定：
   - **接觸中**：Step 8R 的 17° 輪式滾到 `LEFT_RIM_READY`（需要 `L_transition` 的頂面長度、受接縫限制）。
   - **離地（擁有者指定的主要做法）**：`day14_airborne_rim_swap_2d`——起跳後縮到 17°、順向旋轉**剛好的角度**讓左輪緣轉到 descent-ready 的方位、落在後緣前的正確接觸點；**旋轉量與時機就是把下降接觸點對準的旋鈕**。它是一個離地事件，和其他揮腳一樣排進序列。落地後用 `_corner_readiness_failure` / `corner_states_along_transition` 驗 `LEFT_RIM_READY`，再交給 `run_left_rim_roll_down_2d`（它只讀 `left_rim_ready`、`final_frame`、`trailing_corner_world_xz_m`）。
   為了穩定性，這段不需要無時無刻接觸；要接觸也可以，兩種都保留、都要驗證。
5. **一腳離地由構造保證。** 事件依 wave 順序序列化，下一腳要等前一腳落地才能起；**頂面上的空中換 rim、重定位、揮下都算離地事件**，沒有例外。duty 0.85 下每 0.6 s 一個 0.36 s 的揮腳窗，中間 0.24 s 四腳著地——足以吸收 B5 量到的 34.5–94.5 mm 髖前進（0.22–0.59 s）。一隻腳越障期間可能需要不只一次離地（上、頂面換 rim、下），每次都各自排隊。
6. **必要修正：空中幀要看得到障礙物。** `day12_nominal_cycle_2d._airborne_frame` 目前只算對地面的 clearance 且 `collision=False`，一個刮到前緣的 nominal swing 會被放過。改用 `posture.query(scene).collision` + `query_point_to_terrain_surfaces_2d` 對所有面的最小 gap；`obstacle_xwh_m is None` 時行為不變（凍結數字不動）。
7. **驗證器分兩級。** `validate_whole_body_2d` 的 12 項拆成 HARD（timing、joint、continuity、motor rate、stance contact、chaining、terrain collision）與 ADVISORY（support_margin、body requirement）。summary CSV 每項標明層級；只有 HARD 擋輸出。
8. **輸出。** 主路徑：整條 nominal → 越障 → nominal 一次生成、均勻取樣（5 ms 等級）後 `hardware_command_2d`。`day13_step5_splice_driver` 保留為替代路徑（已驗證平地段 + 越障段）。方向一律由規劃器 `body_position_world_m` 決定（Day 13 §32.6）。

## 4. 檔案

**新增（`LegWheel/hybrid_note/scripts/experiments/`）**

| 檔案 | 內容 |
|---|---|
| `day14_gait_clock_2d.py` | `GaitClock2D`：wave 順序、離地事件序列、`body_x(t)`（含 `SpeedZone2D`）、揮腳最短時間（由幀差 + `motor_rates_rad_s` 反推） |
| `day14_leg_terrain_rule_2d.py` | `decide_leg_action_2d(leg_state, terrain, tables, strategy)`：依這隻腳**實際位置**回傳 `LegAction`（STANCE_ROLL{max_distance} / SWING_UP / SWING_DOWN / TOP_REPOSITION / SWING_OVER / ROLL_UP / ROLL_DOWN + 端點）。用 `decide_2d`／`bracketing_height_m` 取策略與參數 |
| `day14_nominal_transitions_2d.py` | 把 B5 一般化：`run_nominal_transition_2d(spec, posture, config, takeoff_pose, landing_hip_x, from_surface, to_surface)` 給 UP/DOWN/TOP/OVER；地形感知 clearance；`segment_from_nominal_transition_2d` |
| `day14_rolling_crossing_2d.py` | Day 6–7 幀 → 依 `hip_x` 索引的 stance 段（`RollingCrossing2D.at_hip_x()`），拆成 ROLL_UP / TOP_TRANSITION / ROLL_DOWN 三段，並輸出 `HipZProfile2D` 給其餘腳 |
| `day14_airborne_rim_swap_2d.py`（已有型別骨架） | 補上 `run_airborne_rim_swap_2d(roll_up_result, ...)`：以 `run_recovery_swing_2d` 起跳、縮 17°、轉到左輪緣 descent-ready 方位、落在後緣前的接觸點；回傳與 Step 8R 同型的 `WheelModeTransitionResult2D` 給 `run_left_rim_roll_down_2d`。旋轉量／落點是輸入旋鈕 |
| `day14_whole_body_planner_2d.py` | 事件迴圈；輸出每腳 `LegPlan2D`（frames 齊全）＋以**實際事件時間**建的 `FourLegSchedule2D` ＋ 含 pitch 的 `BodyTrajectory2D` → 既有 `assemble_whole_body_2d` / `validate_whole_body_2d` 不用改 |
| `day14_step{1..5}_driver.py` | 平地回歸、40 mm swing、100/140 mm roll、CSV 匯出、指標 |
| `tests/test_day14_*.py` | 見 §5 各步驗收 |
| `hybrid_note/notes/day14/day14_plan_zh_TW.md`、`day14_implementation_log_zh_TW.md` | 本計畫、逐步紀錄（擁有者規矩：一定要留紀錄、總結用中文） |

**修改（皆為加法，預設值重現凍結數字）**

| 檔案 | 改動 |
|---|---|
| `day12_nominal_cycle_2d.py` | `_airborne_frame` 地形感知 clearance（§3-6）；`run_recovery_swing_2d` 加 `landing_rim: RimId = FOOT` 參數（目前落在非 foot rim 一律 `TOUCHDOWN_ON_*_NOT_FOOT_RIM` 拒絕，空中換 rim 需要落在 left rim），預設不變 |
| `day12_body_trajectory_2d.py` | `_pitched_body_2d`：25° 上限改為註記不拒絕；硬條件只剩共平面殘差 |
| `day10_11_composer_2d.py` | 補完 `nominal_ascent=True` 的 `frame_rows`（目前標 `UNFINISHED (Day 13 B5)`，匯出器需要） |
| `day12_whole_body_validation_2d.py` | `CheckId` 加 `severity`（HARD/ADVISORY）；`ValidationReport2D.hard_failures()` |
| `day12_terrain_generalization_2d.py` | `plan_terrain_2d(..., planner="position"|"gait_first")`，預設 `"position"` 不變 |
| `day13_hardware_export_2d.py` | 不動（缺腿即拒絕的防線保留） |

## 5. 實作步驟（每步：跑 tests → 看圖 → 記 log → 再下一步）

**Step 0 — 文件與分類表**（≈1 h）
寫 `day14_plan_zh_TW.md`（含 §2 的限制分類表）與 `day14_implementation_log_zh_TW.md` 骨架。

**Step 1 — Gait clock + 平地回歸**（≈半天）
`GaitClock2D` + 事件迴圈只跑平地。驗收：與 `plan_terrain_2d(None)` 逐位元一致或 < 1e-6（120 samples、475.33 mm、2.975 s、8 recovery、margin 4.839 mm、Step 9 12/12）；匯出後與 `hybrid_flat_v1.csv` 峰值 95.0% 一致。
測試：`test_day14_gait_clock_2d.py`（事件順序 FL→RR→FR→RL、四窗鋪滿週期、平地逐位元）。

**Step 2 — 空中動作一般化 + 地形感知 clearance**（≈1 天）
`run_nominal_transition_2d` 四種；`_airborne_frame` 修正；composer `frame_rows` 補完。
驗收：SWING_DOWN 落地 z == 0 且落在頂面後方；TOP 落地 z == top_z；OVER 全程 clearance > 10 mm 且**確實對前緣量到**（反例：把障礙物移到路徑上必須被拒）；旋轉全程 θ == θ_compact；拒絕時仍記錄被問的問題。
測試：`test_day14_nominal_transitions_2d.py`（每種一個成功 + 一個反例）。

**Step 3 — 地形規則 + 事件迴圈，40 mm × 400 mm，SWING_UP → 頂面 stroke → SWING_DOWN**（≈1–2 天）
四腳一起過障礙；`max_airborne == 1` 由構造成立；車體 pitch 由平面擬合（40 mm → 4.49°）；產出 CSV、方向檢查、無折腿、超標列 0。
驗收：HARD 全過；ADVISORY 只回報；`hybrid_gait_first_40mm.csv` 可上模擬。
測試：`test_day14_whole_body_planner_2d.py`（一腳離地、四腳每 sample 都在、chaining、body 單調前進、每腳越障順序與 wave 順序一致）。

**Step 4 — 滾動越障接進 stance，頂面過渡可離地**（≈2 天）
100 mm（θ_climb 60°，已知可行 cell）與 140 mm：
- 4a：ROLL_UP → **空中換 rim**（`run_airborne_rim_swap_2d`，落 left rim、`LEFT_RIM_READY` 驗過）→ ROLL_DOWN。先量：從 roll-up 出口的 right-rim 姿態起跳，轉多少度落在後緣前 X mm 才滿足 readiness；把旋轉量與落點掃成一張小表（這正是擁有者說的「調旋轉的時間來對準下去的接觸點」）。
- 4b：ROLL_UP → 接觸中 17° 輪式（Step 8R）→ ROLL_DOWN，作為對照與短頂面備援。
- 4c：ROLL_UP → nominal 揮下（Day 10–11 的 `#2` 因為換手改在空中而解開）。
驗收：滾動腳的 `hip_z(hip_x)` 成為車體軌跡，其餘腳跟隨且 `hip_z_tracking_error_m` < 1 mm；四髖共平面殘差 < 2 mm；每一次離地都在序列裡、`max_airborne == 1`；三份 CSV（40/100/140）。

**Step 5 — 驗證分級、指標、交接**（≈半天）
summary CSV 每項標 HARD/ADVISORY 與「模型限制／硬體事實」；`day12_paper_metrics_2d` 跑三地形 + 190 mm challenge query（NOT_MEASURED 就誠實回報）；更新 `day13_handoff_zh_TW.md` 指向 Day 14；記憶檔更新。

## 6. 驗證方式

```bash
# 單元／回歸（Day 12/13 不得變）
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_day14_*.py tests/test_day13_b5_nominal_ascent_2d.py tests/test_day12_terrain_generalization_2d.py -q
```

- 平地：Step 1 的逐位元比對 + `hybrid_flat_v1` 峰值。
- 越障：driver 印出 `max_airborne`、每腳事件表（時間 | 腳 | 動作 | 起跳 x | 落地 x | 面）、車體 x 單調、pitch 曲線、HARD/ADVISORY 明細；`planner_rows_2d` 缺腿即拒絕；方向由 `body_position_world_m` 決定並檢查。
- 機器時間：四腳越障約 8 分鐘／地形，一律 `setsid python3 x.py > x.log 2>&1 < /dev/null & disown` 背景跑。
- 上模擬前看 `_summary.csv`：SIMULATION_ONLY 標記、失敗項層級。

## 7. 風險與已知邊界（寫進 log，不繞過）

- 滾動 primitive 的 `hip_z` 是輸出 → 車體必須跟著俯仰（≈ arctan(h/510)）；這是幾何必然，不是失敗。唯一的硬條件是四髖共平面。
- 空中換 rim 的落地必須同時滿足：落在 left rim、`LEFT_RIM_READY`、旋轉全程不刮頂面（實測淨空 74 mm）、且從 right-rim 出口起算的旋轉量在馬達預算內；Day 12 freeze §8 已記錄 foot-rim 出發只有 −90°..+30° 的窗口成立，right-rim 出發要重量，不能假設。
- Day 6–7 可行性判準對取樣密度敏感（22/30 翻面），Step 4 只用已知穩健的 cell（h=100/θ=60、h=120/θ=45）。
- 揮上高障礙需要的髖前進（190 mm → 94.5 mm）逼近 0.6 s 的事件間距；不足時用 `SpeedZone2D` 減速，並記錄等待。
- 2D、準靜態、無質量：下沉、柔度、動力學仍未建模；margin 只是註記。
