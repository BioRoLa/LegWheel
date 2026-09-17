# Day 13 資料索引

> **要接手這個專案 → 先讀 [`day13_handoff_zh_TW.md`](day13_handoff_zh_TW.md)。**
> 那一份是給新對話的交接文件：工作原則、機器人真實限制、B5 現況、
> 未完成的部分、以及踩過不要重踩的坑，讀完就能接手。


**狀態：主要目標已達成（2026-09-07）。**
Day 12 已凍結，見 `../day12/day12_README_zh_TW.md`。

**目標**：一份實機與模擬都能跑的越障 CSV。不是讓 Step 9 十二項全過。

**達成的**：三份越障 CSV（40/100/140 mm），超標列 0、無折腿、
**方向已驗證前進**，可上模擬。策略包絡線從「只有掃過的幾個高度」
變成 20-200 mm 連續。車體可俯仰後 `body_requirement` 整項消失。
回歸 265 passed。

**未達成的**：越障仍未通過 Step 9，剩三類失敗 ——
`support_margin`（floor 建立在【猜測】的 ±2 mm 上，模型無質量無動力學）、
`at_most_one_airborne` / `three_support_legs`（真的，排程層無解，見 findings §2）。
`body_requirement` 與 `motor_rate_limit` 已解決。
每一項在 summary CSV 裡都標明屬於哪一類。

---

## 1. 三份文件，各管一件事

```text
day13_plan_zh_TW.md        要做什麼、為什麼、待辦排序      【先讀這份】
day13_findings_zh_TW.md    做出來的結論、失敗的歸納、教訓
day13_README_zh_TW.md      這份：檔案在哪、怎麼用

archive/                   完整過程（2683 行 log）與舊版計畫
                           想知道某個決定的來龍去脈才需要翻
```

**2026-09-07 重整過**：原本只有一份 2683 行的 log，混著成功、失敗與
中途的錯誤修正，很難用。現在拆成「計畫／結論／索引」三份，
完整過程搬進 `archive/`。

---

## 2. 可以直接用的 CSV

### 2.1 平地（已驗證，Step 9 全過）

```text
hybrid_flat_v1          24796 列，走 3.1627 m，馬達 95.0%
hybrid_flat_V150_v1     18416 列，150 mm/s，馬達 89.0%
```

**這兩份是 Day 13 的基準**，兩段式拼接的平地段直接沿用 `hybrid_flat_v1`。

### 2.2 越障（模擬用，未通過 Step 9）

```text
hybrid_spliced_40mm     132.11 s   【建議用這個】
hybrid_spliced_100mm    139.63 s
hybrid_spliced_140mm    141.91 s
    平地段 = flat_v1 原封不動 + 停頓 2 s + 越障段（放慢 16 倍）
    三份都：超標列 0、theta 17.00-72.49 度、無折腿、方向已修正
    峰值 1881.65 deg/s (95.0%)，【瓶頸在平地段】而非越障段

hybrid_obstacle_40mm_SIM     單一規劃（全程含接近段）
hybrid_obstacle_100mm_SIM    同上 —— 平地段會被迫用粗網格，見下
```

**每份的 `_summary.csv` 都標明**：`SIMULATION_ONLY`、失敗了哪幾項、
每一項是**模型限制**還是**硬體事實**。上模擬前看那份就夠，不用回頭翻對話。

**單一規劃 vs 兩段式的差別**：

```text
單一規劃   平地段被迫用越障的粗網格（每點 324 ms vs flat 的 5 ms）
           -> 走平地看起來跟已驗證的 flat_v1 差很多
兩段式     平地段【完全不重新規劃】，越障段可獨立放慢
```

---

### 2.3 可規劃的範圍（2026-09-07 修正後）

```text
        L=300      L=400      L=600
 20-30   #4         #4        拒絕
 40-100  #4         #4         #1
110-140  #4/#1      #1         #1
150-200  #4        拒絕       拒絕
```

**任何高度都有答案**（不再是「只有掃過的那幾個點」）。
邊界是**資料的邊界**：高度超出 20-200 mm、或 L=600 需要
440 mm 的 takeoff（從沒量過）—— 都是缺口不是不可行。
細節見 `day13_findings_zh_TW.md` §5.4。

---

## 3. 量測資料

```text
day13_d1_grid_sensitivity.csv   搜尋步長敏感度   5/30 格翻面
day13_d1_search_window.csv      搜尋範圍敏感度   0/30 格（不敏感）
day13_d1_arc_samples.csv        輪緣取樣敏感度   22/30 格（最敏感）
day13_b3_stagger.csv            錯開落點的五種嘗試
day13_b3b_*.txt                 事件驅動排程的四次失敗過程
```

**引用前必讀**：這些證明的是「判準本身對離散化敏感」，
**不是**天花板在哪。細節見 findings §3。

---

## 4. 程式碼（`../../scripts/experiments/`）

```text
day13_step3_hardware_driver.py    平地 CSV【已驗證】
day13_step4_obstacle_driver.py    越障 CSV（單一規劃）
day13_step5_splice_driver.py      兩段式拼接【目前主力】
day13_hardware_export_2d.py       Walk 契約的匯出層
```

### 這一輪動到的 Day 12 檔案（全部是加法，預設值不變）

```text
day12_world_registration_2d.py    + SpeedZone2D / time_at_body_x_2d（分段速度）
                                  + 時間軸補洞（修折腿 bug）
                                  ~ delay_overlapping_swings_2d 改為明確拒絕
day12_nominal_cycle_2d.py         + NominalPosture2D.arc_reserve
day12_terrain_generalization_2d.py + speed_zones / event_driven 參數
day10_11_motion_schema_2d.py      + SegmentKind.BODY_HOLD
day13_hardware_export_2d.py       + 拒絕缺腿的 sample
```

**所有預設值都重現 Day 12 的凍結數字**（已逐項驗證）。

---

## 5. 常用指令

```bash
# 平地（已驗證）
python3 hybrid_note/scripts/experiments/day13_step3_hardware_driver.py \
    --metres 3.0 --planner-hz 200 --out hybrid_flat_v1

# 兩段式越障【建議用這個】
python3 hybrid_note/scripts/experiments/day13_step5_splice_driver.py \
    --height-mm 40 --hold-s 2.0 --crossing-slowdown 16

# 單一規劃越障（會讓平地段變粗）
python3 hybrid_note/scripts/experiments/day13_step4_obstacle_driver.py \
    --height-mm 40 --speed-mm-s 100 --crossing-slowdown 3 \
    --arc-reserve 0.10 --margin-floor-mm -40
```

---

## 6. 上機／上模擬前必讀

```text
CSV 格式      12 欄無表頭；欄 0-7 是 (theta, beta) x 4 腿
              （專案腿索引 0 FL / 1 FR / 2 RR / 3 RL），欄 8-11 是 gamma（0）
              前 5000 列是 cosine prep ramp，之後 1 kHz 播放
方向          reverse=True 才是前進（實機驗證過）
馬達          兩顆：phi_r = θ̇+β̇、phi_l = β̇−θ̇，各自 1980 deg/s
下沉未建模    抬腳那側會沉多少，這個模型答不出來
190 mm        五個策略全部 NOT_MEASURED，【產不出軌跡】
```
