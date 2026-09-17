# Day 12 資料索引與收尾

**狀態：Step 0-12 全部完成，已凍結（2026-09-05）。**
之後的工作在 `../day13/`。

這份檔案的用途：**一年後的你、或一個沒有上下文的 Claude，
從這裡就能找到所有東西，而且知道每個數字是什麼意思。**

---

## 1. 先讀哪一份

```text
想知道 Day 12 做完是什麼狀態、Day 13-14 要接什麼
    -> day12_step12_freeze_and_handoff_zh_TW.md      【最重要，先讀這份】

想知道某個決定為什麼是那樣、某個數字怎麼來的
    -> day12_implementation_log_zh_TW.md             6413 行，按時間順序

想知道原始規格要求什麼
    -> hybrid_gait_day12_whole_body_integration_FINAL_zh_TW.md

想知道 2026-09-04 那次 formulation 討論
    -> hybrid_gait_day12_discussion_summary_zh_TW.md
       （已加註記：其中三個前提在討論當下就已經過期，見文內方括號）
```

---

## 2. 一句話狀態

```text
平地    Step 9 全過（0 失敗），可上機 CSV 已產出並驗證
越障    5 項驗證失敗，其中 2 項幾何鎖死、1 項才是 ABAD 的事
規格    §17 完成條件【達成】（同步四腳軌跡 或 結構化不可行）
        §20 checklist 22 項中 18 項成立
回歸    1153 passed / 9 failed，day12 與 day13 【零失敗】
```

**Day 12 的完成條件不是「越障成功」**，是「產生同步四腳軌跡**或**
結構化的不可行結果，且核心 planner 無地形尺寸專用邏輯」。達成的是
**平地成功、越障結構化不可行**。

---

## 3. 十二個 Step 各自的結論

```text
Step 0   segment 語意契約（SegmentKind 12 個成員）           完成
Step 1   nominal cycle 生成器                                完成
Step 2   四腳世界座標 + 參數化地形註冊                        完成
Step 3   時間軸                                              完成
Step 4   per-leg sequence -> 共同時間軸                       完成
Step 5   body requirement 合併          結論 INFEASIBLE（平地已解，見 Day 13 §3）
Step 6   support triangle + margin      duty 0.75 下 0.000 mm 是【定義值】
                                        duty 0.85 -> 4.839 mm，STABLE
Step 7   TOP_REPOSITION                 planning floor 下 support 不足
Step 8   完整四腳軌跡                                        完成
Step 9   whole-body validation          平地 12/12；越障 5 項失敗
Step 10  唯一入口 + generalization gate  四地形一入口，gate 乾淨
Step 11  paper metrics                                       完成
Step 12  Freeze 與 Handoff               七項要求全達成，已凍結
```

---

## 4. 資料檔清單（全部是 2026-09-04/05 重跑的）

### 4.1 各 Step 的產出

```text
day12_step0_segment_semantics.csv     11 個 kind 的語意表
day12_step0_boundary_evidence.csv     10 個 boundary，CUT/HANDOVER 分開
day12_step0_boundary_evidence.png
day12_step1_cycle_summary.csv         每個 cycle 一列
day12_step1_cycle_frames.csv          334 幀 x 2 cycle
day12_step1_arc_start_evidence.csv    roll/pivot 邊界的量測
day12_step1_phase_plot.png
day12_step1_cycle_animation.gif
day12_step2_leg_mounts.csv            四個掛點
day12_step2_four_leg_state.csv        body + 四腳 + 對稱檢查
day12_step2_terrain_sweep.csv         同一份 code、五組地形參數
day12_step2_four_leg_state.png
day12_step3_schedule.csv              timing + 16 segment + conflicts
day12_step3_rate_demand.csv           三個週期下的 rate demand
day12_step3_timeline.png
day12_step4_leg_plans.csv             每個 cell 一列
day12_step4_debug_table.csv           規格 §11 要求 8 的表
day12_step4_timeline.png
day12_step5_body_trajectory.csv       取樣 + concession + conflict
day12_step5_body_trajectory.png
day12_step6_stability.csv             traversal/swing/sample 三種列
day12_step6_stability.png
day12_step7_reposition.csv            attempt/support_gate 兩種列
day12_step8_whole_body.csv            summary/assumption/handoff/sample
day12_step9_validation.csv            summary/check/failure/... 七種列
day12_step10_generalization.csv       規格 §17 要求 7 的比較表
day12_step10_size_literals.csv        generalization gate 掃到的每一行
day12_step11_metrics.csv              每個地形一列 + provenance
day12_step12_regression.log           全套回歸原始輸出（1:51:44）
```

### 4.2 排查專用

```text
day12_a4_margin_scan.csv / .png       A4：support margin 掃描
day12_b1_decision_criterion.csv/.png  B1/B2：決策準則比較
day12_obstacle_registration.csv/.png  附錄 B：越障世界座標註冊
day12_whole_body_animation.gif        附錄 A：全機動畫
day12_obstacle_animation.gif          附錄 B：越障動畫
day12_whole_body_frames.png / day12_obstacle_frames.png
day12_*_viewing_height.csv            動畫的視角高度
```

---

## 5. 【重要】哪些數字是哪個配置量的

這是最容易誤讀的地方。

```text
duty 0.75 + 固定 theta（Day 12 原始 / legacy）
    Step 0-11 的 CSV 全部是這個配置
    plan_terrain_2d(legacy_configuration=True) 可重現
    Step 5 227 個 body conflict、Step 6 margin 0.000 mm 都出自這裡

duty 0.85 + 水平化姿態（Hybrid 選定配置）
    Step 10/11 的 CSV、Day 13 的所有 CSV
    plan_terrain_2d() 預設就是這個
    margin 4.839 mm、Step 9 平地 12/12 出自這裡
```

**同一個 Step 的數字在兩個配置下不一樣，引用時一定要說是哪一個。**

---

## 6. 已知限制（上機或寫論文前必讀）

```text
模型是   2D 矢狀面、準靜態、幾何
有的     剛體機身、真實接觸幾何、無滑動滾動、關節行程、
         馬達速率預算、支撐三角形與 margin
沒有的   質量、慣量、接觸力、摩擦、柔度、動力學、
         車體 roll/pitch/yaw（全程 0）、ABAD（gamma 鎖 0）

抬腳那側會下沉  —— 完全沒有建模
    現在擋它的是靜態穩定要求（margin >= 3 mm floor），
    而那 3 mm 只涵蓋【重心量測誤差】，不涵蓋下沉/柔度/動力學。
    平地剩下 4.839 - 3 = 1.839 mm 要留給所有沒建模的東西。

馬達 95.0%  平地 CSV 峰值 1881.6 / 1980 deg/s，餘裕 5%
            （72.0% 那個舊數字是粗網格低估，見 Day 13 §11.4）
車速不是參數  159.763 mm/s = cycle 髖部前進 / 週期
duty 0.75 是奇異點  margin 正比於 duty - 3/4，在那裡恆為零
19 cm      從未掃過（資料缺口，不是不可行）
```

---

## 7. 程式碼位置

```text
所有 Day 12 模組    ../../scripts/experiments/day12_*.py
所有 driver         ../../scripts/experiments/day12_step*_driver.py
測試                ../../../tests/test_day12_*.py

唯一入口            day12_terrain_generalization_2d.plan_terrain_2d
凍結清單            見 freeze 文件 §9
```

---

## 8. Day 12 期間動到的非 Day 12 檔案

```text
legwheel/planners/hybrid/terrain_query_2d.py
    效能改寫（71.02 -> 25.32 us/point），介面不變
    有 9288 列 golden 資料做【逐位元】等價測試
day10_11_decision_map_2d.py / day10_11_composer_2d.py
    加 body_tolerance_m 參數（加法，預設保持原行為）
day10_11_motion_schema_2d.py / day10_11_sequence_builders_2d.py
    加法與純抽取，見 log §1
```
