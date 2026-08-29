# Day 6–7 rolling research index

這個資料夾只保存 Day 6–7 的規格、可重現輸出與歷史資料；主要互動入口是上一層的兩本 notebook。

## 建議閱讀順序

1. `hybrid_gait_day6_7_v4.md`：目前 rolling-only 主線與 Step 10R–12R roadmap。
2. `../hybrid_gait_day6_7_progress_dashboard.ipynb`：Notebook A，Step 1–6.75 開發歷程、legacy baseline 與 alternatives。
3. `../hybrid_gait_day6_7_right_up_left_down_dashboard.ipynb`：Notebook B，v4 主線 Step 7R–9R 的驗證與交接入口。

`hybrid_gait_day6_7_rolling_feasibility_zh_TW.md` 是 Day 6–7 最初的研究規格，保留供追溯問題形成過程。

## 目前主線狀態

```text
Step 4.5  RIGHT_RIM_ROLL_UP                 已有獨立驗證
Step 7R   RETRACT_ON_TOP_TO_17_DEG          已有獨立驗證
Step 8R   WHEEL_MODE_TO_TRAILING_CORNER     已有獨立驗證
Step 9R   LEFT_RIM_ROLL_DOWN                已有獨立驗證
Step 10R  COMPOSE_FULL_TRAVERSAL             待實作
Step 11R  HEIGHT_X_THETA_SWEEP               待實作
Step 12R  TOP_LENGTH_TRANSITION_ANALYSIS     單一條件雛形，待一般化
```

「已有獨立驗證」表示 single-leg 2D sampled kinematic/contact feasibility；不代表 whole-body stability、motor capability、dynamics 或完整 planner 已驗證。

## 輸出命名

- `day6_7_roll_up_end_state.json`：Step 4.5 成功終點的 canonical cache。
- `day6_7_step4_5_*` 至 `day6_7_step6_75_*`：Notebook A 的歷史／替代策略輸出。
- `day6_7_step7r_*` 至 `day6_7_step9r_*`：Notebook B 的 v4 主線輸出。
- `*_summary.csv`：stage 或 branch 摘要。
- `*_frames.csv`：完整逐幀資料；notebook 畫面只顯示摘要與首尾預覽。

## Archive

- `archive/notes/`：已被 v4 取代的 v2/v3 與 2026-08-26 handoff snapshot。
- `archive/right_rim_only_roll_down/`：被 right-up/left-down 主線取代的 right-rim-only trailing-edge 探索。

Archive 內容僅移動、不刪除，仍可用於 paper 的設計演進、negative result 或 ablation 敘述；不可當成目前主線結果。
