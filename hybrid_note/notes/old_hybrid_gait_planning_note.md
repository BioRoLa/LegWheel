# Corgi 舊版 Hybrid 步態規劃筆記

> 這份筆記整理舊 Corgi 輪腿機器人的 Hybrid / WLW 步態規劃流程，方便快速複習，也作為之後設計新腿部步態的參考。
>
> 內容以目前 workspace 的實作為準。舊程式的命名有歷史包袱，不能直接當成新機器人的最佳介面設計。

## 1. 一頁總覽

目前主要的 WLW runtime 路徑：

    wlw_open.cpp
        -> 讀設定、執行 1 kHz 主迴圈、發布馬達命令
    GaitSelector / Simple_fsm
        -> 保存共享的姿態、相位與步態參數
    Hybrid::Initialize()
        -> 設定四腳初始 duty 與第一組 next_eta
    Hybrid::Step()
        -> 支撐腳：LegModel::move()
        -> 擺動腳：HybridSwing::generate() / Swing_step()
    GaitSelector::next_eta[i] = {theta, beta}
        -> motor/command

一句話：

> 支撐期保留輪腿與地面的接觸，利用 LegModel::move() 配合身體前進；擺動期讓輪腿離開地面，利用 HybridSwing 從目前的 theta/beta 姿態移到下一個目標姿態。

目前的 wlw_open 不是完整的地形感知落腳點規劃器。它主要使用速度、步長、站立高度和步態相位等局部參數，不會自動從地圖決定每隻腳的世界座標落腳點。

## 2. 建議閱讀順序

### 目前 WLW / Hybrid 主路徑

1. src/corgi_mpc/src/wlw_open.cpp
2. src/corgi_gait_selector/include/corgi_gait_selector/Simple_fsm.hpp
3. src/corgi_gait_selector/src/Simple_fsm.cpp
4. src/corgi_gait_generate/corgi_hybrid/src/hybrid_gen.cpp
5. src/corgi_gait_generate/corgi_hybrid/src/hybrid_swing.cpp
6. src/corgi_utils/src/leg_model.cpp

### 只想看單隻腳擺動

    Hybrid::Swing()
        -> HybridSwing::generate()
        -> Hybrid::Swing_step()

也可參考實驗檔：

    src/corgi_gait_generate/corgi_hybrid/src/Single_leg_traj copy 2.cpp

### 一般 Walk 對照

    src/corgi_walk/src/walk_gait.cpp

一般 Walk 的擺動期使用 SwingProfile / Bezier 類腳端曲線，支撐期同樣呼叫 LegModel::move()。

## 3. 重要資料與座標

### eta 與 next_eta

每隻腳有：

    eta[i][0]       // 目前 theta
    eta[i][1]       // 目前 beta
    next_eta[i][0]  // 下一個控制週期的 theta
    next_eta[i][1]  // 下一個控制週期的 beta

Step() 通常只計算下一個控制週期的 next_eta，不會一次完成整個擺動期或完整一步。wlw_open 發布 next_eta 後，再把它複製回 eta。

### duty 與 swing_phase

duty[i] 是第 i 隻腳在一個步態週期中的相位：

    0.0 ---------------- 1.0
           支撐期       擺動期

若 swing_time = 0.2：

    duty 0.0 ~ 0.8：支撐期
    duty 0.8 ~ 1.0：擺動期

    swing_phase[i] == 0  // 支撐期
    swing_phase[i] == 1  // 擺動期

swing_index 只決定初始化時哪隻腳先進入擺動相位，不代表之後永遠只有那一隻腳擺動。

### stand_height、step_height、foothold 高度

不要混在一起：

    stand_height：
        身體與支撐腳之間的相對幾何高度

    step_height：
        腳離地擺動時的 clearance / 抬腳高度

    target_foothold_z：
        世界座標中預期的落腳高度

舊程式的 current_stand_height[i] 是給腿部幾何使用的相對高度，不是完整的世界座標落腳點。若要做崎嶇地形的 open-loop，較好的外部介面是：

    target_foothold_position[i]
    // 或 target_foothold_height[i]

再依預定的 body pose 轉成腿部模型需要的相對位置或 target_stand_height[i]。

## 4. Hybrid::Initialize()

Initialize() 通常在開始行走前呼叫一次。

### 4.1 設定四腳初始相位

例如 swing_index = 1：

    gaitSelector->duty = {
        0.5 - swing_time,
        1.0 - swing_time,
        0.0,
        0.5
    };

若 swing_time = 0.2：

    RF：0.3
    LF：0.8，接近先擺
    LH：0.0
    RH：0.5

這不是直接寫一個固定的擺動順序，而是用四腳的相位差形成交錯節奏。

### 4.2 計算開始前的四腳姿態

    auto tmp0 = find_pose(
        current_stand_height[i],
        current_shift[i],
        step_length,
        duty[i],
        0
    );

    next_eta[i][0] = tmp0[0];
    next_eta[i][1] = tmp0[1];

也就是先把每隻腳放到正確的起始步態相位與 theta/beta 姿態，再開始呼叫 Step()。

## 5. Hybrid::find_pose()

核心起點：

    double pos[2] = {0, -height + leg_model.r};
    pose = leg_model.inverse(pos, "G");

pos[0]、pos[1] 是腿部局部座標中 G 點的位置：

    pos[0]：水平位置
    pos[1]：垂直位置，向下為負

"G" 表示使用輪腿模型中 G 點的反向運動學。它是建立初始幾何姿態的參考點，不代表所有地面接觸永遠只在 G。

因為 G 點和輪胎最低接觸點有半徑差，所以：

    G 點高度 = -height + r

### 5.1 先移到步態起始位置

    for (double i = 0;
         i < shift + step_length * (1 - swing_time) * 0.5;
         i += 0.001) {
        pose = leg_model.move(
            pose[0], pose[1],
            {-0.001, 0},
            0
        );
    }

這是在局部模型中以 1 mm 一次呼叫 move()，把姿態往負 x 方向移到支撐相位的起始位置。

### 5.2 模擬目前 duty 已經走過的支撐位移

    for (double t = 0.0;
         t <= duty;
         t += incre_duty) {
        pose = leg_model.move(
            pose[0], pose[1],
            {dS, 0},
            slope
        );
    }

最後得到目前步態相位應使用的 theta/beta，不是單純的固定站立姿態。

## 6. Hybrid::Step() 完整流程

每次 Step() 是一個控制週期，通常是 1 ms 的小更新，不是完整擺腿或完整一步。

### 6.1 更新 body / hip 與 duty

    for (int i = 0; i < 4; i++) {
        next_hip[i][0] += dS;
        duty[i] += incre_duty;
    }

其中：

    dS = velocity / pub_rate
    incre_duty = dS / step_length

所以速度以每個控制週期的小位移累積，duty 也逐步前進。

### 6.2 判斷是否進入擺動期

    if (duty[i] > (1 - swing_time)
        && swing_phase[i] == 0) {
        swing_phase[i] = 1;
        ...
    }

意思是：這隻腳原本在支撐期，現在相位進入最後的 swing_time 比例，開始離地擺動。

進入擺動時會：

1. 決定這一腳的步長。
2. 呼叫 find_pose() 算 swing_pose。
3. 呼叫 Swing() 建立完整擺腿軌跡。

    swing_pose = find_pose(
        current_stand_height[i],
        current_shift[i],
        step_length,
        0.0,
        0
    );

    Swing(gaitSelector->eta, swing_pose, swing_variation, i);

前腳通常採用 new_step_length；後腳會參考對側前腳的目前步長：

    int last_leg = (i + 2) % 4;
    step_length = current_step_length[last_leg];

這是舊版步態的協調規則，不是所有新腿都必須沿用。

### 6.3 判斷擺動結束 / 觸地

    else if (duty[i] > 1.0) {
        swing_phase[i] = 0;
        duty[i] -= 1.0;
        current_step_length[i] = next_step_length[i];
    }

也就是：

    擺動期結束
        -> 回到支撐期
        -> duty 減 1，進入下一個週期
        -> 下一步長度變成目前步長

### 6.4 支撐腳更新

    if (swing_phase[i] == 0) {
        leg_model.forward(
            eta[i][0],
            eta[i][1],
            true
        );

        result_eta = leg_model.move(
            eta[i][0],
            eta[i][1],
            {
                next_hip[i][0] - hip[i][0],
                next_hip[i][2] - hip[i][2]
            },
            0
        );

        next_eta[i][0] = result_eta[0];
        next_eta[i][1] = result_eta[1];
    }

意思是：

    腳仍接觸地面
        -> 身體前進一小段
        -> LegModel::move() 維持接觸幾何
        -> 反算新的 theta、beta

這是 Hybrid 滾動／支撐部分的核心。

### 6.5 擺動腳更新

    else {
        Swing_step(
            swing_pose,
            swing_variation,
            i,
            duty[i]
        );
    }

擺動腳不呼叫 move()，而是從已建立的 swing_traj[i] 取出目前相位的軌跡點。

最後：

    hip[i] = next_hip[i];

## 7. 擺腿：Swing、HybridSwing::generate、Swing_step

### 7.1 Swing：建立單腳完整軌跡

起點是該腳目前的：

    theta_start = eta[swing_leg][0]
    beta_start  = eta[swing_leg][1]

終點是 find_pose() 算出的：

    theta_end
    beta_end

Swing() 也會先分析目標姿態：

    leg_model.forward(endX, endY, false);
    leg_model.contact_map(endX, endY);

取得目標姿態的：

    rim
    alpha

再呼叫：

    swing_traj[swing_leg] = HybridSwing::generate(
        leg_model,
        swing_type,
        startX, endX,
        startY, endY,
        rim, alpha,
        body_velocity,
        terrain,
        500
    );

### 7.2 HybridSwing::generate：在 theta-beta 空間規劃

目前可選：

    LINEAR
    CUBIC
    FIVETIMES
    OPTIMIZE

預設是：

    SwingType::FIVETIMES

CUBIC / FIVETIMES 的概念：

    theta：目前姿態 -> 收到約 17° -> 伸到目標 theta
    beta ：從 beta_start 平滑走到 beta_end

五次 smoothstep：

    10t^3 - 15t^4 + 6t^5

它讓擺動起點與終點的 beta 速度較平滑，減少速度突變。

這是關節角度空間軌跡，不是直接指定腳端的 x/y/z 曲線。

### 7.3 Swing_step：每個控制週期查表

    ratio =
        (duty_ratio - (1 - swing_time))
        / swing_time;

    ratio = clamp(ratio, 0.0, 1.0);
    idx = static_cast<int>(ratio * 500);

    next_eta[swing_leg][0] = swing_traj[swing_leg][idx].theta;
    next_eta[swing_leg][1] = swing_traj[swing_leg][idx].beta;

所以：

    generate()   -> 產生整條軌跡
    Swing_step() -> 隨 duty 查表執行

## 8. OPTIMIZE 策略

OPTIMIZE 仍然主要在 theta/beta 空間工作，但加入局部接觸幾何補償：

1. 由終點姿態、rim、alpha 算出輪緣目標接觸點 P_end。
2. 由原始軌跡最後一點附近估計 P_prev。
3. 比較接觸點沿地面方向的速度和 body velocity。
4. 計算 eps，用 r + eps*r*(1-r) 扭曲 theta 的時間進度。
5. 限制相鄰軌跡點的 theta 變化最多約 1 度。
6. beta 使用線性與五次曲線的混合。
7. 最後用 alpha 的數值修正微調終點。

它不是完整的全域最佳化，而是：

    接觸幾何補償
    + 時間分配調整
    + 角度變化限制
    + 終點 alpha 修正

目前 OPTIMIZE 區塊有外層 generate() 迴圈與內層產生整條軌跡的迴圈重疊，會重複 push_back 很多資料。若之後要使用，應先整理成一次呼叫只生成一條軌跡。

## 9. LegModel::move：支撐與輪框接觸核心

move() 不是普通的腳端位置插值，而是：

    目前 theta、beta
        -> contact_map() 找目前接觸 rim
        -> 固定該 rim，建立接觸下的位移模型
        -> Newton solver 解 dtheta、dbeta
        -> 回傳新的 theta、beta

### 9.1 先呼叫 contact_map

    contact_map(
        theta_in,
        beta_in,
        slope,
        contact_upper,
        contact_lower
    );

    int contact_rim = rim;

可能的 rim：

    1 = U_l
    2 = L_l
    3 = G
    4 = L_r
    5 = U_r

同一次 move() 的 Newton 迭代中，contact_rim 會固定。下一次呼叫 move() 才會重新 contact_map()，所以 rim 的切換通常發生在兩次 move() 之間。

### 9.2 objective 如何使用 rim

對不同 contact_rim，objective() 使用不同機構點建立接觸模型。概念都相同：

    theta/beta 改變
        -> 接觸方向 alpha 改變
        -> alpha 變化 × 接觸半徑
        -> 得到接觸下的滾動距離
        -> 和目標 move_vec 比較

以 G 分支為例：

    double d_alpha = ...;
    double roll_d = d_alpha * r;

這表示同一個 rim 上的接觸位置可以連續改變；它不是只有一個固定接觸點。

### 9.3 Newton 數值求解

初始猜測：

    guess_dq = {0.0, 0.0};

其中：

    guess_dq[0] = dtheta
    guess_dq[1] = dbeta

每一輪：

1. 把猜測的角度變化送入 objective()。
2. 取得 x/y 位移誤差。
3. 用數值差分建立 Jacobian。
4. 解 Jacobian * dq = -error。
5. 更新 dtheta/dbeta。
6. 直到誤差或修正量小於容許值。

最後：

    theta += guess_dq[0];
    beta  += guess_dq[1] + slope;

    return {theta, beta};

## 10. contact_map 如何選 rim

contact_map() 是純幾何推測，不是力感測器。

流程：

    theta、beta、slope
        -> forward() 算出機構點
        -> 建立五個候選 rim
        -> arc_min() 檢查每段是否含最低方向
        -> 算每段可能的最低高度
        -> 選最低的有效候選

五個候選區段：

    U_l：UH_l -> UF_l
    L_l：LF_l -> LG_l
    G：   LG_l -> LG_r
    L_r：LG_r -> LF_r
    U_r：UF_r -> UH_r

arc_min() 對每段回傳：

    lowest_point
    alpha
    contact_x

如果一段 rim 的端點範圍涵蓋輪子向下的方向，就認為它可能接地；否則把 lowest_point 設成 1.0 作為無效值。

最後選最低的有效候選，寫入：

    rim
    alpha
    contact_p

因此它不是永遠 G 點觸地。G 只是很多初始反向運動學與站立姿態使用的參考點。

## 11. 一般 Walk 與 Hybrid

目前一般 Walk 也不是完全不用 move()：

    Walk 擺動期：SwingProfile / Bezier -> inverse()
    Walk 支撐期：LegModel::move()

    Hybrid 擺動期：HybridSwing -> theta/beta 軌跡
    Hybrid 支撐期：LegModel::move()

主要差異是擺動期的規劃空間：

    一般 Walk：較偏腳端 Cartesian 曲線
    Hybrid：較偏 theta/beta 關節角度曲線

move() 是輪腿機構的接觸保持運動函式，不是只有 Hybrid 才能使用。

## 12. Bezier 與 HybridSwing

### HybridSwing

直接規劃：

    theta(t), beta(t)

優點：

- 直接輸出馬達角度。
- 容易限制 theta/beta 的變化量與角度範圍。
- 和 contact_map()、輪腿模型接得自然。
- 適合目前固定輪腿幾何的 Hybrid 擺腿。

缺點：

- 不容易直觀看出腳端的 x/y/z 路徑。
- 不容易直接指定腳要抬多高或落在世界座標哪裡。

### Bezier / SwingProfile

較偏規劃：

    腳端 p(t) = {x(t), y(t)}

再經過：

    腳端位置 -> inverse() -> theta/beta

優點：

- 直觀控制起點、終點、抬腳高度與中間路徑。
- 適合指定 foothold 或跨障礙。
- 對崎嶇地 open-loop 較容易設計。

缺點：

- 每個軌跡點都要經過反向運動學。
- 要檢查 inverse 是否有解、角度是否超限、接觸幾何是否合理。
- 不能直接取代支撐期的 rim 接觸與滾動模型。

未來若要做崎嶇地 Hybrid，合理的混合架構：

    擺動期：Bezier / 腳端 foothold 軌跡
            -> LegModel::inverse()
            -> theta/beta

    支撐期：LegModel::move()
            -> 維持接觸與滾動幾何

## 13. wlw_open 的實際控制迴圈

主迴圈大致是：

    hybrid_gait.Step();

    // 發布 swing_phase 狀態
    swing_phase_pub->publish(swing_phase_msg);

    // 將 next_eta 填入 motor command
    motor_cmd_modules[i]->theta = next_eta[i][0];
    motor_cmd_modules[i]->beta  = next_eta[i][1];

    // 真正送出馬達命令
    motor_cmd_pub->publish(motor_cmd);

重要區分：

    Hybrid::Step()：
        計算步態與下一個姿態

    swing_phase topic：
        發布目前狀態，不會直接讓馬達揮腳

    motor/command：
        真正送出 theta/beta，讓馬達執行

## 14. 舊版實作限制

### 沒有自動的世界座標 foothold 規劃

主 WLW 路徑不會自動從地形感測器取得每腳下一個世界座標落腳點。若要走崎嶇地，需要：

    地形或接觸資訊
        -> 每腳 target_foothold_position
        -> body frame 轉換
        -> inverse 或 target_relative_foothold
        -> 擺腿軌跡

### next_stand_height 沒有完整接上

資料結構中有 next_stand_height，但主要 Hybrid 流程常用的是 current_stand_height[i]。若要真正使用下一步高度，應明確設計：

    規劃器寫入 next_stand_height[i]
    腳進入 swing 前鎖存目標
    觸地後 next -> current

### step_height 沒有完整接到 HybridSwing

一般 Walk 的 step_height 直接進入 SwingProfile。目前 corgi_hybrid 的 HybridSwing 主要用 theta/beta 曲線，沒有像 Cartesian Bezier 一樣直接以腳端高度規劃 clearance。

### contact_map 是模型，不是真實回饋

它根據模型選理論上最低的 rim。若地面不平、輪胎變形或馬達誤差很大，理論 rim 可能和真實接觸不同。要提高可靠性，需要力、電流、觸地或其他回饋。

## 15. 新腿步態的建議分層

不要把舊版 stand_height 直接當成新機器人的唯一輸入。建議分三層。

### A. 外部步態／地形規劃層

    target_foothold_position[leg]
    target_foothold_velocity[leg]
    target_body_pose
    step_duration
    swing_clearance

回答：

    這隻腳下一步要落在哪裡？
    要抬多高？
    身體要怎麼移動？

### B. 接觸與運動學層

    contact_map()
    forward()
    inverse()
    move()

回答：

    目前哪裡接觸？
    目標腳端位置需要什麼關節角？
    支撐時身體移動需要什麼關節補償？

### C. 控制輸出層

    theta_cmd[leg]
    beta_cmd[leg]
    motor/command

只負責把規劃結果轉成馬達可執行命令。

## 16. 新 Hybrid 步態的最小設計清單

### 幾何模型

- 定義機構座標系與正方向。
- 定義關節角與馬達角的關係。
- 完成 forward()。
- 完成 inverse()，並明確說明輸入是哪個機構點。
- 定義所有可能的接觸區段或接觸點。

### 接觸模型

- 定義如何判斷目前接觸。
- 定義接觸點或接觸線的幾何。
- 定義支撐時 body displacement 如何轉成關節變化。
- 決定接觸區段切換是否需要 hysteresis 或接觸感測器。

### 擺腿模型

- 決定用 Cartesian Bezier，還是 joint-space trajectory。
- 定義起點、目標落腳點與 clearance。
- 每個軌跡點做 IK 與 joint-limit check。
- 確認觸地瞬間的速度與接觸姿態。

### 步態時序

- 定義每腳的 duty 或獨立 phase。
- 定義 swing duration 與 stance duration。
- 定義觸地失敗或超時行為。
- 定義下一步參數何時鎖存。

### 控制介面

- 明確區分 current 與 next。
- 明確區分世界座標 foothold 與 body-relative target。
- 明確區分 planner、kinematics、contact model、motor command。

## 17. 最後的概念圖

    外部輸入
    ├─ velocity
    ├─ step_length
    ├─ body pose / body displacement
    ├─ target foothold（新版本建議加入）
    └─ terrain / contact information（若要適應崎嶇地）
            ↓
    步態時序
    ├─ duty
    ├─ swing_phase
    └─ current / next per-leg parameters
            ↓
    每隻腳
    ├─ Stance
    │   └─ contact_map -> objective -> Newton solve -> move
    │                                      ↓
    │                                  theta/beta
    │
    └─ Swing
        ├─ HybridSwing：theta/beta trajectory
        └─ Bezier：foot trajectory -> inverse -> theta/beta
            ↓
    next_eta[leg] = {theta, beta}
            ↓
    馬達命令

最終濃縮：

> 用 duty 決定每隻腳何時支撐、何時擺動；用 move() 維持支撐期的輪腿接觸與滾動；用 HybridSwing 或 Bezier 規劃擺動期；最後每個控制週期產生四隻腳下一個姿態，送成 theta/beta 馬達命令。

