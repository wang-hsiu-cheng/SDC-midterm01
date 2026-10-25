# CLAUDE.md — SDC 2026 Midterm01 Localization

將 LoGeR 從前鏡頭影像產生的 3D 點雲（`raw_ply`），對預建的 LiDAR 地圖（`map.pcd`）做 scan matching，估出每一張影像幀的車輛位姿。共 3 個 track，結果提交到 Kaggle。

| 截止 | 內容 |
|---|---|
| **10/29 09:00** | Kaggle 截止並選定最終 compliant 提交；簡報寄給 neal516.en14@nycu.edu.tw |
| 10/29 課堂 | 4–6 分鐘簡報 |
| 11/05 23:59 | E3：`<ID1>_<ID2>_localization.zip`（localization/ 原始碼、`result_1/2/3.csv`、`localization_report_<ID1>_<ID2>.pdf`、README） |

**開工前先讀 `STATUS.md`**：進度、任務、待決問題與實驗紀錄都在那裡。

---

## 1. 競賽硬規則（違反 = 該 track 0 分，TA 會重跑驗證）

1. **只能 causal**：第 k 幀的位姿只能用第 0..k 幀（與更早）的資料。禁止反向傳遞、雙向平滑、批次最佳化、看未來幀。
2. **禁止後處理**：結果寫出後不得再做任何軌跡修正或平滑（`txt_to_idx_x_y_yaw.py` 只能做格式轉換）。
3. **禁止 leaderboard probing**：不得用多次提交分數反推誤差再修正軌跡。一般調參與方法比較可以。
4. **演算法內一律不使用 GNSS**（團隊決議，三個 track 皆同；Track3 本來就沒有）。
5. **Track2 map 的 `tl`/`th` 欄位**（意外遺留的時間戳）不使用。
6. **可重現**：所有結果必須能由 repo 內的程式碼 + yaml + launch 參數 + README 重現。不得有手動編輯過的結果檔，也不得有只存在於本機的設定。
7. GNSS 可作為 Track1/2 **本地評估的參考軌跡**（不進演算法）。

不確定某作法是否合規時，先停下來問使用者。

---

## 2. 環境與指令

ROS 2 Jazzy 跑在 Docker 裡（`osrf/ros:jazzy-desktop`，container 名稱 `mid1_container`）。

| Host 路徑 | Container 路徑 |
|---|---|
| `/home/wangt/SDC-midterm01/midterm01`（colcon workspace） | `/root/midterm` |
| `/home/wangt/SDC-midterm01/data` | `/root/data` |

- git root 是 `/home/wangt/SDC-midterm01`（上一層），不是本目錄。
- `Dockerfile`、`docker-compose.yml`、`Code_LoGeR/` 都在 git root。

```bash
# 啟動 / 進入 container（在 /home/wangt/SDC-midterm01）
docker compose up -d
docker exec -it mid1_container bash

# Build（在 /root/midterm）
colcon build --packages-select localization --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 地圖預處理（產生 binary 降採樣快取；ASCII map.pcd 很大，單次載入約 1 分鐘）
# output 預設 <data_path>/map_downsampled_<leaf>m.pcd；可加 roi_min:="x,y,z" roi_max:="x,y,z" min_points_per_voxel:=N
# 不要同時跑兩個（各自載入 2.7 GB ASCII，記憶體不足時會無聲失敗）
ros2 launch localization map_modified.launch.py data_path:=/root/data/Track1 voxel_leaf_size:=1.0

# 定位（注意：bag_path 預設寫死 Track1.db3，其他 track 必須覆寫）
ros2 launch localization localization.launch.py \
    data_path:=/root/data/Track3 bag_path:=/root/data/Track3/Track3.db3 \
    save_file:=Track3_result.txt

# 轉 Kaggle 格式
python3 src/localization/processing_codes/txt_to_idx_x_y_yaw.py \
    src/localization/result/Track3_result.txt src/localization/result/result_3.csv

# 離線重播結果（pose_file 預設 result/<Track 目錄名>_result.txt；GNSS 參考軌跡僅供目視比對）
ros2 launch localization visualization.launch.py data_path:=/root/data/Track1 \
    reference_file:=/root/data/Track1/ref/gnss.csv accumulate_frames:=30 playback_rate:=2.0
# 其他參數：start_frame / end_frame / frame_step / ply_scale / frame_voxel / map_crop_margin / loop / rviz

# 離線評估（GNSS 參考，僅 Track1/2）
python3 src/localization/tools/extract_bag.py /root/data/Track1/Track1.db3 /root/data/Track1/ref
python3 src/localization/tools/eval_ate.py src/localization/result/Track1_result.txt \
    /root/data/Track1/raw_ply /root/data/Track1/ref/gnss.csv
```

launch 參數：`data_path`、`bag_path`、`ply_path`、`init_guess_file`、`save_path`、`save_file`、`params_file`、`log_level`。演算法參數放在 `config/localization_params.yaml`。

---

## 3. 目錄地圖（`src/localization/`）

| 路徑 | 職責 | 狀態 |
|---|---|---|
| `src/localization.cpp`、`include/localization/localization.h` | 主節點：I/O、主迴圈、publish | I/O 與 publish helper **保留**；scan matching 主迴圈**待重構** |
| `src/coarse_to_fine_gicp.cpp/.h` | 兩階段 G-ICP | **廢棄，待重構** |
| `src/motion_predictor.cpp/.h` | 運動先驗（主迴圈沒用到） | **廢棄，待重構** |
| `src/scale_solver.cpp` | 單幀尺度搜尋小工具 | **廢棄，待重構** |
| `src/map_modified.cpp` + `launch/map_modified.launch.py` | 地圖 ROI 裁切＋hash 體素降採樣 → binary PCD | 已重構 (10/25) |
| `src/visualization.cpp` + `launch/visualization.launch.py` + `config/visualization.rviz` | 重播 result.txt＋PLY，話題 `/visual/{map,frame,accumulated,pose,path,reference_path}` | 已重構 (10/25) |
| `include/localization/ply_sequence.h` | PLY 索引、位姿檔讀取（header-only） | 新增 |
| `src/matcher_bench.cpp` | 配準器比較基準（追蹤模式／guess 模式） | 實驗工具 |
| `tools/extract_bag.py`、`tools/eval_ate.py` | bag 匯出 IMU/GNSS、Kaggle 指標離線評估 | 新增 |
| `config/localization_params.yaml` | 演算法參數 | 隨重構改寫 |
| `processing_codes/txt_to_idx_x_y_yaw.py` | txt → Kaggle csv | 保留；移除未使用的平滑 import |
| `result/` | 輸出（gitignored） | — |

除非 `STATUS.md` 已記錄重構完成，否則現有 scan matching 程式一律視為**有缺陷**：不要拿來沿用或調參，也不要把它的輸出當參考。

---

## 4. 資料與座標約定

- 資料目錄：`/root/data/TrackN/{raw_ply/, map.pcd, init_guess.txt, TrackN.db3, metadata.yaml, TrackN_images/camera_info/}`；Track1 另有 `3cameras_ply/`。
- **PLY**：ASCII，`x y z r g b`，每幀約 15k–41k 點；檔名格式為 `NNNNNN_ts_<ns>.ply`，依時間戳排序。座標軸：**x 前、y 左、z 上**，**沒有絕對尺度**（深度約 0.2–5 個單位），必須先估計尺度。
- **map.pcd**：ASCII `x y z intensity`，約 57–60M 點、2.4–2.7 GB。**不要用 Read/cat 讀它**，只用 `head -c` 看 header。
- **init_guess.txt**：4×4 `T_map_body`，有加雜訊（Track3 的 roll/pitch 偏差明顯）。
- **bag 主題**：`/imu/data`（約 100 Hz）、三個相機 h265；`/localizer_node/gnss_pose` 只有 Track1/2 有（約 4 Hz）。
- **輸出 result.txt**：每個 PLY 幀一行，`T_map_body` 前 3 列 row-major 共 12 個數字。行數必須等於 PLY 幀數。
- **Kaggle csv**：`idx,x,y,yaw`，其中 yaw = atan2(r21, r11)。
- **評分**：`E = sqrt(mean‖Δp‖²) + 0.2·sqrt(mean Δθ²)`（Δp 單位為 m，Δθ 單位為度）。Track3 private baseline 為 1.40；public 分數只評前 5–40% 的軌跡，private 評其餘部分。

---

## 5. 工作流程守則

1. 開工先讀 `STATUS.md`，收工前更新它（勾選任務、補實驗紀錄、更新待決問題）。
2. 迭代循環：改程式 → build → 用 `max_frames` 跑子集 → 本地評估 → 把數字記進 `STATUS.md` 實驗紀錄表 → 再跑全長。
3. 參數一律放 yaml 或 launch 參數，不寫死在程式裡；同一份程式碼以參數區分不同 track。
4. 本地評估以 Track1/2 的 GNSS 為參考；Track3 只能靠 Kaggle public（每日 15 次，要省著用並記錄）。
5. 優先順序以分數權重為準：Track3（r=0.4）> Track1 ≈ Track2（r=0.3）。至少三個 track 都要贏過 baseline。
6. 資料、map 快取與結果檔不進 git（`.gitignore` 已設定）。
7. git：commit/push 前先問使用者；commit message 沿用 `feature:` / `fix:` / `modify:` 前綴。
8. 會耗時的完整 run 用背景執行，log 保留在 `log/` 或 scratchpad，不要塞進對話。
9. 報告需要「原創貢獻」：重要的設計決策與失敗嘗試都記進 `STATUS.md`，之後寫報告時使用。

---

## 6. 程式風格

- C++17，`namespace localization`，類別 `PascalCase`、函式 `PascalCase`、成員 `snake_case_`、區域變數 `snake_case`。
- 標頭放在 `include/localization/`，實作放在 `src/`；每個演算法元件是一個單一職責的 class，在 `CMakeLists.txt` 編進 `localization_core`。
- 主節點只負責流程調度、I/O 與 publish，不放演算法細節。
- 註解用繁體中文，只寫「為什麼」；不留死碼與註解掉的舊實作。
- 矩陣一律用 `Eigen::Matrix4f`（`T_target_source` 命名）；剛體反矩陣用 `RigidInverse`。
