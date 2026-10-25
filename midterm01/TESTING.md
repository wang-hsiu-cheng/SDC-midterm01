# TESTING.md — 定位測試、並行 sweep 與容器隔離規範

> 給所有要跑定位測試的人與 AI session。重點：**測試用一次性容器跑，不要在 `mid1_container` 裡並行或亂改；每個 run 的參數、輸出、程式版本都要說得清楚**。
> 2026-10-25 整理，下面的 `docker run` 旗標已實測（見最後一節）。
> 不想手打這些指令：`src/localization/tools/` 有對應的腳本（`docker_run.sh`、`run_track.sh`、`sweep.sh`、`analyze_diag.py`），用法與驗證狀況見 `src/localization/tools/README.md`。

## 1. 為什麼要有這份規範

- 一次 Track 全長約 3–15 分鐘（多起點 NDT 約 200 ms／幀），要比較設定就得並行；本機 8 核、15 GB。
- `mid1_container` 是 `host` 網路、`Privileged`、`DISPLAY=:0`，掛載的是 master 工作區：在裡面跑會與你的 `build/`、`install/`、`result/` 共用狀態，並行時 ROS 節點還會在同一個 DDS 網域裡互相看見（`/image_point_cloud`、`/map_pc`、`/localization_path`、`/tf` 同名）。
- 結果必須可重現（CLAUDE.md 規則 6）：要能說出每個結果檔由哪份程式、哪份參數、哪些覆寫產生。

## 2. 單一 run 的標準指令（一次性容器）

在你要測的 workspace（master 的 `midterm01/`，或某個 git worktree 的 `midterm01/`）內執行；`<WS>` 是該 workspace 的絕對路徑，`<DATA>` 是主 repo 的 `data/`（worktree 裡沒有資料，要指回主 repo）。

```bash
docker run --rm \
  --label sdc_run=<Track>-<標籤> --name sdc_run_<Track>-<標籤> \
  -e HOME=/tmp -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -v <WS>:/root/midterm -v <DATA>:/root/data:ro \
  sdc_ros2_dev:latest bash -c "
source /opt/ros/jazzy/setup.bash && cd /root/midterm && source install/setup.bash
ros2 run localization localization --ros-args \
  --params-file src/localization/config/localization_params_<Track>.yaml \
  -p map_path:=/root/data/<Track> -p map_cache_file:=map_downsampled_1m.pcd \
  -p bag_path:=/root/data/<Track>/<Track>.db3 -p ply_path:=/root/data/<Track>/raw_ply \
  -p init_guess_file:=/root/data/<Track>/init_guess.txt \
  -p save_path:=/root/midterm/src/localization/result/ -p save_file:=<Track>_<標籤>.txt \
  -p keep_alive:=false -p publish_frames:=false  <這個 run 要覆寫的 -p …> 2>&1 | tail -n 3
chown -R 1000:1000 src/localization/result"
```

- 這條指令為什麼這樣寫：
  - `--rm` 一次性容器；`--label`／`--name` 讓你找得到、停得掉自己的容器（見 §5），不會碰到 `mid1_container`。
  - 預設 bridge 網路，不加 `--network host`：和 host 的 DDS 隔開。`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 是額外保險。
  - 資料以 `:ro` 掛載：測試不可能改到資料或 map 快取（產生 map 快取要另外用 `mid1_container` 跑 `map_modified`，一次只跑一個）。
  - `keep_alive:=false publish_frames:=false`：不發 topic、跑完就退出（`ros2 launch` 會多開 rviz，測試不要用）。
  - 容器以 root 執行，結束前 `chown` 把結果還給主機使用者，否則之後主機端無法覆寫那些檔案。
- 每個 track 用**自己的** yaml（`localization_params_<Track>.yaml`）；之後的 `-p` 覆寫會蓋掉 yaml（後面的蓋前面的）。直接用 `ros2 run` 不會自動選 track yaml，所以一定要自己指定 `--params-file`。
- 要 build：同樣用一次性容器，指令換成 `colcon build --packages-select localization --cmake-args -DCMAKE_BUILD_TYPE=Release`（`/root/data` 不需要時可不掛）。

## 3. 防止參數、輸出、程式版本互相洩漏

1. **輸出檔名唯一**：`<Track>_<標籤>.txt`（診斷檔自動多一個 `.diag.csv`）。不重用 `save_file`，也不要用 launch 的預設檔名（`Track3_result.txt` 之類是「正式結果」的名字）。並行或前後兩次 run 同名會互相覆寫。
2. **同一批比較的 run，差異只能來自你列出的 `-p` 覆寫**：其他一律吃同一份 track yaml。要在同一批裡改 yaml，先確認各 run 開始前 yaml 已定稿——yaml 在 `install/` 是指向原始碼的符號連結（symlink-install），改原始碼的 yaml 會立刻影響之後啟動的 run。
3. **sweep 進行中不要 build 同一個 workspace**：新啟動的 run 會用到新 binary。要比較不同的程式版本，用各自的 git worktree（各有自己的 `build/`、`install/`），不要共用。
4. **一個 worktree 同一時間只有一個寫入者**：兩個 session 在同一目錄同時編輯過（ZUPT 與其他工作），只能靠逐段比對避免互相覆蓋。每個 session／任務請開自己的 worktree。
5. **記錄可重現資訊**：每個要引用的結果，在 `STATUS.md` 實驗紀錄寫下：commit（或「工作區未提交」）、track yaml、`-p` 覆寫清單、frames 數。有未提交修改時，結果不是由單一 commit 產生，要註明。
6. **`install/` 會留下失效的符號連結**（例如已刪除的 `localization_params.yaml`）：不影響 launch，但看到時別誤以為檔案還在。

## 4. 並行與資源

- 本機 8 核、15 GB，NDT 單執行緒：**並行 5–7 個 run 可行，建議上限 6**。`map_modified`（載入 2.7 GB ASCII）不要並行。
- 並行時每幀耗時會被拖慢（單獨約 200 ms／幀；並行 3–6 組時約 250–800 ms）。**`diag.csv` 的 `ms` 欄不能跨不同並行度比較**；需要可比時用 `docker run --cpus 1`。
- 演算法結果是**確定性**的（同設定重跑逐位元相同，已多次驗證），所以並行不影響評估數字。
- 並行寫法（把 §2 的指令各包成一行再背景執行，每組不同標籤與覆寫）：
  ```bash
  mkdir -p log/sweeps
  ( <run A 的 docker run 指令> > log/sweeps/<Track>_A.log 2>&1 ) &
  ( <run B 的 docker run 指令> > log/sweeps/<Track>_B.log 2>&1 ) &
  wait
  ```

## 5. 背景執行、監控、停止

- 會跑超過幾分鐘的 run 一律放背景（CLAUDE.md §5.8），log 放 `log/sweeps/`（`log/` 已被 colcon 忽略），不要塞進對話。
- 看進度：讀各 run log 的最後一行（節點每 100 幀印一行 `Frame N/總數 …`，結束印 `Finished: …`）。
- 找出並停止**自己的**容器：
  ```bash
  docker ps --filter label=sdc_run                  # 只列出用 §2 啟動的容器
  docker stop <容器 ID 或名稱>                      # 一個一個停；不要用 pkill -f（曾誤殺自己的 shell）
  ```
  不要停 `mid1_container`、`hw1-main-1`、`ros2_container`（不是這個專案的測試容器）。
- 中途停掉的 run 會留下**不完整的結果檔**（行數少於幀數）。不要拿它當結果，也不要 commit（STATUS 與 `result_N.csv` 對不上，規則 6）。

## 6. 評估

- Track1/2（有 GNSS 參考，僅離線評估）：`python3 src/localization/tools/eval_ate.py <result.txt> <data>/<Track>/raw_ply <data>/<Track>/ref/gnss.csv [--max-frames N]`。
- Track3 沒有 GNSS：`eval_ate.py` 會報錯（預期）。改看診斷檔 `<result>.diag.csv`：`fitness`（越低越好）、`mahalanobis_sq`（NIS）、`pred_*`／`meas_*`（預測與 NDT 量測位姿，可算「NDT 提議 vs 實際套用的修正」）、`gyro_bias`（對照 IMU 靜止窗口的真實零偏約 −0.004 rad/s）。
- 另一個不依賴 GNSS 的檢查：位姿 yaw 的變化對照 IMU 陀螺儀積分（用 `tools/extract_bag.py` 產生 `ref/imu.csv`；積分要扣零偏）。Track3 初始航向偏差約 20° 就是這樣發現的，做法與數據見 `STATUS.md`。
- Kaggle 每日 15 次：只用來比較方法與記錄，**不可用來反推誤差再修正軌跡**（規則 3）。

## 7. 什麼時候可以用 `mid1_container`

| 情況 | 用 `mid1_container`？ |
|---|---|
| 單一 run、互動除錯、跑 `map_modified`（產生 map 快取）、用 rviz／`visualization` 看結果 | 可以。但記得它是 host 網路，並與 master 工作區共用 `build/`、`install/`、`result/` |
| 並行 sweep、需要改程式的實驗、會跑很久的 run | **不要**，用 §2 的一次性容器＋自己的 worktree |
| 在 mid1 裡同時 build 與跑測試 | 不要（見 §3.3） |

## 8. 已知陷阱

- 容器內產生的檔案屬於 root：沒有 `chown` 時主機端 `cp`／覆寫會被拒（發生過）。
- 用 `launch` 預設參數時輸出檔名是固定的（如 `Track3_result.txt`），會蓋掉上一次的正式結果。
- `ros2 launch` 會開 rviz，沒有顯示器時 rviz 會死掉（不影響定位節點，但別把它當錯誤）。
- 結果檔行數必須等於 PLY 幀數；被中斷的 run 行數會較少。
- Track2 尚未驗證：目前 `localization_params_Track2.yaml` 暫同 Track1；Track2 沒有 `ref/gnss.csv`、也還沒匯出 `ref/imu.csv`。

## 9. 驗證狀況（2026-10-25）

- §2 的指令（含 `:ro` 資料、`label`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`、結束前 `chown`）在 worktree 實測：Track1 前 30 幀（`-p max_frames:=30 -p zupt_enabled:=false`）輸出與 master 已提交的 `Track1_result.txt` 前 30 行**逐位元相同**，產物擁有者為主機使用者，容器跑完自動消失，`mid1_container` 未受影響。
- 並行 5–7 組同時跑（§4）在先前的實驗批次中已用相同模式做過（一次性容器、預設 bridge、每組不同標籤與 `-p` 覆寫），當時資料目錄是可寫掛載、未設 `ROS_AUTOMATIC_DISCOVERY_RANGE`；`:ro` 與該環境變數只做過單一 run 的驗證，尚未在並行下重測。
