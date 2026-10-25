# tools/ 腳本使用說明

規範與原因見 `midterm01/TESTING.md`；本檔只講「怎麼用」。以下指令都在 workspace（`midterm01/`，或某個 worktree 的 `midterm01/`）根目錄執行。腳本用自己所在的位置推算 workspace，所以在 git worktree 裡執行就自然作用在該 worktree；資料目錄會自動往主 repo 的 `data/` 找。

## 一覽

| 檔案 | 用途 | 驗證狀況（2026-10-25） |
|---|---|---|
| `docker_run.sh "<指令>"` | 在一次性、隔離的容器內執行任何指令（build、定位）。資料唯讀、bridge 網路、結束前 `chown` | build 指令實測可用 |
| `run_track.sh <Track> <標籤> [-p …]` | 套用該 track 的 yaml 跑一次定位，輸出 `result/<Track>_<標籤>.txt`、`.diag.csv`、`.manifest.txt` | **只做過語法檢查，尚未實跑**（同樣的 `docker run` 旗標以手動指令驗證過，見 TESTING.md §9） |
| `sweep.sh <Track> <標籤1> "<-p 覆寫1>" <標籤2> "<覆寫2>" …` | 並行跑多組，結束後評估（有 GNSS 才評估） | **只做過語法檢查，尚未實跑** |
| `analyze_diag.py <result.txt>` | 診斷：NDT 提議 vs 實際套用的修正、有 GNSS 時的預測／量測／最終誤差與累積段、有 IMU 時的 yaw 一致性 | 在 Track1／Track2／Track3 既有結果上實測，Track1 的數字與先前手算一致 |
| `sdc_env.sh` | 上面三支 shell 腳本共用的路徑解析（被 source，不要直接執行） | 隨 `docker_run.sh` 實測 |
| `eval_ate.py`、`extract_bag.py`、`diff_params.py` | 既有工具：GNSS 評估（Track1/2）、匯出 bag 的 IMU／GNSS、比較三份 track yaml 的差異 | — |

⚠ 尚未實跑的兩支，第一次用請先用小範圍確認，例如 `run_track.sh Track1 smoke -p max_frames:=30`，再開 sweep。

## 常用流程

```bash
T=src/localization/tools

# 1. build（一次性容器，不用進 mid1_container；sweep 進行中不要 build 同一個 workspace）
SDC_TAG=build $T/docker_run.sh "colcon build --packages-select localization --cmake-args -DCMAKE_BUILD_TYPE=Release"

# 2. 單一 run（前 600 幀）；後面的 -p 會覆蓋該 track yaml 的值
$T/run_track.sh Track1 try1 -p max_frames:=600

# 3. 並行 sweep，放背景；每組一個標籤、一組覆寫（引號內可放多個 -p）
mkdir -p log/sweeps
nohup $T/sweep.sh Track1 base "-p max_frames:=600" noz "-p max_frames:=600 -p zupt_enabled:=false" > sweep.out 2>&1 &
#    進度：讀 sweep.out 與 log/sweeps/Track1_<標籤>.log 的最後一行（每 100 幀一行）；結束時 sweep.out 會印評估

# 4. 診斷（Track3 沒有 GNSS 也能用；Track2 沒有 ref/imu.csv 時會提示先跑 extract_bag.py）
python3 $T/analyze_diag.py src/localization/result/Track1_try1.txt
python3 $T/analyze_diag.py src/localization/result/Track3_try1.txt --max-frames 900

# 5. 停掉自己的容器（不會碰到 mid1_container）
docker ps --filter label=sdc_run
docker stop <名稱或 ID>
```

## 環境變數（都可省略）

| 變數 | 預設 | 說明 |
|---|---|---|
| `SDC_WORKSPACE` | 腳本所在的 `midterm01/` | 掛成 `/root/midterm` 的 workspace |
| `SDC_DATA` | 主 repo 根目錄的 `data/` | 資料目錄（要含 `Track1/`），容器內以唯讀掛載 |
| `SDC_IMAGE` | `sdc_ros2_dev:latest` | 容器映像 |
| `SDC_TAG` | `manual` | 容器標籤，用於 `docker ps --filter label=sdc_run`；`run_track.sh` 會自動設成 `<Track>-<標籤>` |
| `SDC_CPUS` | 不限 | 每個容器的 CPU 上限（例如 `1`），並行時每幀耗時才可比 |
| `SDC_MAX_PARALLEL` | `6` | `sweep.sh` 同時跑的上限 |
| `SDC_LOG_DIR` | `<workspace>/log/sweeps` | `sweep.sh` 的 log 位置 |
| `FORCE` | 空 | 設 `FORCE=1` 才允許 `run_track.sh` 覆寫已存在的 `<Track>_<標籤>.txt` |

## 行為重點

- **輸出檔名由 Track 與標籤決定，已存在就拒絕**（避免並行或前後兩次互相覆寫）。結果在 `src/localization/result/`：`<Track>_<標籤>.txt`、`.txt.diag.csv`、`.txt.manifest.txt`。
- **manifest** 記錄日期、git commit、是否有未提交修改（及哪些檔案）、track yaml 與其 sha256、`-p` 覆寫、映像 ID、CPU 限制；引用結果時把 commit、yaml、覆寫抄進 `STATUS.md`（見 CLAUDE.md §5.13）。
- 每個 run 的差異只來自該組列出的 `-p`；其餘都吃 `config/localization_params_<Track>.yaml`。
- Track3 沒有 GNSS：`sweep.sh` 結束時不會評估，只提示改用 `analyze_diag.py`。
- `run_track.sh` 不開 rviz、不發 topic、跑完就退出；要看畫面請用 `mid1_container` 的 `ros2 launch`（單一 run）。
