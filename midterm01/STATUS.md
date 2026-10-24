# STATUS — SDC 2026 Midterm01 Localization

> 最後更新：2026-10-25。每次工作結束時更新本檔（任務勾選、實驗紀錄、待決問題）。

## 1. 截止與計分

| 項目 | 時間 |
|---|---|
| Kaggle 截止＋選定最終提交、寄出簡報 | **2026-10-29 09:00** |
| 課堂簡報（4–6 分鐘） | 2026-10-29 |
| E3 zip（程式＋報告＋result_1/2/3.csv＋README） | 2026-11-05 23:59 |

| Track | 幀數 | 權重 r | Baseline | GNSS | 備註 |
|---|---|---|---|---|---|
| Track1 | 3011 | 0.3 | 待確認（Kaggle） | 有 | 另有 3cameras_ply |
| Track2 | 3299 | 0.3 | 待確認（Kaggle） | 有 | map 含 tl/th 欄位，不使用 |
| Track3 | 3142 | 0.4 | private 1.40 | **無** | public 只評前 5–40% |

排名分數：第 1 名 56r、第 2 名 52r、第 3 名 48r、前 50% 42r、前 75% 36r、其餘高於 baseline 30r、低於 baseline 0 分。Kaggle 每日上限 15 次提交。
評分公式：`E = RMSE(Δp[m]) + 0.2 · RMSE(Δθ[deg])`。

## 2. 資料狀態

| Track | 狀態 | 內容 |
|---|---|---|
| Track1 | ✅ `/root/data/Track1` | raw_ply、3cameras_ply、map.pcd、map_downsampled_1m.pcd、init_guess、db3、images |
| Track2 | ⏸ 待 Track1 驗證程式功能正常後由使用者自行下載 | — |
| Track3 | ✅ `/root/data/Track3` | raw_ply、map.pcd、init_guess、db3（無 GNSS）、images；尚無 map 快取 |

觀察：
- PLY 軸向為 x 前、y 左、z 上，沒有絕對尺度（深度約 0.2–5 單位）。
- 幀間隔中位數 36 ms；Track3 有 11.7 ms 的近重複幀。
- init_guess 的 roll/pitch 也有雜訊（Track3 較明顯）。

## 3. 程式現況

**保留（基礎設施，重構時搬到新結構）**
- `localization.cpp`：`IndexPlyFiles`、`TimestampNsFromPlyFilename`、`LoadPlyFrame`、`LoadInitialPose`、`LoadMap`（含 cache）、`CropSubmap`、`WritePose`、`ToTransformStamped`/`ToPoseStamped`、`PublishFrame`/`PublishGlobalMap`、`RigidInverse`、`IsFinite`
- `map_modified.cpp` + launch（10/25 重構：hash 體素降採樣、ROI、稀疏體素剔除、依 leaf 自動命名）
- `visualization.cpp` + launch + `visualization.rviz`（10/25 重構：重播 result.txt＋PLY、參考軌跡、多幀疊加、倍速/區段播放）
- `include/localization/ply_sequence.h`：PLY 索引與位姿檔讀取（header-only，之後核心重構可沿用）
- `localization.launch.py`、`txt_to_idx_x_y_yaw.py`

**廢棄（有缺陷，全部重構）**
- `RunLocalization` 主迴圈：NDT 呼叫、縱/橫/垂直修正量 clamp、coast 機制、前進軸自學習、速度 EMA
- `AlignNDT`、`CameraCorrection`（pitch/yaw offset hack）、`ScalePly*` 中固定的 `ply_scale`
- `coarse_to_fine_gicp.*`、`motion_predictor.*`、`scale_solver.cpp`
- `LoadDB3Frame`/`OpenDB3File`（目前是空殼）
- yaml 中對應上述功能的參數

**已知問題**
- ~~`map_modified` 小 leaf 時 PCL VoxelGrid 溢位~~ → 10/25 已重寫為 hash 體素降採樣（1 m 結果與舊版逐點一致 496,909 點；0.5 m = 1,796,100 點）
- `localization.launch.py` 的 `bag_path` 預設寫死 `Track1.db3`
- `txt_to_idx_x_y_yaw.py` import 了 `savgol_filter`/`gaussian_filter1d`（未使用，但有違規疑慮，須移除）
- `result/` 內現有結果（Track1 98 行、Track3 31 行）都是不完整的舊 run，不具參考價值

## 4. 目標架構（重構藍圖）

依 PDF Flow：點雲處理 ＋ 地圖處理 → Scan Matching → State Update。

```
PLY ──► FramePreprocessor ──┐
                            ├─► ScanMatcher ──► StateEstimator ──► WritePose / Publish
map ──► MapManager(submap) ─┘        ▲                │
IMU ────────────────────────────────────────── predict ┘
```

| 模組 | 職責 |
|---|---|
| `FramePreprocessor` | 套用尺度、範圍/FOV 裁切、天空/離群點去除、voxel 降採樣 |
| `MapManager` | 離線產生 binary PCD 快取；以滑動 tile 快取產生子地圖，避免每幀對全圖做 CropBox |
| `ScanMatcher`（介面） | NDT / GICP 實作可切換；支援尺度估計（Sim3 或尺度搜尋）；初始化時做多假設（yaw/位置）搜尋 |
| `StateEstimator` | IMU 角速度＋等速模型預測、causal 量測更新、退化方向與發散處理（取代舊的 clamp hack） |
| `BagReader` | 從 db3 讀 IMU（以及 Track1/2 的 GNSS，用於評估），並依 PLY 時間戳對齊 |
| `LocalizationNode` | 只負責流程調度、I/O 與 publish |
| `tools/eval_ate.py` | 以 GNSS 為參考計算 Kaggle 指標（Track1/2 本地評估用） |

## 5. 任務清單

### P0 — 10/25 準備
- [x] 讀規格、盤點 workspace、寫 CLAUDE.md / STATUS.md
- [ ] （使用者）Track1 驗證完成後下載 Track2
- [x] 重構地圖降採樣 `map_modified` 與離線視覺化 `visualization`（含 launch、rviz）
- [x] 產生 map 快取：Track1 1 m / 0.5 m、Track3 1 m（Track2 待下載後產生）
- [x] bag 讀取：`tools/extract_bag.py` → `data/TrackN/ref/{imu,gnss}.csv`（Track3 無 GNSS）
- [x] `tools/eval_ate.py`：GNSS 參考軌跡＋Kaggle 公式（時間插值、yaw wrap）
- [x] 評估 G-ICP 是否取代 NDT（`matcher_bench`，見 §7）→ **結論：不採用 PCL G-ICP 取代 NDT**
- [ ] 移除 `txt_to_idx_x_y_yaw.py` 的平滑 import；修正 launch 的 `bag_path` 預設

### P1 — 10/26 重構骨架＋第一版跑通
- [ ] 拆除廢棄程式，建立 §4 的模組骨架（CMake `localization_core`）
- [ ] 尺度實驗：固定尺度 vs 每幀估計，用 Track1 GNSS 驗證
- [ ] 第一版 ScanMatcher（NDT 或 GICP）＋等速預測，三個 track 全長跑通
- [ ] 第一次本地評估並記錄

### P2 — 10/27 精度
- [ ] IMU 融合進 StateEstimator
- [ ] 第 0 幀初始化的全域/多假設搜尋（init_guess 有雜訊）
- [ ] Track3 調參（權重最高）
- [ ] 第一批 Kaggle 提交（三個 track）

### P3 — 10/28 穩健與定稿
- [ ] 發散偵測與重定位
- [ ] 三個 track 參數定稿（各自的 yaml 或 launch 參數）
- [ ] README：完整重現指令
- [ ] 簡報

### 10/29 09:00 前
- [ ] 在 Kaggle 選定三個 track 的 compliant 提交
- [ ] 寄出簡報

### 11/05 前
- [ ] 報告（Pipeline / Contributions / Problems & Solutions / Workload / Others）
- [ ] 打包 E3 zip

## 6. 待決問題

- [x] 尺度：**8.0**（使用者實測較佳）。是否隨時間漂移仍待觀察
- [x] GNSS：**三個 track 演算法內都不使用**；僅作 Track1/2 離線評估參考（2026-10-25 決議）
- [ ] 是否使用 3cameras_ply（只有 Track1/2 有）或自行用 MapAnything 產生 Track3 點雲？
- [ ] 組號（Kaggle team name）、兩位成員學號
- [ ] Track1/2 的 baseline 分數

## 7. 實驗紀錄

| 日期 | commit | Track | 設定摘要 | 本地 E（GNSS 參考） | Kaggle public | 備註 |
|---|---|---|---|---|---|---|
| 10/25 | 工作區（03654ae+未提交） | T1 前 600 幀 | 現有方案（NDT r3 + 限幅/coast），scale 8 | 3.10（rmse_p 2.12, yaw 4.92°） | — | 使用者正在跑的全長 run，取前 600 幀 |
| 10/25 | bench | T1 前 600 幀 | 純追蹤：等速預測＋配準、無限幅，NDT r3 / 1 m 地圖 | 10.66 | — | 發散 |
| 10/25 | bench | T1 前 600 幀 | 同上，NDT r2 / 0.5 m 子地圖 | 57.5 | — | 發散 |
| 10/25 | bench | T1 前 600 幀 | 同上，G-ICP / 1 m 地圖 | 92.9 | — | 600 幀中 455 幀被拒（每幀修正 2–3 m） |
| 10/25 | bench | T1 前 600 幀 | 同上，G-ICP / 0.5 m 子地圖 | 132.5 | — | 578 幀被拒 |
| 10/25 | bench guess 模式 | T1 22 幀 × 7 擾動 | 單幀配準，起點=GNSS xy/yaw（roll/pitch/z 取自現有結果）±1 m 縱/橫、±3° | NDT r2 1.22、r3 1.25、r5 2.11；G-ICP c1 1.84、c2 2.33、0.5 m c1 2.06 | — | 詳見下方分析 |

| 10/25 | 工作區舊版 | T1 全長（跑到 2481/3011 被手動停止） | 舊 NDT r3＋限幅/coast，scale 8 | 75.6（rmse_p 71.9, yaw 18.7°）；前 1700 幀 ≈ 2.4–3.1 | — | 約 1750 幀起發散，見下方分析 |

**舊版 Track1 全長 run 評估（10/25，2481 幀）**
- 分段：0–300 幀 rmse_p 2.5–3.0 m（yaw 6–7°，起步低速段）；300–1600 幀 0.8–1.7 m（縱/橫誤差 std 各約 0.9 m，平均偏差 lon +0.45 / lat +0.19 m）；1650–1850 幀 yaw 誤差先升到 9–15°、橫向 2–4 m；1875 幀起縱向線性發散（約 −0.4 m/frame），2400 幀誤差 ≈ 220 m。
- 發散機制：估計速度在 1875 後掉到 1–2 m/s，GT 仍約 12 m/s。原因鏈：①yaw 先漂（旋轉修正限幅 1.5°/幀，追不回 10°+）→ ②NDT 結果被 `divergence_*` 整幀拒絕並 coast → ③速度 EMA 只在「被採納」的幀更新，且量的是被 `max_long_correction=0.5` 截斷後的位移 → 速度被往下拉、永遠回不來 → ④縱向誤差線性累積。限幅/coast 是造成不可恢復的設計缺陷，不是調參問題。
- 估計速度本身也很吵（v_est 與 GT 差 ±2–4 m/s，例如 900 幀 7.4 vs 10.1），等速模型品質不足；IMU 尚未使用。
- 結論：舊版不可救（與 §3 判定一致），全部移除限幅/coast/速度 EMA/前進軸自學習。

**G-ICP vs NDT 分析（10/25）**
- NDT 從參考位姿出發誤差小，但幾乎不修正擾動：縱向 ±1 m 仍剩 0.7–0.97 m、橫向 +1 m 仍剩 0.99 m，yaw ±3° 只修回到約 2–2.8°。它的「準」主要來自起點本來就準，收斂域很窄。
- G-ICP 收斂到一個與起點無關的固定點（收斂域較寬），但相對 GNSS 有一致的縱向 +0.55 m 偏差與 1–1.7° 的 yaw 誤差。純追蹤時每幀跳 2–3 m，非常不穩定。
- 兩者速度相近（單執行緒約 60 ms/幀）；PCL 1.14 的 GICP 沒有多執行緒。
- 結論：瓶頸不在配準器選擇。影像點雲在縱向約束弱、形狀有失真，改進重點應放在 (1) 狀態估計（IMU yaw rate＋速度模型）與 (2) 點雲前處理（去除遠處/天空/失真點）；NDT 繼續當核心，resolution 2 略優於 3。
- 注意：GNSS 參考點與 Kaggle GT 的參考點可能有約 0.5 m 的 lever arm 差異（init_guess 相對 GNSS 為縱向 −0.49、橫向 +0.25 m），本地 E 的絕對值僅供相對比較。
- PLY body 的 roll 約 −6~−8°、pitch 約 1~6°，是持續存在的姿態，不是 init 的雜訊。

## 8. Kaggle 提交紀錄

| 日期時間 | Track | 對應實驗 | Public | 當日已用 | 備註 |
|---|---|---|---|---|---|
| | | | | | |
