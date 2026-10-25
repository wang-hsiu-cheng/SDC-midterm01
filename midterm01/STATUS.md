# STATUS — SDC 2026 Midterm01 Localization

> 最後更新：2026-10-25（IMU＋不鎖死 EKF 第一版，worktree `imu-ekf` / branch `worktree-imu-ekf`，尚未 commit）。每次工作結束時更新本檔（任務勾選、實驗紀錄、待決問題）。

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
- ~~`txt_to_idx_x_y_yaw.py` import 了 `savgol_filter`/`gaussian_filter1d`~~ → 10/25 已移除未使用的第三方 import（腳本現在只做格式轉換，且不依賴 pandas/scipy）
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
- [x] 移除 `txt_to_idx_x_y_yaw.py` 未使用的 pandas／numpy／scipy（含平滑函式）import（原本在無這些套件的環境無法執行）
- [x] launch 的 `bag_path` 預設改為 `<data_path>/<目錄名>.db3`

### P1 — 10/26 重構骨架＋第一版跑通
- [x] 拆除廢棄程式（coarse_to_fine_gicp / motion_predictor / scale_solver），建立 `ImuProcessor` / `MapManager` / `NdtMatcher` / `StateEstimator`（編進 `localization_core`）；主節點只剩 I/O 與流程
- [ ] 尺度實驗：固定尺度 vs 每幀估計，用 Track1 GNSS 驗證
- [~] 第一版 NDT＋EKF：Track1 全長跑通（E=3.88），Track3 尚未跑（無 GNSS，只能看 diag 與 Kaggle）
- [ ] 第一次本地評估並記錄

### P2 — 10/27 精度
- [x] IMU 融合進 StateEstimator（陀螺儀 yaw＋零偏狀態、姿態扣重力的前向加速度）
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
| 10/25 | worktree imu-ekf | T1 全長 | EKF＋IMU，單起點 NDT r2，accel_gain 1，psp 0.3 | 4.21（rmse_p 3.54, yaw 3.34°） | — | **不發散**；但對參數極敏感，見下 |
| 10/25 | 同上 | T1 全長 | 消融：accel_gain 0（只靠 NDT 修速度） | 571 | — | 車幾乎不動（v≈0），NDT 縱向拉不動，加速度計是必要的 |
| 10/25 | 同上 | T1 全長 | 單起點 NDT，改 psp 0.1／0.6、r3、lon 0.5／2.0 | 49／27／68／39／99 | — | 全部發散：縱向一旦偏 >幾 m，NDT 拉不回 |
| 10/25 | 同上 | T1 全長 | 多起點（縱向 0,±1.5,±3，取 NDT probability 最大），r2 | 577（4 組皆是） | — | 起步即被拉向「後方」候選，v 被學成≈0 後鎖死 |
| 10/25 | 同上 | T1 全長 | 多起點＋r3：psp 0.1／0.3／0.6、lon 0.5／2.0 | 3.88／3.81／107／3.75／3.79 | — | 除 psp 0.6 外穩定 |
| 10/25 | 同上（預設值） | T1 全長 | 多起點＋r3、psp 0.1（現行 yaml 預設） | **3.88**（rmse_p 3.18, yaw 3.51°） | — | 確定性可重現；前 300 幀 rmse 7.0 m（lon +4.4）、1800–2400 幀 yaw 偏 +5.6° |
| 10/25 | worktree imu-ekf | T1 全長 | 使用者新版 run：ply_scale 8.5、r3、多起點(argmax)、psp 0.1（舊 yaml 預設） | **2.25**（rmse_p 1.76, yaw 2.44°） | — | 對照 ply_scale 8.0 的 3.88：縱向系統性偏差 +2.6→−0.3 m，尺度是縱向偏差主因 |
| 10/25 | 同上 | T1 前 600 幀 | 8.5＋尺度搜尋／最大距離 30／20／FOV60 | 2.78／2.47／47.2／2.84 | — | 尺度搜尋只改善 yaw 且慢 10×；FOV 無影響（點雲本就在 ±60° 內）；20 m 發散、30 m 小幅改善 |
| 10/25 | 同上 | T1 全長 | 最大距離 30 | 1.99（rmse_p 1.57） | — | 縱向 rms 1.46→1.27 |
| 10/25 | 同上 | T1 前 600 幀 | 最大距離 30＋process_sigma_xy 0.3／1.0 | 3.20／3.24（對照 2.47） | — | 調大增益只是灌入 NDT 雜訊 |
| 10/25 | 工作區（本次 commit） | T1 全長 | 候選軟加權＋逐幀協方差＋加速度零偏狀態，r3、30 m | **1.79**（rmse_p 1.34, yaw 2.23°） | — | 見下方「縱向累積延遲」分析 |
| 10/25 | 同上 | T1 全長 | 同上，process_sigma_speed 0.3／ndt_resolution 2 | 2.09／250（發散） | — | res 2＋多起點仍會發散，resolution 固定 3 |
| 10/25 | 工作區 | T1 全長 | 一致性校準 A：R 設為實測創新雜訊（0.45/0.33/1.2°）、spread_gain 0.15 | 9.49（縱向 rms 9.06） | — | σ 更小、局部失控（縱向 9 m）；只降 R 有害 |
| 10/25 | 同上 | T1 全長 | B：A＋process_sigma_xy 0.15／yaw 0.015 | 2.00 | — | NEES 縱向 8.1×；>1.5 m 段中位 17 幀但段數 5→11 |
| 10/25 | 同上 | T1 全長 | C：A＋process_sigma_xy 0.3／yaw 0.03／speed 0.2 | 1.91 | — | NEES 縱向 5.6×、橫向 4.9×；段中位 19 幀、段數 11；仍不如 f_new（1.79） |
| 10/25 | 工作區 | T3 前 300 幀 | ply_scale 7.5／8.0／8.5／9.0／9.5（Track3，無 GNSS） | — | — | 自洽度：零偏估計 −0.009／−0.007／**−0.046**／−0.051／+0.013（真值 −0.0041，由 IMU 靜止窗口得到，僅診斷）；NIS 1.5／3.4／**4.8**／3.5／3.9；第 300 幀位置相對 7.5 差 0／3.6／**10.1**／10.1／61 m（9.5 發散） |
| 10/25 | 工作區（本次 commit） | T3 前 900 幀 | 零偏先驗 −0.004 (σ 0.003)＋ply_scale 7.0／7.25／7.5／7.75／8.0 | — | — | fitness 中位 0.232／0.224／**0.223**／0.225／0.236；第 899 幀五組最大差距僅 2.0 m、7.5 為中位；第 10 幀零偏 ≈ −0.0042（先驗生效）；之後所有尺度零偏都漂到 ≈ −0.0076（原因未明）。→ Track3 採 7.5 |

**舊版 Track1 全長 run 評估（10/25，2481 幀）**
- 分段：0–300 幀 rmse_p 2.5–3.0 m（yaw 6–7°，起步低速段）；300–1600 幀 0.8–1.7 m（縱/橫誤差 std 各約 0.9 m，平均偏差 lon +0.45 / lat +0.19 m）；1650–1850 幀 yaw 誤差先升到 9–15°、橫向 2–4 m；1875 幀起縱向線性發散（約 −0.4 m/frame），2400 幀誤差 ≈ 220 m。
- 發散機制：估計速度在 1875 後掉到 1–2 m/s，GT 仍約 12 m/s。原因鏈：①yaw 先漂（旋轉修正限幅 1.5°/幀，追不回 10°+）→ ②NDT 結果被 `divergence_*` 整幀拒絕並 coast → ③速度 EMA 只在「被採納」的幀更新，且量的是被 `max_long_correction=0.5` 截斷後的位移 → 速度被往下拉、永遠回不來 → ④縱向誤差線性累積。限幅/coast 是造成不可恢復的設計缺陷，不是調參問題。
- 估計速度本身也很吵（v_est 與 GT 差 ±2–4 m/s，例如 900 幀 7.4 vs 10.1），等速模型品質不足；IMU 尚未使用。
- 結論：舊版不可救（與 §3 判定一致），全部移除限幅/coast/速度 EMA/前進軸自學習。

**縱向累積延遲（「誤差累積到 ~1.5 m 才被拉回」）診斷（10/25，Track1 全長，GNSS 僅作本地參考；其 ~0.5 m 安裝偏移使絕對值只能比相對）**
- 診斷方法：`diag.csv` 逐幀記錄 預測／NDT 量測／濾波後位姿與 σ，離線對 GNSS 比較三者誤差、修正效率（修正量對所需修正的迴歸斜率）、|縱向|>1.5 m 連續段長度。
- 改動前（最大距離 30）：最終位姿≈預測，修正效率縱向／橫向／yaw 皆 ≈0.01；NDT 量測誤差不比預測小（lon 1.58 vs 1.28 m）；NDT 修正量扣訊號後殘餘雜訊 lon 1.1 m／lat 0.5 m／yaw 1.9°（大於 R），馬氏距離重尾（11% 幀超過閘門）。→ 量測是雜訊而非被壓縮的訊號；調高增益只會灌入雜訊。推測原因：多起點只取 probability 最大者，會在模式間翻轉。
- 改動：候選軟加權（(p/pmax)^4 × 以預測為中心的先驗）取代 argmax；量測 R＝基準 R＋候選分散度協方差（逐幀）；EKF 加前向加速度零偏 b_a。
- 結果（f_new vs 改動前）：殘餘雜訊 lon 1.18→0.41 m、lat 0.50→0.30 m、yaw 1.85→1.17°；超過閘門 11%→0%；縱向 rms 1.27→0.98 m；|縱向|>1.5 m 的幀 20%→11%、段數 12→5；E 1.99→1.79。
- **未解決**：|縱向|>1.5 m 的段持續時間中位 42／最長 180 幀（未改變），修正效率仍 ≈0.01。診斷：①濾波器過度自信（自認 σ_lon 0.27 m、σ_lat 0.09 m，實際 ~1 m）→ 幾乎不採用 NDT；②縱向似然近乎平坦（候選縱向分散中位 1.13 m、有效候選 4.2/5），NDT 單幀只能把誤差往真值拉 19%——影像點雲在縱向本質上可觀測性差，融合方式無法補救。b_a 幾乎沒動（−0.08~0.05），貢獻未做消融，不確定。
- 下一步候選：①讓 P 與實際誤差一致（過程雜訊／量測相關性），因量測雜訊已降，調大增益現在才可能有效（先前在雜訊 1.1 m 時調大是變差）；②提高縱向可觀測性（累積多幀局部點雲、顏色—強度的車道線特徵）；③處理 resolution 2 發散。

**濾波器一致性（讓 σ 與實際誤差一致）嘗試（10/25，負面結果）**
- 診斷：f_new 的 σ_lon 0.27 m／σ_lat 0.09 m，實際誤差 0.98／0.77 m（NEES 3.6×／8.3×）；NIS 中位 0.32、平均 0.69（χ²₃ 期望 2.37／3.0）→ S 被高估（R 太大）而 P 被低估（Q 太小）。預測誤差增量高度自相關（lag-1：lon 0.82／lat 0.94／yaw 0.90），隨機游走擴散約 0.32 m/√s，約為原過程雜訊的 3 倍；NDT 創新實測 std lon 0.45 m／lat 0.33 m／yaw 1.19°，小於原設定 R。
- 試驗（A/B/C，見上表）：只降 R 會更糟（σ 更小→更自信→失控）；加大 Q 能讓 NEES 靠近（橫向 8.3→4.9）並縮短 >1.5 m 段的持續時間（中位 42→17–19 幀），但段數增加、縱向 rms 與 E 變差，修正效率仍 ≈0.01，NDT 單獨能給的修正量反而更低（lon 0.19→0.06）。
- 結論：在目前量測結構下，調 Q／R 無法同時讓 σ 誠實又讓精度變好。原因是 NDT 量測誤差在時間上相關且貼著預測（黏性），沒有足夠的獨立資訊——要誠實就得更不信任它，要修正快就得多信它，兩者互相抵消。**維持 f_new 預設（E 1.79）。** 保留診斷檔的 sigma_yaw 欄位與 `StateEstimator::Sigma`。
- 因此要縮短累積延遲只能增加量測的獨立資訊（縱向可觀測性）：累積多幀局部點雲、顏色—強度的車道線特徵等。

**Track3 診斷與尺度（10/25，Track3 沒有 GNSS，只能用自洽度）**
- Track3 Kaggle public 3.98（助教 baseline 1.29，用 ply_scale 8.5）。檢驗過的推測：①初始位姿誤差大導致收斂慢——不成立（init_guess 與第 0 幀 NDT 結果只差 3.1 m／3.3°；init_guess 的 roll／pitch 14.2°／14.9° 在第 0 幀就被修成 1.1°／2.2°）；②初速 0 追不上（車一開始就約 10 m/s）——不成立（估計速度 1–2 幀內到 ~10 m/s，前 60 幀縱向落後 ≈0 m）；③後半段跟丟（估計速度 ≈0.2 m/s）——不成立（IMU 顯示第 1100–1950 幀完全靜止，加速度雜訊 0.01–0.02，80–100 s 有 ~80° 轉彎，結尾又停下）。
- Track3 的 public 只評前 5–40%（約第 157–1257 幀），含 ~40 s 靜止段，固定偏差會被重複計分。
- 無 GNSS 的判準：IMU 靜止窗口得到的真實陀螺儀零偏 −0.0041 rad/s（Track1 也是 −0.004，同一顆感測器；靜止前向加速度偏差 ≈0）。濾波器估出的零偏若被拉離這個值，代表 NDT 的 yaw 與陀螺儀矛盾（尺度不對）。此判準僅作離線診斷，演算法本身不用未來資料。
- 結論：8.5 對 Track3 不適用（零偏 −0.046、NIS／fitness 最差）；採 **ply_scale 7.5**（fitness 最低、軌跡為五組中位）。Track3 開頭只有 4 個靜止 IMU 樣本，估不了零偏，加零偏先驗（`gyro_bias_prior` −0.004、σ 0.003；Track1 靜止樣本充足，不受影響）。
- 實作：參數拆成每個 track 一份完整的 yaml（`localization_params_Track1/2/3.yaml`，原通用檔移除），launch 依 `data_path` 目錄名選用，找不到就報錯；三份目前只有 `ply_scale` 不同（Track1/2 為 8.5、Track3 為 7.5；Track2 尚未驗證，暫用 Track1 的值）。`tools/diff_params.py` 檢查三份的差異。驗證：Track3 用 7.5 的前 30 幀與 ply_scale 7.5 的 run 逐位元相同，與 8.5 差 3.6 m。選尺度的依據是自洽度而非 Kaggle 分數（規則 3）。
- 未解決：所有尺度的零偏估計在第 100 幀後都漂到 ≈ −0.0076（真值 −0.0041），加速度零偏估計 −0.03～−0.04（真值 ≈0）——與尺度無關，表示 NDT 的 yaw／縱向量測或模型仍有固定偏差，原因待查。Track3 完整 run 尚未用 7.5 重跑。

**IMU＋EKF 設計與發現（10/25）**
- 狀態 `[x, y, yaw, v, b_g, beta]`：beta＝行進方向相對車體 x 軸的偏角（取代舊的 camera_yaw_offset／前進軸自學習）。NDT 當 (x,y,yaw) 量測，縱向 σ 大於橫向；馬氏距離超過門檻只**放大 R**，不丟棄、不截斷修正量。z/roll/pitch 不進濾波器，以固定比例向 NDT 靠攏。
- IMU 資料：gyro wz 即 yaw rate（5 s 視窗對 GNSS 斜率 0.92），**零偏約 −0.004 rad/s（110 s 累積 ~28°）必須估計**；Track1 第 0 幀前有 165 個靜止樣本可初估，Track3 只有 4 個（改在線估，σ 0.02）。
- 加速度計：用 IMU 內建 orientation 扣重力後從靜止積分，100 s 內與 GNSS 速度差約 1 m/s；用加速度低通估重力方向或高通濾波都不行。**速度的唯一可靠來源是它**，NDT 的縱向約束弱。
- 發散根因（NDT 縱向收斂域窄）沒有消除：目前穩定區間很窄（psp 0.6 即發散）。多起點在 r3 有效、在 r2 反而有害，推測 NDT 縱向最佳點有系統性偏後（尺度／形狀失真）。下一步應處理縱向觀測：每幀尺度估計、前處理（去遠處點）、或用先驗加權選起點，而不是只看 probability。
- 注意：NDT probability 選起點未加先驗是第一版最脆弱的環節（已於 10/25 改為軟加權＋先驗，見上）。

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
