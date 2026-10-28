import re
T = '/home/wangt/.claude/jobs/53600edd/tmp/'
M = {
    # figs.py
    r'舊版': 'Old', r'尺度 8.5\n無 ZUPT': r'scale 8.5\nno ZUPT', r'尺度 7.5\n零偏先驗\n＋ZUPT': r'scale 7.5\nbias prior\n+ZUPT',
    r'＋初始位姿\n搜尋＋靜止\n凍結 yaw': r'+init-pose\nsearch, +yaw\nfreeze at stop', r'＋hold\n＋floor 0.6': r'+hold\n+floor 0.6',
    r'H2 去掉\nfloor': r'H2 minus\nfloor', r'＋ba 雜訊\n0.002': r'+ba noise\n0.002', r'＋軌跡線\n航向': r'+trail\nheading',
    r'＋σ_yaw 4\n＋軌跡線 σ1.5°\n每 3 幀': r'+σ_yaw 4\n+trail σ1.5°\nevery 3 frames', r'現行 p5': 'current p5',
    r'Kaggle public（越低越好）': 'Kaggle public (lower is better)',
    r'Track3 分數演進（實心＝沿主線採用；空心＝旁支試驗，未採用）': 'Track3 score history (filled = adopted on the main line; hollow = side experiments, not adopted)',
    r'無 GNSS': 'No GNSS', r'只用 xy': 'xy only', r'只用 yaw': 'yaw only', r'xy＋yaw\n（現行）': r'xy + yaw\n(current)',
    r'−%.2f（%d%%）': '−%.2f (%d%%)',
    r'GNSS 先驗的貢獻（官方文件允許使用 GNSS；σxy 0.7 m、σyaw 2°）': 'Contribution of the GNSS prior (σxy 0.7 m, σyaw 2°)\nGNSS use is allowed by the official documents',
    r'zb0（無軌跡線）': 'zb0 (no trail)', r'bt1（軌跡線）': 'bt1 (trail)', r'p5（現行）': 'p5 (current)',
    r'輸出 yaw − IMU 姿態 yaw\n（相對第 0 幀，度；＋＝逆時針）': r'Output yaw − IMU attitude yaw\n(relative to frame 0, deg; + = CCW)',
    r'700–1000 幀\n殘差形成': r'Frames 700–1000\nresidual forms', r'左側 %d': 'Left %d', r'右側 %d': 'Right %d',
    r'前方 0–25 m 的地圖結構點數\n（離地 >1.2 m）': r'Map structure points\nahead (0–25 m, >1.2 m high)',
    r'幀': 'Frame',
    r'Track3 +5° 航向殘差：形成於 700–1000 幀（接近並停在紅燈），左側結構點數在 700–750 幀升高': 'Track3 +5° yaw residual forms in frames 700–1000 (approach and stop at the red light);\nleft-side structure count rises at frames 700–750',
    r'行駛中': 'Driving', r'停等（紅燈）': 'Stopped (red light)', r'尺度 %.1f': 'scale %.1f', r'%s：第 %d 幀': '%s: frame %d',
    r'距離誤差 − 該曲線最小值': 'Error − curve min.', r'縱向偏移（m，＋＝往前）': 'Longitudinal offset (m, + = forward)',
    r'縱向 fitness 曲線（固定 p5 位姿、不優化；圓點＝曲線最小值，虛線＝p5 位置）。每條曲線減去自身最小值，跨尺度的絕對值不可比': 'Longitudinal fitness curves (fixed p5 pose, no optimisation; dot = curve minimum, dashed = p5 position)\nEach curve has its own minimum subtracted; absolute values are not comparable across scales',
    r'地圖：相機高度（z ±0.25 m）的點': 'Map: points at camera height (z ±0.25 m)', r'p5 輸出軌跡（0–1300 幀）': 'p5 output trajectory (frames 0–1300)',
    r'沿行駛路徑的距離（m）': 'Distance along the driving path (m)', r'橫向偏移（m；左為正）': 'Lateral offset (m; left positive)',
    r'地圖相機高度的點：沿建圖車軌跡的細點線': 'Camera-height map points: thin line along the mapping car path', r'紅燈停等': 'Red-light stop',
    r'軌跡線航向量測 − 預測（度）': 'Trail heading measurement − prediction (deg)', r'每次修正的航向量（套用 %d 次）': 'Heading correction per update (%d applied)',
    r'停等不套用': 'not applied while stopped',
    # figs2.py
    r'LoGeR 點雲\nraw_ply（無尺度）': r'LoGeR point cloud\nraw_ply (no scale)', r'LiDAR 地圖\nmap.pcd': r'LiDAR map\nmap.pcd', r'IMU（約 100 Hz）': 'IMU (~100 Hz)',
    r'前處理\n× ply_scale、30 m、\n體素 0.5 m': r'Preprocess\n× ply_scale, 30 m,\n0.5 m voxel', r'NDT 多起點\n縱向 0、±1.5、±3 m': r'Multi-start NDT\nlongitudinal offsets\n0, ±1.5, ±3 m',
    r'軟加權融合\n分散度 → 量測協方差': r'Soft-weighted fusion:\nspread →\nmeasurement cov.', r'IMU 預測\n＋ZUPT（靜止釘速度）': r'IMU prediction\n+ ZUPT (zero speed\nat stops)',
    r'7 狀態 EKF\nx、y、yaw、v、\n陀螺零偏、β、ba': r'7-state EKF\nx, y, yaw, v,\ngyro bias, β, ba', r'位姿': 'Pose',
    r'T1/T2：GNSS 先驗\nxy＋yaw（σ 0.7 m／2°）': r'T1/T2: GNSS prior\nxy + yaw (σ 0.7 m / 2°)',
    r'T2：牆面方向先驗、\nYawBank（延後 yaw 承諾）': r'T2: wall-direction prior\n+ YawBank (deferred\nyaw commitment)',
    r'T3：軌跡線航向量測\n（地圖細點線，σ 1.5°、每 3 幀）': r'T3: trail heading\nmeasurement (thin map line,\nσ 1.5°, every 3 frames)',
    r'第 0 幀初始位姿搜尋\n（T1 yaw、T3 yaw＋xy）': r'Frame-0 initial pose search\n(T1 yaw, T3 yaw + xy)',
    r'同一套管線，三個 track 以 yaml 開關各自的外掛模組（虛線＝只在該 track 啟用）': 'One pipeline; each track enables its own add-on modules via yaml (dashed = enabled on that track only)',
    r'KISS-ICP 式': 'KISS-ICP-style', r'Track1 全長對 GNSS 的本地 E（越低越好；對數軸）': 'Track1 full-length local E vs GNSS (lower is better; log axis)',
    r'① 乾淨基準：純追蹤，無 IMU／濾波': '(1) Clean baseline: pure tracking, no IMU / filter',
    r'NDT（gb／g2b／bt1）': 'NDT (gb / g2b / bt1)', r'G-ICP（S4／S4／S3）': 'G-ICP (S4 / S4 / S3)',
    r'② 套上完整管線（多起點＋軟加權＋EKF）後': '(2) With the full pipeline (multi-start + soft weighting + EKF)',
    r'G-ICP 裸配準最準，但套上完整管線後 NDT 在三個 track 都較好': 'Bare G-ICP is the most accurate, but NDT wins on all three tracks once the full pipeline is added',
    r'左圖只有 Track1。右圖 NDT 的 Track3 對照版是 bt1（1.87），現行 p5 為 1.74。': 'Left: Track1 only. Right: the NDT reference for Track3 is bt1 (1.87); current p5 is 1.74.',
    r'地圖相機高度的點': 'Camera-height map points', r'p5 輸出軌跡': 'p5 output trajectory',
    r'地圖中沿建圖車軌跡的細點線 → 航向量測': 'Thin trail line in the map → heading measurement',
    r'ZUPT\n尺度 7.5': r'ZUPT\nscale 7.5', r'初始位姿': 'init pose', r'軌跡線': 'trail', r'Track3 分數演進': 'Track3 score history',
    r'輸出 yaw − IMU 姿態 yaw（度，＋＝逆時針）': 'Output yaw − IMU attitude yaw (deg, + = CCW)',
    r'yaw：+5° 殘差形成於接近並停在紅燈的 700–1000 幀': 'yaw: +5° residual forms in frames 700–1000',
    r'縱向（行駛中，第 500 幀）：最小值隨尺度移動': 'Longitudinal, driving (frame 500)',
    r'縱向（停等，第 1100 幀）：最小值在 −0.5～0 m': 'Longitudinal, stopped (frame 1100)',
    r'縱向偏移（m，＋＝往前；虛線＝p5 位置）': 'Longitudinal offset (m, + = forward; dashed = p5 position)',
    r'橫向與 yaw 有結構、可修；縱向退化，最佳位置隨尺度移動，停等時偏差 ≤0.5 m': 'Lateral and yaw errors are structured and fixable;\nlongitudinal is degenerate: its best position moves with scale (≤0.5 m offset while stopped)',
    # figs3.py
    r'紅燈停等（約 948–2179 幀）': 'Red-light stop (frames ≈ 948–2179)', r'起步與轉彎': 'Start-up and turn', r'ply_scale 7.5（採用）': 'ply_scale 7.5 (adopted)',
    r'行駛中（速度 >0.5 m/s）': 'Driving (speed > 0.5 m/s)', r'停等': 'Stopped', r'落在掃描範圍邊界（6.5 或 9.5）': 'At sweep-range boundary (6.5 or 9.5)',
    r'單幀 NDT 最佳尺度（fitness 最小）': 'Best single-frame NDT scale (lowest fitness)',
    r'行駛段最佳尺度約 7.5；停車期間升到 8.3–9.1（約 +20%）；轉彎段散在 6.5–9.2': 'Best scale ≈ 7.5 while driving; rises to 8.3–9.1 (≈ +20%) while stopped;\nscatters over 6.5–9.2 in the start-up and turn',
    r'停等時單幀最佳尺度會與平移耦合，只看趨勢': 'While stopped the single-frame best scale couples with translation; read the trend only',
    r'關閉 ZUPT': 'ZUPT off', r'開啟 ZUPT（現行 p5）': 'ZUPT on (current p5)', r'靜止窗口估得的零偏 −0.0041': 'Bias estimated in the stationary window: −0.0041',
    r'位移（m）': 'Displacement (m)', r'yaw 變化（度）': 'Yaw change (deg)', r'陀螺零偏（rad/s）': 'Gyro bias (rad/s)',
    r'Track3 紅燈停等（948–2179 幀；位移、yaw 皆相對停等起點）：關 ZUPT 時位置與 yaw 持續漂移、零偏估計被拉走': 'Track3 red-light stop (frames 948–2179; displacement and yaw relative to the start of the stop)\nWithout ZUPT, position and yaw drift and the bias estimate is pulled away',
    r'承諾 %g°（fitness 最小）\n＝不改變濾波器 yaw': r'Commits %g° (lowest fitness)\n= filter yaw unchanged',
    r'起步時濾波器 yaw 的偏移（度）': 'Yaw offset of the filter at start (deg)', r'平均 NDT fitness（越低越好）': 'Mean NDT fitness (lower is better)',
    r'9 個 yaw 假設各跑一份濾波器\n（%d–%d 幀，共 %d 幀後承諾）': r'9 yaw hypotheses, one filter each\n(frames %d–%d; commit after %d frames)',
    r'YawBank 開（rhp2）': 'YawBank on (rhp2)', r'YawBank 關': 'YawBank off', r'%s：E %.2f': '%s: E %.2f',
    r'位置誤差（m）': 'Position error (m)', r'yaw 誤差（度）': 'Yaw error (deg)',
    r'對 GNSS 的誤差，放大 0–800 幀（僅離線評估；兩個 run 都關閉 GNSS 先驗；承諾後兩條完全重合；E 為全長）': 'Error vs GNSS, zoomed to frames 0–800 (offline evaluation only; GNSS prior off in both runs;\ncurves coincide after commitment; E is full-length)',
}
keys = sorted(M, key=len, reverse=True)
for name, new in (('figs.py', 'figs_en.py'), ('figs2.py', 'figs2_en.py'), ('figs3.py', 'figs3_en.py')):
    s = open(T + name).read()
    for k in keys:
        s = s.replace("'" + k + "'", "'" + M[k].replace("'", "\\'").replace(chr(10), chr(92) + 'n') + "'")
    s = s.replace("import figs as F", "import figs_en as F").replace("from figs import", "from figs_en import")
    s = s.replace("'slides_figs/'", "'slides_figs_en/'")
    s = s.replace("['DejaVu Sans', 'Droid Sans Fallback']", "['DejaVu Sans']")
    open(T + new, 'w').write(s)
    left = sorted(set(re.findall(r"'[^'\n]*[一-鿿][^'\n]*'", s)))
    print(new, 'remaining CJK literals:', len(left))
    for l in left:
        print('   ', l[:110])
