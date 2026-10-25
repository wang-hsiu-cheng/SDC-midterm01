#!/usr/bin/env python3
"""診斷一次定位結果（<Track>_<標籤>.txt 與同名的 .diag.csv）：誤差從哪裡來？NDT 提出的修正有沒有被濾波器採用？

用法：python3 analyze_diag.py <result.txt> [--max-frames N] [--data DIR]
Track 由檔名開頭（Track1_…）推得；資料目錄預設為主 repo 的 data/（與 sdc_env.sh 相同）。
有資料才印的三個區塊：
  A 診斷檔本身（任何 track，含沒有 GNSS 的 Track3）
  B GNSS 參考（Track1/2，ref/gnss.csv；GNSS 只作離線評估，不進演算法）：預測／NDT 量測／最終位姿的誤差、修正效率、累積段
  C IMU yaw 一致性（有 ref/imu.csv，由 tools/extract_bag.py 產生）：位姿 yaw 的變化對照扣零偏的陀螺儀積分，
    不需要 GNSS 就能看出航向是否被拉歪（Track3 初始航向偏差就是這樣發現的）
"""
import argparse
import math
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import eval_ate as E  # noqa: E402  讀結果、PLY 時間戳、GNSS 的共用函式


def data_root(explicit):
    if explicit:
        return explicit
    common = subprocess.run(['git', 'rev-parse', '--git-common-dir'], cwd=HERE, capture_output=True, text=True,
                            check=True).stdout.strip()
    return os.path.join(os.path.dirname(os.path.abspath(os.path.join(HERE, common))), 'data')


def acf_time(x, thr=math.exp(-1)):
    x = x - x.mean()
    v = float(np.dot(x, x))
    for k in range(1, min(600, len(x) - 1)):
        if np.dot(x[:-k], x[k:]) / v < thr:
            return k
    return 600


def section_a(d, x, y):
    print('== A. 診斷檔（NDT 提議 vs 實際套用）  frames %d' % len(d))
    have_pm = all(k in d.dtype.names for k in ('pred_x', 'meas_x'))
    if have_pm:
        proposed = np.hypot(d['meas_x'] - d['pred_x'], d['meas_y'] - d['pred_y'])
        applied = np.hypot(x - d['pred_x'], y - d['pred_y'])
        print('   NDT 與預測的位置差 |meas-pred|  中位 %.3f m  p90 %.3f m  max %.2f m' % (
            np.median(proposed), np.percentile(proposed, 90), proposed.max()))
        print('   實際套用的修正 |final-pred|     中位 %.4f m  p90 %.3f m  修正 >5 cm 的幀 %.1f%%（套用比例約 %.0f%%）' % (
            np.median(applied), np.percentile(applied, 90), 100 * (applied > 0.05).mean(),
            100 * np.median(applied) / max(np.median(proposed), 1e-9)))
    nis = d['mahalanobis_sq'][5:]
    print('   NIS 中位 %.2f 平均 %.2f（χ²₃ 期望 2.37／3.0）；R 被放大的幀 %.1f%%' % (
        np.median(nis), nis.mean(), 100 * (d['inflation'][5:] > 1.001).mean()))
    print('   fitness 中位 %.3f（越低越好）；速度前 50 幀中位 %.1f、全程中位 %.1f m/s' % (
        np.median(d['fitness'][5:]), np.median(d['speed'][5:55]), np.median(d['speed'][5:])))
    if 'final_fitness' in d.dtype.names:
        ff = d['final_fitness'][5:]
        ff = ff[ff >= 0]
        seg = [(a, min(a + 500, len(d))) for a in range(0, len(d), 500)]
        print('   final_fitness（最終輸出位姿對子地圖，越低越好）中位 %.3f p90 %.3f；每 500 幀中位 %s' % (
            np.median(ff), np.percentile(ff, 90),
            ' '.join('%.3f' % np.median(d['final_fitness'][a:b][d['final_fitness'][a:b] >= 0]) for a, b in seg)))
        yi = d['yaw_minus_imu_deg']
        print('   位姿 yaw − 陀螺儀自第 0 幀積分（診斷檔，扣第 0 幀零偏）：每 500 幀末 %s deg' % (
            ' '.join('%+.1f' % yi[min(b, len(d)) - 1] for a, b in seg)))
    if 'ply_scale' in d.dtype.names:
        sc = d['ply_scale']
        seg = [(a, min(a + 500, len(d))) for a in range(0, len(d), 500)]
        print('   點雲尺度（ply_scale，線上估計時才會變）：每 500 幀末 %s；範圍 %.2f～%.2f' % (
            ' '.join('%.2f' % sc[b - 1] for a, b in seg), sc.min(), sc.max()))
    b = d['gyro_bias']
    print('   陀螺儀零偏估計 第 10 幀 %+.4f、最後 %+.4f、範圍 [%+.4f, %+.4f] rad/s（靜止窗口的真實值 Track1/3 約 −0.004）' % (
        b[min(10, len(b) - 1)], b[-1], b.min(), b.max()))
    if 'sigma_lon' in d.dtype.names:
        print('   濾波器自認 σ 中位 縱向 %.2f m 橫向 %.2f m' % (np.median(d['sigma_lon'][5:]), np.median(d['sigma_lat'][5:])))


def section_d(d, x, y, sig_thr=0.02):
    """更新離散度：NDT 每幀都提議修正，濾波器是不是只在少數幀才真的採用？（只用診斷檔，不需 GNSS）"""
    if not all(k in d.dtype.names for k in ('pred_x', 'meas_x')):
        return
    n = len(d)
    applied = np.hypot(x - d['pred_x'], y - d['pred_y'])[1:]
    proposed = np.hypot(d['meas_x'] - d['pred_x'], d['meas_y'] - d['pred_y'])[1:]
    sig = np.where(applied >= sig_thr)[0]
    if len(sig) >= 2:
        gaps = np.diff(sig)
        gap_txt = '間隔中位 %d／p90 %d／最大 %d 幀' % (np.median(gaps), np.percentile(gaps, 90), gaps.max())
    else:
        gap_txt = '顯著更新少於 2 次'
    top = np.sort(applied)[::-1][:max(1, len(applied) // 10)].sum() / max(applied.sum(), 1e-12)
    print('\n== D. 更新離散度（前 %d 幀；顯著更新 = 套用修正 ≥ %.0f cm）' % (n, sig_thr * 100))
    print('   採用比例（套用修正量總和／NDT 提議量總和）%.3f；顯著更新的幀 %.1f%%（%s）' % (
        applied.sum() / max(proposed.sum(), 1e-12), 100.0 * len(sig) / len(applied), gap_txt))
    print('   修正量集中度：最大的 10%% 幀占總修正量 %.0f%%（均勻分布約 10–20%%；越高 = 越像「少數幀才更新」）' % (100 * top))


def section_e(result, d, min_len=100):
    """靜止段（估計速度 < 0.05 m/s 的連續區段）：車停著時位姿該不動。路徑長反映逐幀抖動，淨位移反映累積漂移。"""
    P = np.loadtxt(result)[:len(d)]
    R = P[:, [0, 1, 2, 4, 5, 6, 8, 9, 10]].reshape(len(P), 3, 3)
    still = d['speed'] < 0.05
    segs, a = [], None
    for k, f in enumerate(still):
        if f and a is None:
            a = k
        if (not f or k == len(still) - 1) and a is not None:
            b = k if not f else k + 1
            if b - a >= min_len:
                segs.append((a, b))
            a = None
    print('\n== E. 靜止段（估計速度 < 0.05 m/s 且連續 ≥ %d 幀）：位姿該不動；路徑長 = 逐幀抖動，淨位移 = 累積漂移' % min_len)
    if not segs:
        print('   沒有符合的靜止段')
        return
    for a, b in segs:
        x, y = P[a:b, 3], P[a:b, 7]
        path = np.hypot(np.diff(x), np.diff(y)).sum()
        net = math.hypot(x[-1] - x[0], y[-1] - y[0])
        exc = np.hypot(x - x[0], y - y[0]).max()
        yaw = np.degrees(np.unwrap(np.arctan2(P[a:b, 4], P[a:b, 0])))
        pitch = np.degrees(-np.arcsin(np.clip(R[a:b, 2, 0], -1, 1)))
        roll = np.degrees(np.arctan2(R[a:b, 2, 1], R[a:b, 2, 2]))
        cosang = np.clip((np.trace(R[a].T @ R[b - 1]) - 1) / 2, -1, 1)
        print('   幀 %4d-%4d：路徑長 %.2f m、淨位移 %.2f m、最大偏離 %.2f m｜淨旋轉 %.2f deg、yaw 標準差 %.3f deg、roll／pitch 標準差 %.3f／%.3f deg' % (
            a, b - 1, path, net, exc, math.degrees(math.acos(cosang)), yaw.std(), roll.std(), pitch.std()))


def section_f(d, x, y, yaw, moving_speed=0.5):
    """軌跡平滑度（只看行駛中的幀；靜止段由 E 區段負責）：位置二階差（逐幀加速度）、步長的高頻抖動、yaw 逐幀變化。

    為什麼要有這一區：本地 GNSS 只有約 4 Hz、雜訊約 0.7 m，分不出逐幀的抖動，但 Kaggle 的真值精度高，抖動會直接算進誤差。
    Track1 參考值（全部幀，單位 m／度）：Kaggle 1.56 的版本 加速度中位 0.013／p90 0.034、步長殘差 0.014、yaw 步 std 0.033；
    floor 0.6 的版本（Kaggle 1.62）0.037／0.098、0.035、0.053。調參時要和 GNSS 的 E 並看，不能只看其中一個。
    """
    n = len(x)
    moving = d['speed'][:n] > moving_speed
    p = np.stack([x, y], 1)
    step = np.hypot(*np.diff(p, axis=0).T)                     # n-1
    acc = np.hypot(*np.diff(p, 2, axis=0).T)                   # n-2
    ok_step = moving[1:] & moving[:-1]
    ok_acc = moving[2:] & moving[1:-1] & moving[:-2]
    if ok_acc.sum() < 30:
        print('\n== F. 軌跡平滑度：行駛中的幀不足，略過')
        return
    kernel = np.ones(11) / 11
    resid = step - np.convolve(step, kernel, mode='same')
    ok_resid = ok_step.copy()
    ok_resid[:5] = False
    ok_resid[-5:] = False
    dyaw = np.degrees(np.diff(np.unwrap(yaw)))
    d2yaw = np.degrees(np.diff(np.unwrap(yaw), 2))
    print('\n== F. 軌跡平滑度（行駛中 %d 幀；越小越平滑）' % moving.sum())
    print('   位置加速度（二階差）中位 %.4f／p90 %.4f m/frame²｜步長高頻殘差 std %.4f m｜yaw 逐幀變化 std %.3f deg、二階差 p90 %.3f deg' % (
        np.median(acc[ok_acc]), np.percentile(acc[ok_acc], 90), resid[ok_resid].std(), dyaw[ok_step].std(),
        np.percentile(np.abs(d2yaw[ok_acc]), 90)))


def section_b(d, x, y, yaw, ts, gnss_path):
    gt, gx, gy, gyaw = E.load_gnss(gnss_path)
    t = (ts - gt[0]).astype(float)
    tg = (gt - gt[0]).astype(float)
    rx, ry, ryaw = [np.interp(t, tg, v) for v in (gx, gy, gyaw)]
    valid = (ts >= gt[0]) & (ts <= gt[-1])

    def err(px, py, pyaw):
        ex, ey = px - rx, py - ry
        return np.array([ex * np.cos(ryaw) + ey * np.sin(ryaw), -ex * np.sin(ryaw) + ey * np.cos(ryaw),
                         np.degrees((pyaw - ryaw + np.pi) % (2 * np.pi) - np.pi)])

    fin = err(x, y, yaw)
    print('\n== B. GNSS 參考（僅離線評估；GNSS 參考本身有約 0.5 m 安裝偏移與取樣雜訊，絕對值只能看相對）')
    if not all(k in d.dtype.names for k in ('pred_x', 'meas_x')):
        print('   診斷檔沒有 pred/meas 欄位，只報最終誤差')
        for i, nm in enumerate(('縱向', '橫向', 'yaw')):
            print('   %s final rms %.2f %s' % (nm, np.sqrt(np.mean(fin[i][valid] ** 2)), 'deg' if i == 2 else 'm'))
        return
    pred = err(d['pred_x'], d['pred_y'], d['pred_yaw'])
    meas = err(d['meas_x'], d['meas_y'], d['meas_yaw'])
    m = valid.copy()
    m[:2] = False
    for i, nm in enumerate(('縱向', '橫向', 'yaw')):
        u = 'deg' if i == 2 else 'm'
        need, did = -pred[i][m], fin[i][m] - pred[i][m]
        eff = np.polyfit(need, did, 1)[0]
        eff_ndt = np.polyfit(need, meas[i][m] - pred[i][m], 1)[0]
        print('   %s rms：預測 %.2f  NDT 量測 %.2f  最終 %.2f %s ｜修正效率（1＝完全修正）濾波器套用 %.2f、NDT 單獨 %.2f' % (
            nm, np.sqrt(np.mean(pred[i][m] ** 2)), np.sqrt(np.mean(meas[i][m] ** 2)), np.sqrt(np.mean(fin[i][m] ** 2)), u,
            eff, eff_ndt))
    inn = (meas - pred)[:, m]
    print('   NDT 創新（量測−預測）去相關時間 縱向 %d 橫向 %d 幀' % (acf_time(inn[0]), acf_time(inn[1])))
    lon = fin[0]
    for thr in (1.0, 1.5):
        big = (np.abs(lon) > thr) & valid
        starts = [i for i in np.where(big)[0] if i == 0 or not big[i - 1]]
        durs = []
        for s0 in starts:
            e = s0
            while e < len(big) and big[e]:
                e += 1
            durs.append(e - s0)
        if durs:
            print('   |縱向誤差|>%.1f m：%.0f%% 的幀、%d 段，持續時間中位 %d／最長 %d 幀' % (
                thr, 100 * big[valid].mean(), len(durs), np.median(durs), max(durs)))
        else:
            print('   |縱向誤差|>%.1f m：無' % thr)
    r = np.hypot(fin[0], fin[1])[valid]
    e_total = math.sqrt(np.mean(r ** 2)) + 0.2 * math.sqrt(np.mean(fin[2][valid] ** 2))
    print('   Kaggle 公式 E（GNSS 參考）= %.3f  （rmse_p %.3f、yaw %.2f deg、%d 幀）' % (
        e_total, math.sqrt(np.mean(r ** 2)), math.sqrt(np.mean(fin[2][valid] ** 2)), valid.sum()))


def section_c(yaw, ts, imu_path):
    im = np.genfromtxt(imu_path, delimiter=',', names=True)
    t = (im['stamp_ns'] - ts[0]) * 1e-9
    qx, qy, qz, qw = im['qx'], im['qy'], im['qz'], im['qw']
    up = np.stack([2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx ** 2 + qy ** 2)], 1)  # 世界上方向在 IMU 座標系
    wup = (np.stack([im['wx'], im['wy'], im['wz']], 1) * up).sum(1)
    dt = np.diff(t, prepend=t[0])
    # 零偏：取陀螺儀最安靜的 10 s 窗口（車停著）的平均；僅作離線診斷，演算法不使用未來資料
    w = 1000
    if len(wup) > 2 * w:
        csum, csq = np.cumsum(np.insert(wup, 0, 0)), np.cumsum(np.insert(wup ** 2, 0, 0))
        mean = (csum[w:] - csum[:-w]) / w
        var = (csq[w:] - csq[:-w]) / w - mean ** 2
        k = int(np.argmin(var))
        bias, std = float(mean[k]), float(math.sqrt(max(var[k], 0)))
        where = '約 %.0f–%.0f s' % (t[k], t[k + w])
    else:
        bias, std, where = float(np.median(wup)), float('nan'), '全程中位數'
    imu_yaw = np.degrees(np.cumsum((wup - bias) * dt))
    tf = (ts - ts[0]) * 1e-9
    imu_at = np.interp(tf, t, imu_yaw)
    pose = np.degrees(np.unwrap(yaw))
    diff = (pose - pose[0]) - imu_at
    print('\n== C. IMU yaw 一致性（扣零偏的陀螺儀積分，零偏 %+.4f rad/s 取自最安靜窗口 %s，std %.4f）' % (bias, where, std))
    print('   幀    t(s)  位姿yaw變化  IMU yaw變化  差(位姿−IMU)')
    for k in np.unique(np.linspace(0, len(yaw) - 1, 9).astype(int)):
        print('   %4d %6.1f  %+9.1f  %+9.1f  %+9.2f' % (k, tf[k], pose[k] - pose[0], imu_at[k], diff[k]))
    print('   差的 rms %.1f deg、最大 %.1f deg（第 %d 幀）。差持續增大 = 位姿航向與陀螺儀矛盾（初始航向偏差、尺度不對或 NDT yaw 被拉歪）' % (
        math.sqrt(np.mean(diff ** 2)), np.abs(diff).max(), int(np.abs(diff).argmax())))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('result')
    ap.add_argument('--max-frames', type=int, default=0)
    ap.add_argument('--data', help='資料目錄（含 Track1/2/3）；預設主 repo 的 data/')
    args = ap.parse_args()

    result = os.path.abspath(args.result)
    m = re.match(r'(Track\d+)', os.path.basename(result))
    if not m:
        sys.exit('檔名要以 TrackN 開頭（例如 Track1_try1.txt），才能找到對應的資料')
    track = m.group(1)
    root = os.path.join(data_root(args.data), track)
    diag_path = result + '.diag.csv'
    if not os.path.exists(diag_path):
        sys.exit('找不到診斷檔 ' + diag_path)

    x, y, yaw = E.load_result(result)
    d = np.genfromtxt(diag_path, delimiter=',', names=True, invalid_raise=False)
    n = min(len(x), len(d))
    if args.max_frames:
        n = min(n, args.max_frames)
    x, y, yaw, d = x[:n], y[:n], yaw[:n], d[:n]
    ts = E.ply_timestamps(os.path.join(root, 'raw_ply'))[:n]
    print('result: %s  (%s, %d frames)' % (os.path.basename(result), track, n))

    section_a(d, x, y)
    section_d(d, x, y)
    section_e(result, d)
    section_f(d, x, y, yaw)
    gnss = os.path.join(root, 'ref', 'gnss.csv')
    if os.path.exists(gnss) and sum(1 for _ in open(gnss)) > 2:
        section_b(d, x, y, yaw, ts, gnss)
    else:
        print('\n== B. 略過：%s 沒有可用的 GNSS 參考（Track3 本來就沒有；Track2 要先用 extract_bag.py 匯出 gnss.csv）' % track)
    imu = os.path.join(root, 'ref', 'imu.csv')
    if os.path.exists(imu):
        section_c(yaw, ts, imu)
    else:
        print('\n== C. 略過：沒有 %s；先執行 tools/extract_bag.py %s/%s.db3 %s/ref' % (imu, root, track, root))


if __name__ == '__main__':
    main()
