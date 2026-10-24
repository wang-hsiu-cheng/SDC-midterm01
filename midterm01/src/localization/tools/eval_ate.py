#!/usr/bin/env python3
"""以 GNSS 為參考（僅離線評估，不進演算法）計算 Kaggle 指標。

E = sqrt(mean |dp_xy|^2) + 0.2 * sqrt(mean dyaw_deg^2)

用法：
  python3 eval_ate.py <result.txt> <ply_dir> <gnss.csv> [--max-frames N] [--per-frame out.csv]
result.txt 第 i 行對應 ply_dir 中依時間戳排序的第 i 幀；GNSS 以時間線性插值（yaw 用 unwrap 後插值），
GNSS 時間範圍外的幀不計分。
"""
import argparse
import glob
import math
import os
import sys

import numpy as np

K_ROT = 0.2


def ply_timestamps(ply_dir):
    files = glob.glob(os.path.join(ply_dir, '*.ply'))
    ts = sorted(int(os.path.basename(f)[:-4].split('_ts_')[1]) for f in files)
    return np.array(ts, dtype=np.int64)


def load_result(path):
    rows = np.loadtxt(path, ndmin=2)
    x, y = rows[:, 3], rows[:, 7]
    yaw = np.arctan2(rows[:, 4], rows[:, 0])
    return x, y, yaw


def load_gnss(path):
    g = np.genfromtxt(path, delimiter=',', names=True)
    yaw = 2.0 * np.arctan2(g['qz'], g['qw'])  # GNSS 只有 yaw（roll/pitch 為 0）
    return g['stamp_ns'].astype(np.int64), g['x'], g['y'], np.unwrap(yaw)


def evaluate(result, ply_dir, gnss, max_frames=0):
    x, y, yaw = load_result(result)
    ts = ply_timestamps(ply_dir)[:len(x)]
    if max_frames > 0:
        x, y, yaw, ts = x[:max_frames], y[:max_frames], yaw[:max_frames], ts[:max_frames]
    gt, gx, gy, gyaw = load_gnss(gnss)
    valid = (ts >= gt[0]) & (ts <= gt[-1])
    t = (ts - gt[0]).astype(np.float64)
    tg = (gt - gt[0]).astype(np.float64)
    rx, ry, ryaw = np.interp(t, tg, gx), np.interp(t, tg, gy), np.interp(t, tg, gyaw)
    dp = np.hypot(x - rx, y - ry)
    dyaw = np.degrees((yaw - ryaw + math.pi) % (2 * math.pi) - math.pi)
    dp, dyaw = dp[valid], dyaw[valid]
    rmse_p = math.sqrt(np.mean(dp ** 2))
    rmse_r = math.sqrt(np.mean(dyaw ** 2))
    return {
        'frames': int(valid.sum()),
        'E': rmse_p + K_ROT * rmse_r,
        'rmse_p': rmse_p,
        'rmse_yaw_deg': rmse_r,
        'p_median': float(np.median(dp)),
        'p_max': float(dp.max()),
        'p_gt_1m_ratio': float(np.mean(dp > 1.0)),
    }, (np.nonzero(valid)[0], dp, dyaw)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('result')
    parser.add_argument('ply_dir')
    parser.add_argument('gnss')
    parser.add_argument('--max-frames', type=int, default=0)
    parser.add_argument('--per-frame', help='輸出逐幀誤差 csv')
    args = parser.parse_args()
    stats, (idx, dp, dyaw) = evaluate(args.result, args.ply_dir, args.gnss, args.max_frames)
    if stats['frames'] == 0:
        sys.exit('沒有落在 GNSS 時間範圍內的幀')
    print(' '.join(f'{k}={v:.4f}' if isinstance(v, float) else f'{k}={v}' for k, v in stats.items()))
    if args.per_frame:
        np.savetxt(args.per_frame, np.column_stack([idx, dp, dyaw]), delimiter=',',
                   header='idx,dp_m,dyaw_deg', comments='', fmt=['%d', '%.4f', '%.4f'])


if __name__ == '__main__':
    main()
