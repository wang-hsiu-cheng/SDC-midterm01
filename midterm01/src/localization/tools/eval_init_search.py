#!/usr/bin/env python3
"""評估第 0 幀初始位姿搜尋：probability／fitness 對收斂後 yaw 的曲線，以及不同「挑最佳起點」準則選出的 yaw。

用法：python3 eval_init_search.py <TrackN_標籤.txt.initsearch.csv> [--data DIR]
輸入由 `-p max_frames:=1 -p init_search_dump:=true` 的定位 run 產生。Track 由檔名開頭推得。
有 GNSS 參考（Track1/2 的 ref/gnss.csv）時，另印各準則選出的 yaw 對 GNSS 航向的誤差（GNSS 只作離線評估）。

為什麼：直行路段的 yaw 景觀很平，取 probability 最大者不一定可靠；Track3 的 probability 是 156°～169° 的寬平台，
Track1 的平台更寬；fitness 的相對變化比 probability 明顯。限制：GNSS 航向只有一個資料點，且搜尋視窗以 init／IMU 為中心，
「平台加權中心」會近似視窗中心，不是獨立證據。
"""
import argparse
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import eval_ate as E  # noqa: E402
from analyze_diag import data_root  # noqa: E402


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('initsearch')
    ap.add_argument('--data')
    args = ap.parse_args()

    track = re.match(r'(Track\d+)', os.path.basename(args.initsearch)).group(1)
    root = os.path.join(data_root(args.data), track)
    d = np.genfromtxt(args.initsearch, delimiter=',', names=True)
    yaw, p, f, x, y = d['yaw_deg'], d['probability'], d['fitness'], d['x'], d['y']
    print('%s：%d 個起點' % (os.path.basename(args.initsearch), len(d)))

    ref_yaw = None
    gnss = os.path.join(root, 'ref', 'gnss.csv')
    if os.path.exists(gnss) and sum(1 for _ in open(gnss)) > 2:
        ts = E.ply_timestamps(os.path.join(root, 'raw_ply'))
        gt, gx, gy, gyaw = E.load_gnss(gnss)
        ref_yaw = math.degrees(np.interp(ts[0] - gt[0], gt - gt[0], np.unwrap(gyaw)))
        print('第 0 幀 GNSS 航向 %.1f°（與車體 x 軸可能有安裝偏差，只看相對）' % ref_yaw)

    def report(name, yaw_value, i=None):
        err = '' if ref_yaw is None else '（對 GNSS %+.1f°）' % wrap(yaw_value - ref_yaw)
        extra = '' if i is None else '  prob %.3f fit %.3f' % (p[i], f[i])
        print('  %-30s yaw %7.1f° %s%s' % (name, yaw_value, err, extra))

    print('\n各準則選出的 yaw：')
    i = int(np.argmax(p))
    report('probability 最大（現行預設）', yaw[i], i)
    i = int(np.argmin(f))
    report('fitness 最小', yaw[i], i)
    idx = np.where(p >= 0.95 * p.max())[0]
    j = idx[int(np.argmin(f[idx]))]
    report('平台內 fitness 最小', yaw[j], j)
    ang = np.radians(yaw[idx])
    wgt = p[idx] ** 4
    center = math.degrees(math.atan2((wgt * np.sin(ang)).sum(), (wgt * np.cos(ang)).sum()))
    report('平台加權中心（%d 個起點）' % len(idx), center)

    print('\n收斂後 yaw 每 2° 的最大 probability／最小 fitness（只列 probability ≥ 80%% 最大值，最大值 %.3f）：' % p.max())
    for a in np.arange(math.floor(yaw.min()), yaw.max() + 2, 2):
        m = (yaw >= a) & (yaw < a + 2)
        if m.any() and p[m].max() >= 0.8 * p.max():
            print('  [%6.1f,%6.1f) n=%3d  prob %.3f  fit %.3f' % (a, a + 2, m.sum(), p[m].max(), f[m].min()))


if __name__ == '__main__':
    main()
