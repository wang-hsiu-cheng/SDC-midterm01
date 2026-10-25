#!/usr/bin/env python3
"""位姿 yaw 對照 IMU 內建姿態的絕對航向（不是陀螺儀積分），不需要 GNSS。

用法：python3 pose_vs_imu_attitude.py <result.txt> [<result2.txt> …] [--frames 0,100,300,…] [--data DIR]
需要 ref/imu.csv（由 tools/extract_bag.py 產生）。Track 由第一個檔名開頭（Track3_…）推得。

為什麼：陀螺儀積分有零偏與尺度誤差，長時間會漂；IMU 姿態的航向則不累積積分誤差。
若 map 與 IMU 座標之間是固定的航向偏差，則「位姿 yaw − IMU 姿態 yaw」在位姿正確時應該是常數；
它隨時間變化的部分就是位姿（或 IMU 姿態自己）的航向誤差。Track3 初始航向偏差與收斂時間就是這樣量到的。
輸出的「差」是絕對值（單位度），另印相對第 0 幀的變化。
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('results', nargs='+')
    ap.add_argument('--frames', default='0,100,300,600,800,1000,1100,1500,2000,2400,2748,3100')
    ap.add_argument('--data')
    args = ap.parse_args()

    track = re.match(r'(Track\d+)', os.path.basename(args.results[0])).group(1)
    root = os.path.join(data_root(args.data), track)
    ts = E.ply_timestamps(os.path.join(root, 'raw_ply'))
    im = np.genfromtxt(os.path.join(root, 'ref', 'imu.csv'), delimiter=',', names=True)
    x, y, z, w = im['qx'], im['qy'], im['qz'], im['qw']
    head = np.degrees(np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))

    frames = [int(k) for k in args.frames.split(',')]
    for res in args.results:
        P = np.loadtxt(res)
        yaw = np.degrees(np.unwrap(np.arctan2(P[:, 4], P[:, 0])))
        ih = np.interp(ts[:len(P)].astype(np.int64), im['stamp_ns'], head)
        diff = yaw - ih
        print(os.path.basename(res))
        print('  幀    位姿yaw  IMU姿態yaw   差(絕對)  差(相對第0幀)')
        for k in frames:
            if k < len(P):
                print('  %4d %8.1f %10.1f %9.2f %10.2f' % (k, yaw[k], ih[k], diff[k], diff[k] - diff[0]))


if __name__ == '__main__':
    main()
