#!/usr/bin/env python3
"""把位姿 yaw 的變化拆成「陀螺儀積分／NDT 修正／零偏估計與參考值的差」，找出航向差（位姿 − 陀螺儀積分）是哪個來源累積的。

用法：python3 decompose_yaw.py <result.txt> [--segments 1-300,300-800,...] [--ref-bias -0.0041] [--data DIR]
需要同名的 <result.txt>.diag.csv（pred_yaw、gyro_bias、speed 欄位）。Track 由檔名開頭（Track1_…）推得。

每一段的欄位（度，區段內逐幀加總）：
  gyro   ：pred_yaw − 前一幀最終 yaw，即陀螺儀積分（扣濾波器當下估計的零偏）
  corr   ：最終 yaw − pred_yaw，即濾波器套用的 NDT 修正
  bias   ：用「估計零偏」積分相對「參考零偏」積分的差；參考零偏取自 IMU 靜止窗口（Track1/3 約 −0.0041）
  final  ：該段最終 yaw 的淨變化
位姿 − 陀螺儀參考積分的變化 = corr + bias。注意「NDT 提議量」逐幀加總沒有物理意義（每幀的提議是相對那一幀的預測），這裡不列。
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
    return (a + np.pi) % (2 * np.pi) - np.pi


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('result')
    ap.add_argument('--segments', default='1-300,300-800,800-1000,1000-1100,1100-2200,2200-end')
    ap.add_argument('--ref-bias', type=float, default=-0.0041, help='rad/s，參考零偏')
    ap.add_argument('--data')
    args = ap.parse_args()

    track = re.match(r'(Track\d+)', os.path.basename(args.result)).group(1)
    ply_dir = os.path.join(data_root(args.data), track, 'raw_ply')
    P = np.loadtxt(args.result)
    d = np.genfromtxt(args.result + '.diag.csv', delimiter=',', names=True)
    n = min(len(P), len(d))
    final = np.arctan2(P[:n, 4], P[:n, 0])
    pred = d['pred_yaw'][:n]
    bias = d['gyro_bias'][:n]
    speed = d['speed'][:n]
    ts = E.ply_timestamps(ply_dir)[:n]
    dt = np.zeros(n)
    dt[1:] = np.diff(ts) * 1e-9

    gyro = np.zeros(n)
    corr = np.zeros(n)
    gyro[1:] = wrap(pred[1:] - final[:-1])
    corr[1:] = wrap(final[1:] - pred[1:])
    bias_effect = np.zeros(n)
    bias_effect[1:] = -(bias[1:] - args.ref_bias) * dt[1:]

    deg = math.degrees
    print('幀區間         速度 |  gyro    corr |  bias   final | 累積 corr+bias（度）')
    for seg in args.segments.split(','):
        a, b = seg.split('-')
        a, b = int(a), (n if b == 'end' else min(int(b), n))
        if a >= b:
            continue
        print('%4d-%4d  %5.1f | %+6.2f %+7.2f | %+6.2f %+6.2f | %+7.2f' % (
            a, b, speed[a:b].mean(), deg(gyro[a:b].sum()), deg(corr[a:b].sum()), deg(bias_effect[a:b].sum()),
            deg(wrap(final[b - 1] - final[a - 1])), deg(corr[a:b].sum() + bias_effect[a:b].sum())))
    marks = [k for k in (10, 300, 600, 800, 900, 1000, 1100, 2000) if k < n]
    print('零偏估計 (rad/s)：' + '、'.join('第%d幀 %+.4f' % (k, bias[k]) for k in marks) + '（參考 %+.4f）' % args.ref_bias)


if __name__ == '__main__':
    main()
