#!/usr/bin/env python3
"""NDT 單幀準度基準：和濾波器脫鉤，只量「scan matching 本身準不準」，用來比較 NDT 設定／前處理的變化。

用法：
  python3 ndt_bench.py run  Track1,Track2,Track3 <標籤> [--step 20] [--max-frames 600] [--bench-args "--ndt_resolution 2"]
  python3 ndt_bench.py score Track1 <標籤>         # 只重算指標（已有 matcher_bench 輸出）
  python3 ndt_bench.py table Track1                # 列出該 track 已存的所有標籤

做法：每個取樣幀（預設每 20 幀、前 600 幀、現行輸出速度 >1 m/s）建立 7 個起點（正確位置、縱向 ±1.5 m、橫向 ±1.5 m、yaw ±3°），
各跑一次 matcher_bench（guess 模式，單幀獨立對齊）。起點的位置與 yaw：Track1/2 取 GNSS（只作離線評估參考，不進演算法），
Track3 沒有 GNSS，取現行輸出位姿；z／roll／pitch 取自現行輸出。NDT 參數、尺度、距離過濾、體素預設取自該 track 的 yaml，
所以基準等同現行管線；--bench-args 可覆寫（後者蓋前者），例如 "--ndt_resolution 2 --epsilon 0.001"。

指標（越小越好）：
  準度（只有 Track1/2）：對 GNSS 的車體座標誤差（縱向、橫向、yaw）去掉各 track 的平均（GNSS 與車體參考點有固定偏移）後的 std。
    a0＝只看正確起點的結果（起點本身取自 GNSS，所以偏樂觀）；all＝7 個起點全部（含擾動起點，才反映 NDT 的拉回能力）。
  重複性：同一幀 7 個起點收斂結果的分散（縱向、橫向 std，yaw std）；收斂率＝與正確起點結果相距 <0.3 m 且 yaw <1° 的比例。
  fitness：對齊後最近點平均平方距離。
限制：GNSS 約 4 Hz、雜訊約 0.7 m，數個 cm 的差異無意義；Track3 只能看重複性與 fitness。
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import eval_ate as E  # noqa: E402
from analyze_diag import data_root  # noqa: E402

WS = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
RES = os.path.join(WS, 'src', 'localization', 'result')
STARTS = [(0.0, 0.0, 0.0), (1.5, 0.0, 0.0), (-1.5, 0.0, 0.0), (0.0, 1.5, 0.0), (0.0, -1.5, 0.0), (0.0, 0.0, 3.0), (0.0, 0.0, -3.0)]


def yaml_value(track, key, default=None):
    path = os.path.join(WS, 'src', 'localization', 'config', 'localization_params_%s.yaml' % track)
    for line in open(path):
        m = re.match(r'\s+%s:\s*([-+0-9.eE]+)' % re.escape(key), line)
        if m:
            return m.group(1)
    return default


def rot_xyz(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def pose_rpy(row):
    r = row.reshape(3, 4)[:, :3]
    return math.atan2(r[2, 1], r[2, 2]), -math.asin(max(-1.0, min(1.0, r[2, 0]))), math.atan2(r[1, 0], r[0, 0])


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def make_guess(track, step, max_frames, data):
    root = os.path.join(data_root(data), track)
    result_path = os.path.join(RES, '%s_result.txt' % track)
    P = np.loadtxt(result_path)
    diag_path = result_path + '.diag.csv'
    speed = np.genfromtxt(diag_path, delimiter=',', names=True)['speed'] if os.path.exists(diag_path) else np.full(len(P), 10.0)
    ts = E.ply_timestamps(os.path.join(root, 'raw_ply'))
    gnss = os.path.join(root, 'ref', 'gnss.csv')
    use_gnss = os.path.exists(gnss) and sum(1 for _ in open(gnss)) > 2
    if use_gnss:
        gt, gx, gy, gyaw = E.load_gnss(gnss)
        tg = (gt - gt[0]).astype(float)
        gyaw = np.unwrap(gyaw)
    last = len(P) if max_frames <= 0 else min(len(P), max_frames)
    frames, refs = [], []
    for k in range(step, last, step):
        if speed[k] <= 1.0:
            continue
        r = P[k]
        roll, pitch, yaw = pose_rpy(r)
        x, y, z = r[3], r[7], r[11]
        if use_gnss:
            t = float(ts[k] - gt[0])
            if t < tg[0] or t > tg[-1]:
                continue
            x, y, yaw = float(np.interp(t, tg, gx)), float(np.interp(t, tg, gy)), float(np.interp(t, tg, gyaw))
        frames.append(k)
        refs.append([x, y, z, roll, pitch, yaw])
    guess_path = os.path.join(RES, 'ndtb_%s_guess.txt' % track)
    with open(guess_path, 'w') as f:
        for k, (x, y, z, roll, pitch, yaw) in zip(frames, refs):
            fwd = np.array([math.cos(yaw), math.sin(yaw)]); left = np.array([-fwd[1], fwd[0]])
            for lon, lat, dy in STARTS:
                R = rot_xyz(roll, pitch, yaw + math.radians(dy))
                p = np.array([x, y]) + lon * fwd + lat * left
                v = np.concatenate([R[0], [p[0]], R[1], [p[1]], R[2], [z]])
                f.write('%d ' % k + ' '.join('%.9f' % a for a in v) + '\n')
    meta = {'track': track, 'frames': frames, 'refs': refs, 'use_gnss': use_gnss, 'starts': STARTS}
    json.dump(meta, open(os.path.join(RES, 'ndtb_%s_meta.json' % track), 'w'))
    return len(frames)


def bench_command(track, label, extra):
    scale = yaml_value(track, 'ply_scale')
    args = ['--method ndt', '--map /root/data/%s/map_downsampled_1m.pcd' % track, '--ply_dir /root/data/%s/raw_ply' % track,
            '--guess_file src/localization/result/ndtb_%s_guess.txt' % track,
            '--out src/localization/result/ndtb_%s_%s.txt' % (track, label), '--scale %s' % scale,
            '--max_range %s' % yaml_value(track, 'input_max_range', '0'), '--source_voxel %s' % yaml_value(track, 'input_voxel_leaf_size', '0.5'),
            '--ndt_resolution %s' % yaml_value(track, 'ndt_resolution', '3'), '--ndt_step %s' % yaml_value(track, 'ndt_step_size', '0.2'),
            '--max_iter %s' % yaml_value(track, 'ndt_max_iterations', '35'), '--epsilon %s' % yaml_value(track, 'ndt_transformation_epsilon', '0.01')]
    return 'source install/setup.bash && ros2 run localization matcher_bench ' + ' '.join(args) + ' ' + extra


def evaluate(track, label):
    meta = json.load(open(os.path.join(RES, 'ndtb_%s_meta.json' % track)))
    out = np.loadtxt(os.path.join(RES, 'ndtb_%s_%s.txt' % (track, label)))
    n_start = len(meta['starts'])
    frames, refs = meta['frames'], np.array(meta['refs'])
    assert len(out) == len(frames) * n_start, 'matcher_bench 輸出行數與起點檔不符'
    out = out.reshape(len(frames), n_start, -1)
    acc, acc_all, rep_lon, rep_lat, rep_yaw, conv, fit = [], [], [], [], [], [], []
    for i in range(len(frames)):
        yaw_ref = refs[i, 5]
        fwd = np.array([math.cos(yaw_ref), math.sin(yaw_ref)]); left = np.array([-fwd[1], fwd[0]])
        pos = np.stack([out[i, :, 4], out[i, :, 8]], 1)   # (n_start, 2)
        yaw = np.array([math.degrees(math.atan2(out[i, s, 5], out[i, s, 1])) for s in range(n_start)])
        yaw0 = yaw[0]
        d = pos - pos[0]
        lon, lat = d @ fwd, d @ left
        dyaw = np.array([wrap_deg(a - yaw0) for a in yaw])
        rep_lon.append(lon.std()); rep_lat.append(lat.std()); rep_yaw.append(dyaw.std())
        conv.append(np.mean((np.hypot(lon, lat) < 0.3) & (np.abs(dyaw) < 1.0)))
        fit.append(out[i, :, 13].mean())
        e = pos[0] - refs[i, :2]
        acc.append([float(e @ fwd), float(e @ left), wrap_deg(yaw0 - math.degrees(yaw_ref))])
        ea = pos - refs[i, :2]
        for s_i in range(n_start):
            acc_all.append([float(ea[s_i] @ fwd), float(ea[s_i] @ left), wrap_deg(yaw[s_i] - math.degrees(yaw_ref))])
    acc = np.array(acc)
    acc_all = np.array(acc_all)
    res = {'track': track, 'label': label, 'frames': len(frames), 'fitness': float(np.mean(fit)), 'rep_lon': float(np.sqrt(np.mean(np.square(rep_lon)))),
           'rep_lat': float(np.sqrt(np.mean(np.square(rep_lat)))), 'rep_yaw': float(np.sqrt(np.mean(np.square(rep_yaw)))), 'conv': float(np.mean(conv))}
    if meta['use_gnss']:
        mean = acc.mean(0)
        res.update({'acc_lon': float(acc[:, 0].std()), 'acc_lat': float(acc[:, 1].std()), 'acc_yaw': float(acc[:, 2].std()),
                    'bias_lon': float(mean[0]), 'bias_lat': float(mean[1]), 'bias_yaw': float(mean[2]),
                    'all_lon': float(acc_all[:, 0].std()), 'all_lat': float(acc_all[:, 1].std()), 'all_yaw': float(acc_all[:, 2].std())})
    json.dump(res, open(os.path.join(RES, 'ndtb_%s_%s.json' % (track, label)), 'w'))
    return res


HEADER = '%-8s %-14s %4s | %6s %6s %6s | %6s %6s %6s | %7s %7s %7s | %5s | %6s' % ('track', 'label', 'N', 'a0_lon', 'a0_lat', 'a0_yaw', 'all_lon', 'all_lat', 'all_yaw', 'rep_lon', 'rep_lat', 'rep_yaw', 'conv', 'fit')


def line(r):
    if 'acc_lon' in r:
        a = '%6.2f %6.2f %6.2f | %6.2f %6.2f %6.2f' % (r['acc_lon'], r['acc_lat'], r['acc_yaw'], r['all_lon'], r['all_lat'], r['all_yaw'])
    else:
        a = '%6s %6s %6s | %6s %6s %6s' % (('-',) * 6)
    return '%-8s %-14s %4d | %s | %7.2f %7.2f %7.2f | %5.2f | %6.3f' % (r['track'], r['label'], r['frames'], a, r['rep_lon'], r['rep_lat'], r['rep_yaw'], r['conv'], r['fitness'])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('action', choices=['run', 'score', 'table'])
    ap.add_argument('tracks')
    ap.add_argument('label', nargs='?', default='')
    ap.add_argument('--step', type=int, default=20)
    ap.add_argument('--max-frames', type=int, default=600)
    ap.add_argument('--bench-args', default='')
    ap.add_argument('--data')
    args = ap.parse_args()
    tracks = args.tracks.split(',')

    if args.action == 'table':
        print(HEADER)
        for t in tracks:
            for fn in sorted(os.listdir(RES)):
                if fn.startswith('ndtb_%s_' % t) and fn.endswith('.json') and not fn.endswith('_meta.json'):
                    print(line(json.load(open(os.path.join(RES, fn)))))
        return
    assert args.label, '需要標籤'
    if args.action == 'run':
        procs = []
        for t in tracks:
            n = make_guess(t, args.step, args.max_frames, args.data)
            print('%s：%d 幀 × %d 起點' % (t, n, len(STARTS)))
            env = dict(os.environ, SDC_TAG='ndtb-%s-%s' % (t, args.label))
            log = open(os.path.join(WS, 'log', 'ndtb_%s_%s.log' % (t, args.label)), 'w') if os.path.isdir(os.path.join(WS, 'log')) else subprocess.DEVNULL
            procs.append(subprocess.Popen([os.path.join(HERE, 'docker_run.sh'), bench_command(t, args.label, args.bench_args)], env=env, stdout=log, stderr=subprocess.STDOUT))
        for p in procs:
            p.wait()
    print(HEADER)
    for t in tracks:
        print(line(evaluate(t, args.label)))


if __name__ == '__main__':
    main()
