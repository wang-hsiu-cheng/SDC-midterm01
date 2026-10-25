#!/usr/bin/env python3
"""列出各 track 參數檔（config/localization_params_Track*.yaml）之間數值不同的參數。

三份 yaml 大多相同，改共用參數時容易漏改；用這個檢查「只剩刻意的差異」（目前只有 ply_scale）。
用法：python3 diff_params.py [yaml ...]   不給參數時比較 config/ 下所有 localization_params_Track*.yaml
只做簡單的 `key: value` 解析（不依賴 PyYAML）；註解與空行會忽略。
"""
import glob
import os
import sys


def load(path):
    values = {}
    for line in open(path, encoding='utf-8'):
        line = line.split('#', 1)[0].rstrip()
        if ':' not in line or line.lstrip().startswith(('/**', 'ros__parameters')):
            continue
        key, value = line.split(':', 1)
        values[key.strip()] = value.strip()
    return values


def main():
    config_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
    files = sys.argv[1:] or sorted(glob.glob(os.path.join(config_dir, 'localization_params_Track*.yaml')))
    if len(files) < 2:
        sys.exit('至少需要兩份 yaml')
    params = {os.path.basename(f): load(f) for f in files}
    keys = sorted(set().union(*[set(p) for p in params.values()]))
    differing = [k for k in keys if len({p.get(k, '<缺少>') for p in params.values()}) > 1]
    if not differing:
        print('所有參數檔的數值一致')
        return
    names = list(params)
    width = max(len(k) for k in differing)
    print('%-*s  %s' % (width, 'parameter', '  '.join('%-14s' % n.replace('localization_params_', '').replace('.yaml', '') for n in names)))
    for k in differing:
        print('%-*s  %s' % (width, k, '  '.join('%-14s' % params[n].get(k, '<缺少>') for n in names)))


if __name__ == '__main__':
    main()
