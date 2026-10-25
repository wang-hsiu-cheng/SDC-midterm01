#!/usr/bin/env bash
# 測試腳本共用的路徑解析（被 docker_run.sh / run_track.sh / sweep.sh 以 source 引用，不要直接執行）。
#
# SDC_WORKSPACE  要掛成 /root/midterm 的 colcon workspace。預設是本腳本所在的 midterm01/；
#                在 git worktree 裡執行時自然就是該 worktree 的 midterm01/，不會碰到別的檢出。
# SDC_DATA       資料目錄（含 Track1/2/3）。預設是「主 repo」根目錄下的 data/（worktree 沒有資料，要往主 repo 找）。
# SDC_IMAGE      容器映像，預設 sdc_ros2_dev:latest。

SDC_TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDC_WORKSPACE="${SDC_WORKSPACE:-$(cd "$SDC_TOOLS_DIR/../../.." && pwd)}"
SDC_IMAGE="${SDC_IMAGE:-sdc_ros2_dev:latest}"

if [ -z "${SDC_DATA:-}" ]; then
    # git-common-dir 在 worktree 裡指向「主 repo」的 .git，它的上一層就是主 repo 根目錄
    _common="$(cd "$SDC_WORKSPACE" && cd "$(git rev-parse --git-common-dir)" && pwd)"
    SDC_DATA="$(dirname "$_common")/data"
fi

if [ ! -d "$SDC_WORKSPACE/src/localization" ]; then
    echo "SDC_WORKSPACE 不是 colcon workspace（找不到 src/localization）: $SDC_WORKSPACE" >&2
    return 1 2>/dev/null || exit 1
fi
if [ ! -d "$SDC_DATA/Track1" ]; then
    echo "找不到資料目錄（需要含 Track1/）: $SDC_DATA；可用 SDC_DATA=<路徑> 指定" >&2
    return 1 2>/dev/null || exit 1
fi
export SDC_TOOLS_DIR SDC_WORKSPACE SDC_DATA SDC_IMAGE
