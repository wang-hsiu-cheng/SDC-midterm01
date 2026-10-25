#!/usr/bin/env bash
# 跑一次定位，寫出 result/<Track>_<標籤>.txt（含 .diag.csv）與 .manifest.txt。
# 輸出檔名由 Track 與標籤決定，已存在就拒絕（避免並行或前後兩次 run 互相覆寫；要覆寫請設 FORCE=1）。
#
#   run_track.sh <Track1|Track2|Track3> <標籤> [額外 ros 參數…]
#   例：run_track.sh Track1 try1 -p max_frames:=600 -p zupt_enabled:=false
#
# 套用的參數 = 該 track 的 config/localization_params_<Track>.yaml，再加上你列出的 -p 覆寫（後面的覆蓋前面的）。
# 每個 run 都在自己的一次性容器內執行（見 docker_run.sh），不發 topic、跑完即退出。
set -euo pipefail
# shellcheck source=sdc_env.sh
source "$(dirname "${BASH_SOURCE[0]}")/sdc_env.sh"

[ $# -ge 2 ] || { sed -n '2,10p' "${BASH_SOURCE[0]}" >&2; exit 2; }
track="$1"; tag="$2"; shift 2
overrides="$*"

[[ "$track" =~ ^Track[0-9]+$ ]] || { echo "Track 名稱要像 Track1: $track" >&2; exit 2; }
[[ "$tag" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "標籤只能用英數與 _ . -: $tag" >&2; exit 2; }

yaml_rel="src/localization/config/localization_params_${track}.yaml"
[ -f "$SDC_WORKSPACE/$yaml_rel" ] || { echo "找不到參數檔 $yaml_rel" >&2; exit 2; }
[ -d "$SDC_DATA/$track" ] || { echo "找不到資料 $SDC_DATA/$track" >&2; exit 2; }

result_dir="$SDC_WORKSPACE/src/localization/result"
out="${track}_${tag}.txt"
mkdir -p "$result_dir"
if [ -e "$result_dir/$out" ] && [ -z "${FORCE:-}" ]; then
    echo "$result_dir/$out 已存在；換一個標籤，或用 FORCE=1 覆寫" >&2
    exit 2
fi

# manifest：之後要能說出這個結果是由哪份程式、哪份參數、哪些覆寫產生的（規則 6：可重現）
dirty="$(git -C "$SDC_WORKSPACE" status --porcelain -- src/localization ':!src/localization/result' 2>/dev/null || true)"
{
    echo "date: $(date -Is)"
    echo "track: $track"
    echo "tag: $tag"
    echo "workspace: $SDC_WORKSPACE"
    echo "git_commit: $(git -C "$SDC_WORKSPACE" rev-parse HEAD 2>/dev/null || echo unknown)"
    if [ -n "$dirty" ]; then echo "git_dirty: yes（結果不是由單一 commit 重現，以下為未提交修改）"; echo "$dirty" | sed 's/^/  /'; else echo "git_dirty: no"; fi
    echo "params_file: $yaml_rel"
    echo "params_sha256: $(sha256sum "$SDC_WORKSPACE/$yaml_rel" | cut -d' ' -f1)"
    echo "overrides: ${overrides:-（無）}"
    echo "image: $SDC_IMAGE $(docker image inspect --format '{{.Id}}' "$SDC_IMAGE" 2>/dev/null || true)"
    echo "cpus: ${SDC_CPUS:-unlimited}"
} > "$result_dir/$out.manifest.txt"

SDC_TAG="${track}-${tag}" "$SDC_TOOLS_DIR/docker_run.sh" \
    "source install/setup.bash && ros2 run localization localization --ros-args \
 --params-file $yaml_rel \
 -p map_path:=/root/data/$track -p map_cache_file:=map_downsampled_1m.pcd \
 -p bag_path:=/root/data/$track/$track.db3 -p ply_path:=/root/data/$track/raw_ply \
 -p init_guess_file:=/root/data/$track/init_guess.txt \
 -p save_path:=/root/midterm/src/localization/result/ -p save_file:=$out \
 -p keep_alive:=false -p publish_frames:=false $overrides"
