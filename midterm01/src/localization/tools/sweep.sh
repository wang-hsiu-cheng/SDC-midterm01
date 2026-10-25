#!/usr/bin/env bash
# 並行跑同一個 track 的多組設定（每組一個隔離容器），全部結束後統一評估。
#
#   sweep.sh <Track> <標籤1> "<覆寫參數1>" <標籤2> "<覆寫參數2>" …
#   例：sweep.sh Track1 base "-p max_frames:=600" noz "-p max_frames:=600 -p zupt_enabled:=false"
#
# 背景執行（建議，整批常要 5–15 分鐘）：
#   nohup src/localization/tools/sweep.sh Track1 … > sweep.out 2>&1 &
# 之後讀 sweep.out 與 log/sweeps/<Track>_<標籤>.log 的最後一行看進度（節點每 100 幀印一行）。
#
# 環境變數：SDC_MAX_PARALLEL（預設 6，本機 8 核 / 15 GB）、SDC_LOG_DIR（預設 <workspace>/log/sweeps）、SDC_CPUS（見 docker_run.sh）
# 注意：sweep 進行中不要在同一個 workspace build（新啟動的 run 會用到新 binary）；比較不同程式版本請用各自的 worktree。
set -uo pipefail
# shellcheck source=sdc_env.sh
source "$(dirname "${BASH_SOURCE[0]}")/sdc_env.sh"

[ $# -ge 3 ] && [ $(( ($# - 1) % 2 )) -eq 0 ] || { sed -n '2,9p' "${BASH_SOURCE[0]}" >&2; exit 2; }
track="$1"; shift
n=$(( $# / 2 ))
max="${SDC_MAX_PARALLEL:-6}"
if [ "$n" -gt "$max" ]; then
    echo "要並行 $n 組，超過上限 $max（SDC_MAX_PARALLEL）；分批跑，或確認資源夠再調高" >&2
    exit 2
fi

log_dir="${SDC_LOG_DIR:-$SDC_WORKSPACE/log/sweeps}"
mkdir -p "$log_dir"
result_dir="$SDC_WORKSPACE/src/localization/result"

tags=(); pids=()
while [ $# -gt 0 ]; do
    tag="$1"; args="$2"; shift 2
    tags+=("$tag")
    # $args 刻意不加引號：讓它依空白拆成多個 ros 參數
    # shellcheck disable=SC2086
    "$SDC_TOOLS_DIR/run_track.sh" "$track" "$tag" $args > "$log_dir/${track}_${tag}.log" 2>&1 &
    pids+=($!)
done
echo "started ${#tags[@]} runs on $track: ${tags[*]}   (logs: $log_dir)"

fail=0
for i in "${!tags[@]}"; do
    if ! wait "${pids[$i]}"; then
        fail=1
        echo "!! ${tags[$i]} 失敗（exit code 非 0），log 最後幾行："
        tail -n 5 "$log_dir/${track}_${tags[$i]}.log" | sed 's/^/   /'
    fi
done

echo "=== 評估 ==="
gnss="$SDC_DATA/$track/ref/gnss.csv"
for tag in "${tags[@]}"; do
    result="$result_dir/${track}_${tag}.txt"
    printf '%s: ' "$tag"
    if [ ! -s "$result" ]; then
        echo "沒有結果檔"
    elif [ -s "$gnss" ] && [ "$(wc -l < "$gnss")" -gt 2 ]; then
        python3 "$SDC_TOOLS_DIR/eval_ate.py" "$result" "$SDC_DATA/$track/raw_ply" "$gnss" 2>&1 | tail -n 1
    else
        echo "$(wc -l < "$result") 行；$track 沒有 GNSS 參考，改用 tools/analyze_diag.py（fitness、NIS、IMU yaw 一致性）"
    fi
done
exit $fail
