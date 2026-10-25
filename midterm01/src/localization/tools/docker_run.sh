#!/usr/bin/env bash
# 在「一次性、互相隔離」的容器內執行指令。build、定位測試、sweep 都用它，
# 不要 `docker exec` 進 mid1_container（host 網路、Privileged、與 master 共用 build/install/result，詳見 TESTING.md）。
#
#   docker_run.sh "<容器內指令>"        例：docker_run.sh "colcon build --packages-select localization"
#
# 環境變數（皆可省略）：
#   SDC_WORKSPACE / SDC_DATA / SDC_IMAGE  見 sdc_env.sh
#   SDC_TAG   容器標籤（預設 manual）。所有由本腳本啟動的容器都帶 label sdc_run，
#             可用 `docker ps --filter label=sdc_run` 找到、`docker stop` 停掉，不會碰到 mid1_container
#   SDC_CPUS  限制每個容器可用的 CPU 數（例如 1）。並行時每幀耗時才可比；預設不限
set -euo pipefail
# shellcheck source=sdc_env.sh
source "$(dirname "${BASH_SOURCE[0]}")/sdc_env.sh"

[ $# -eq 1 ] || { sed -n '2,13p' "${BASH_SOURCE[0]}" >&2; exit 2; }
command_in_container="$1"

tag="$(printf '%s' "${SDC_TAG:-manual}" | tr -c 'A-Za-z0-9_.-' '_')"
owner="$(stat -c %u:%g "$SDC_WORKSPACE")"   # 容器以 root 執行，結束前把產物還給 workspace 的擁有者，主機端才能覆寫

args=(--rm
      --label "sdc_run=$tag"
      --name "sdc_run_${tag}_$$"
      -e HOME=/tmp
      -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # 預設 bridge 網路已和 host 的 DDS 隔開；這是額外保險
      -v "$SDC_WORKSPACE:/root/midterm"
      -v "$SDC_DATA:/root/data:ro")                # 資料唯讀：測試不可能改到資料
[ -n "${SDC_CPUS:-}" ] && args+=(--cpus "$SDC_CPUS")

exec docker run "${args[@]}" "$SDC_IMAGE" bash -c '
source /opt/ros/jazzy/setup.bash
cd /root/midterm
(eval "$0")
rc=$?
chown -R '"$owner"' build install log src/localization/result 2>/dev/null
exit $rc' "$command_in_container"
