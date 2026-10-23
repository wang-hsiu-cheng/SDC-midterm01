#!/usr/bin/env bash
set -euo pipefail

HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PYTHON=${PYTHON:-/home/ee904/miniconda3/envs/loger/bin/python}
SOURCE=${SOURCE:-/media/ee904/Data1/Shane/dataset/Track1_images}
SOURCE_MASK=${SOURCE_MASK:-/media/ee904/Data1/Shane/dataset/ego_vehicle_mask.png}
WORK_ROOT=${WORK_ROOT:-/media/ee904/Data1/Shane/dataset/Track1_images/rectified_repro_v1}
RESULT_ROOT=${RESULT_ROOT:-/media/ee904/Data1/Shane/VGGT-SLAM/outputs/track1_halves_rectified_loger_repro_v1}
CHECKPOINT=${CHECKPOINT:-./latest.pt}
DATASET_TAG=${DATASET_TAG:-track1}
CONFIG="$HERE/vendor/loger/original_config.yaml"

for required in "$PYTHON" "$SOURCE" "$SOURCE_MASK" "$CHECKPOINT" "$CONFIG"; do
    if [[ ! -e "$required" ]]; then
        echo "ERROR: required input is missing: $required" >&2
        exit 2
    fi
done

echo "Python: $PYTHON"
PYTHONPATH="$HERE/vendor/loger:$HERE/vendor/CUT3R/src${PYTHONPATH:+:$PYTHONPATH}" \
"$PYTHON" -c 'import cv2, numpy, torch; import demo_viser; from dust3r import post_process; assert callable(post_process.estimate_focal_knowing_depth); print("Dependency preflight: PASS")'

if [[ -e "$WORK_ROOT" ]]; then
    "$PYTHON" - "$SOURCE" "$WORK_ROOT" <<'PY'
import json, sys
from pathlib import Path
source, work = map(Path, sys.argv[1:])
report = work / "report.json"
mask = work / "ego_vehicle_mask_rectified.png"
images = work / "images"
expected = len(list(source.glob("*.jpg")))
actual = len(list(images.glob("*.jpg"))) if images.is_dir() else -1
if not report.is_file() or not mask.is_file() or json.loads(report.read_text()).get("status") != "PASS" or actual != expected:
    raise SystemExit(f"ERROR: existing WORK_ROOT is incomplete ({actual}/{expected} images): {work}")
print(f"Rectification: REUSED ({actual} images)")
PY
else
    "$PYTHON" "$HERE/rectify_fisheye.py" --input-dir "$SOURCE" --ego-mask "$SOURCE_MASK" --output-root "$WORK_ROOT"
fi

"$PYTHON" - "$WORK_ROOT/images" "$WORK_ROOT/infer_halves" <<'PY'
import re, sys
from pathlib import Path
source, output = map(Path, sys.argv[1:])
files = sorted(source.iterdir(), key=lambda p: int(re.search(r'_ts_(\d+)', p.name).group(1)))
cut = len(files) // 2
for name, group in (("part_1", files[:cut]), ("part_2", files[cut:])):
    folder = output / name; folder.mkdir(parents=True, exist_ok=True)
    for path in group:
        link = folder / path.name
        if link.is_symlink() and link.resolve() == path.resolve(): continue
        if link.exists() or link.is_symlink(): raise SystemExit(f"ERROR: conflicting split entry: {link}")
        link.symlink_to(path)
PY

for part in 1 2; do
for window in 32; do
run="$RESULT_ROOT/w${window}/part_${part}"
mkdir -p "$RESULT_ROOT"
cache="$RESULT_ROOT/${DATASET_TAG}_part${part}_w${window}_o1.pt"
if [[ -f "$cache" ]]; then
echo "Inference cache: REUSED $cache"
else
(
cd "$HERE/vendor/loger"
"$PYTHON" demo_viser.py \
--input "$WORK_ROOT/infer_halves/part_${part}" \
--output_folder "$RESULT_ROOT" \
--output "${DATASET_TAG}_part${part}_w${window}_o1" \
--seq_name "${DATASET_TAG}_part${part}" \
--model_name "$CHECKPOINT" \
--config "$CONFIG" \
--window_size "$window" \
--overlap_size 1 \
--keep_overlap \
--intrinsics_conf_min 0 \
--skip_viser
)
fi
export_root="$run"
if [[ -f "$export_root/report.json" ]]; then
echo "Frame-own export: REUSED $export_root"
elif [[ -e "$export_root" ]]; then
echo "ERROR: incomplete export root: $export_root" >&2
exit 2
else
"$PYTHON" "$HERE/export_frame_own.py" \
--prediction-cache "$cache" \
--image-root "$WORK_ROOT/infer_halves/part_${part}" \
--ego-mask "$WORK_ROOT/ego_vehicle_mask_rectified.png" \
--output-root "$export_root" \
--flat-output \
--confidence-min 0.18 \
--voxel-size 0.01
fi
done
done
