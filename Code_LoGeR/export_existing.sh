#!/usr/bin/env bash
set -euo pipefail

HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PYTHON=${PYTHON:-/home/ee904/miniconda3/envs/loger/bin/python}
RESULT_ROOT=${RESULT_ROOT:-/media/ee904/Data1/Shane/VGGT-SLAM/outputs/track2_halves_rectified_loger_v1}
IMAGE_ROOT=${IMAGE_ROOT:-/media/ee904/Data1/Shane/dataset/Track2_images/rectified_v1/infer_halves}
EGO_MASK=${EGO_MASK:-/media/ee904/Data1/Shane/dataset/Track2_images/rectified_v1/ego_vehicle_mask_rectified.png}
EXPORT_NAME=${EXPORT_NAME:-frame_own_portable_v1}

for part in 1 2; do
for window in 24 32; do
run="$RESULT_ROOT/part_${part}/w${window}_o1"
"$PYTHON" "$HERE/export_frame_own.py" \
--prediction-cache "$run/track2_part${part}_w${window}_o1.pt" \
--image-root "$IMAGE_ROOT/part_${part}" \
--ego-mask "$EGO_MASK" \
--output-root "$run/$EXPORT_NAME" \
--confidence-min 0.18 \
--voxel-size 0.01
done
done
