#!/usr/bin/env python3
"""Export one masked, voxelized, camera-own FLU PLY per physical image."""
import argparse, csv, json
from pathlib import Path

import cv2
import numpy as np
import torch

RDF_TO_FLU = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def array(value):
    return value.detach().cpu().numpy() if torch.is_tensor(value) else np.asarray(value)


def voxelize(xyz, rgb, size):
    if size <= 0 or not len(xyz):
        return xyz, rgb
    inverse = np.unique(np.floor(xyz / size).astype(np.int64), axis=0, return_inverse=True)[1]
    count = int(inverse.max()) + 1
    out_xyz = np.zeros((count, 3)); out_rgb = np.zeros((count, 3))
    np.add.at(out_xyz, inverse, xyz); np.add.at(out_rgb, inverse, rgb)
    n = np.bincount(inverse, minlength=count)
    return out_xyz / n[:, None], np.rint(out_rgb / n[:, None]).clip(0, 255).astype(np.uint8)


def write_ply(path, xyz, rgb):
    with path.open("w") as stream:
        stream.write("ply\nformat ascii 1.0\n")
        stream.write(f"element vertex {len(xyz)}\nproperty float x\nproperty float y\nproperty float z\n")
        stream.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        for point, color in zip(xyz, rgb):
            stream.write(f"{point[0]:.8g} {point[1]:.8g} {point[2]:.8g} {color[0]} {color[1]} {color[2]}\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prediction-cache", type=Path, required=True)
    parser.add_argument("--image-root", type=Path, required=True)
    parser.add_argument("--ego-mask", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--flat-output", action="store_true", help="Write PLY files directly under output-root")
    parser.add_argument("--confidence-min", type=float, default=.18)
    parser.add_argument("--voxel-size", type=float, default=.01)
    args = parser.parse_args()
    if args.output_root.exists():
        parser.error(f"output root already exists: {args.output_root}")
    payload = torch.load(args.prediction_cache, map_location="cpu", weights_only=False, mmap=True)
    required = {"local_points", "conf", "images", "window_frame_indices"}
    if missing := required - payload.keys():
        parser.error("cache missing: " + ", ".join(sorted(missing)))
    points, conf, rgb = map(array, (payload["local_points"], payload["conf"], payload["images"]))
    frame_ids = array(payload["window_frame_indices"]).reshape(-1).astype(int)
    if conf.ndim == 4 and conf.shape[-1] == 1: conf = conf[..., 0]
    if rgb.ndim == 4 and rgb.shape[1] == 3: rgb = np.moveaxis(rgb, 1, -1)
    if rgb.dtype != np.uint8:
        if np.nanmax(rgb) <= 1.5: rgb = rgb * 255
        rgb = np.nan_to_num(rgb).clip(0, 255).astype(np.uint8)
    images = sorted(p for p in args.image_root.iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
    mask = cv2.imread(str(args.ego_mask), cv2.IMREAD_GRAYSCALE)
    if mask is None or len(points) != len(frame_ids) or np.any(frame_ids < 0) or np.any(frame_ids >= len(images)):
        parser.error("mask/cache/image population mismatch")
    excluded = cv2.resize(mask, points.shape[2:0:-1], interpolation=cv2.INTER_NEAREST) > 0
    _, rows = np.unique(frame_ids, return_index=True)
    output = args.output_root if args.flat_output else args.output_root / "ply_quicklook" / "frame_own"
    output.mkdir(parents=True)
    index = []
    for row in sorted(rows):
        valid = (np.isfinite(points[row]).all(2) & (points[row, ..., 2] > 0) &
                 np.isfinite(conf[row]) & (conf[row] >= args.confidence_min) & ~excluded)
        xyz = (RDF_TO_FLU @ points[row][valid].T).T
        xyz, color = voxelize(xyz, rgb[row][valid], args.voxel_size)
        name = images[frame_ids[row]].with_suffix(".ply").name
        write_ply(output / name, xyz, color)
        index.append((int(row), int(frame_ids[row]), images[frame_ids[row]].name, name, len(xyz)))
    with (args.output_root / "index.csv").open("w", newline="") as stream:
        writer = csv.writer(stream); writer.writerow(("occurrence_row", "frame_index", "image", "ply", "points")); writer.writerows(index)
    report = {"status":"PASS", "prediction_cache":str(args.prediction_cache.resolve()),
              "image_root":str(args.image_root.resolve()), "ego_mask":str(args.ego_mask.resolve()),
              "coordinate_frame":"camera-own FLU: X front, Y left, Z up", "physical_frames":len(index),
              "confidence_min":args.confidence_min, "voxel_size":args.voxel_size}
    (args.output_root / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
