#!/usr/bin/env python3
"""Rectify gf_road RGB and its ego mask with the supplied fisheye calibration."""
import argparse, json
from pathlib import Path
import cv2
import numpy as np

K = np.array([[663.352839696, 0., 720.883068385], [0., 662.539145725, 463.615796468], [0., 0., 1.]])
D = np.array([-0.0089323473224, -0.048465071177, 0.0390556786134, -0.011927819504])
R = np.eye(3)
P = np.array([[547.354, 0., 751.91], [0., 546.682, 457.682], [0., 0., 1.]])
SIZE = (1440, 928)


def main():
    p = argparse.ArgumentParser(description=__doc__); p.add_argument("--input-dir", type=Path, required=True)
    p.add_argument("--ego-mask", type=Path, required=True); p.add_argument("--output-root", type=Path, required=True); a = p.parse_args()
    if a.output_root.exists(): p.error(f"output root already exists: {a.output_root}")
    images = sorted(a.input_dir.glob("*.jpg")); mask = cv2.imread(str(a.ego_mask), 0)
    if not images or mask is None or mask.shape[::-1] != SIZE: p.error("missing images or mask is not 1440x928")
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, R, P, SIZE, cv2.CV_16SC2)
    image_out = a.output_root / "images"; image_out.mkdir(parents=True)
    for source in images:
        image = cv2.imread(str(source));
        if image is None or image.shape[1::-1] != SIZE: raise ValueError(f"invalid 1440x928 image: {source}")
        if not cv2.imwrite(str(image_out / source.name), cv2.remap(image, map1, map2, cv2.INTER_LINEAR)): raise OSError(source)
    rect_mask = cv2.remap(mask, map1, map2, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=255)
    cv2.imwrite(str(a.output_root / "ego_vehicle_mask_rectified.png"), rect_mask)
    (a.output_root / "report.json").write_text(json.dumps({"status":"PASS","images":len(images),"size":SIZE,
        "mask_policy":"white/nonzero is ego and excluded; out-of-FOV rectification border also excluded",
        "K":K.tolist(),"D":D.tolist(),"R":R.tolist(),"P":P.tolist()}, indent=2)+"\n")


if __name__ == "__main__": main()
