#!/usr/bin/env python3
"""
Step 2 of camera calibration: turn the captured chessboard images into FTC
calibration values (fx, fy, cx, cy + distortion) and print a ready-to-paste
teamwebcamcalibrations.xml block.

Usage:
    python3 calibrate.py --images images --cols 9 --rows 6 --square 25

Notes:
  * --square is the real edge length of one square (e.g. mm). The camera matrix
    (fx, fy, cx, cy) and distortion are INDEPENDENT of this value, so it doesn't
    have to be exact for our purposes — just keep it consistent.
  * All images must be the SAME resolution, and that must be the resolution you
    run on the robot.
"""
import argparse
import glob
import os

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser(description="Compute camera intrinsics from chessboard images.")
    ap.add_argument("--images", default="images", help="folder of captured .png/.jpg images")
    ap.add_argument("--cols", type=int, default=9, help="INNER corners per row (must match capture)")
    ap.add_argument("--rows", type=int, default=6, help="INNER corners per column (must match capture)")
    ap.add_argument("--square", type=float, default=25.0, help="square edge length (any consistent unit)")
    args = ap.parse_args()

    pattern = (args.cols, args.rows)

    # Object points for one board: (0,0,0), (1,0,0), ... scaled by square size, z=0.
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= args.square

    paths = sorted(sum([glob.glob(os.path.join(args.images, e))
                        for e in ("*.png", "*.jpg", "*.jpeg", "*.PNG", "*.JPG")], []))
    if not paths:
        raise SystemExit(f"No images found in '{args.images}'. Run capture_chessboard.py first.")

    obj_points, img_points, used = [], [], []
    image_size = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for p in paths:
        img = cv2.imread(p)
        if img is None:
            print(f"  skip (unreadable): {p}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = (gray.shape[1], gray.shape[0])  # (w, h)
        if image_size is None:
            image_size = size
        elif size != image_size:
            print(f"  skip (size {size} != {image_size}): {p}")
            continue

        found, corners = cv2.findChessboardCorners(gray, pattern, None)
        if not found:
            print(f"  skip (no board): {p}")
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        obj_points.append(objp)
        img_points.append(corners)
        used.append(p)
        print(f"  ok: {os.path.basename(p)}")

    n = len(obj_points)
    if n < 5:
        raise SystemExit(f"Only {n} usable images — need at least ~10 (ideally 15-25). Capture more.")

    print(f"\nCalibrating on {n} images at {image_size[0]}x{image_size[1]}...")
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, image_size, None, None)

    # Per-image reprojection error, so you can spot/re-shoot bad frames.
    print("\nPer-image reprojection error (px):")
    worst = []
    for i, path in enumerate(used):
        proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, dist)
        err = cv2.norm(img_points[i], proj, cv2.NORM_L2) / len(proj)
        worst.append((err, os.path.basename(path)))
    for err, name in sorted(worst, reverse=True)[:5]:
        print(f"  {err:6.3f}  {name}")

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    d = dist.ravel().tolist()
    d = (d + [0.0] * 8)[:8]  # OpenCV gives [k1,k2,p1,p2,k3(,k4,k5,k6)]; pad to 8 for FTC

    print("\n================ RESULTS ================")
    print(f"Overall RMS reprojection error: {rms:.4f} px  (aim for < 0.7; < 0.4 is great)")
    print(f"Resolution : {image_size[0]} x {image_size[1]}")
    print(f"focalLength    (fx, fy) : {fx:.4f}, {fy:.4f}")
    print(f"principalPoint (cx, cy) : {cx:.4f}, {cy:.4f}")
    print(f"distortion [k1,k2,p1,p2,k3,k4,k5,k6]: {d}")

    dist_str = ", ".join(f"{v:g}" for v in d)
    print("\n---- paste into TeamCode/src/main/res/xml/teamwebcamcalibrations.xml ----")
    print("---- (put your real USB vid/pid in the <Camera> tag — see README) ----\n")
    print(f'''    <Camera vid="0xXXXX" pid="0xXXXX">
        <Calibration
            size="{image_size[0]} {image_size[1]}"
            focalLength="{fx:.4f}f, {fy:.4f}f"
            principalPoint="{cx:.4f}f, {cy:.4f}f"
            distortionCoefficients="{dist_str}"
            />
    </Camera>''')

    print("\n---- OR, the quick code path (ignores distortion) in your AprilTagProcessor.Builder ----")
    print(f"        .setLensIntrinsics({fx:.4f}, {fy:.4f}, {cx:.4f}, {cy:.4f})")


if __name__ == "__main__":
    main()
