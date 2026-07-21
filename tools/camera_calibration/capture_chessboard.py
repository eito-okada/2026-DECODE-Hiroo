#!/usr/bin/env python3
"""
Step 1 of camera calibration: capture chessboard images from your webcam.

Plug the SAME webcam you use on the robot into this computer. Print a chessboard
pattern (see README), tape it to something flat and rigid, then move it around in
front of the camera and save ~20 shots from varied angles/distances.

IMPORTANT: capture at the SAME resolution you run on the robot (FTC default 640x480),
because a calibration is only valid for the resolution it was captured at.

Controls:
    SPACE / c : save the current frame (only saves if the full board is detected)
    a         : toggle auto-capture (saves a detected board every ~1s, while it moves)
    u         : undo (delete the last saved image)
    q / ESC   : quit

Usage:
    python3 capture_chessboard.py --camera 0 --cols 9 --rows 6
"""
import argparse
import os
import time

import cv2


def main():
    ap = argparse.ArgumentParser(description="Capture chessboard images for calibration.")
    ap.add_argument("--camera", type=int, default=0, help="webcam index (0 is usually the first)")
    ap.add_argument("--width", type=int, default=640, help="capture width (match the robot)")
    ap.add_argument("--height", type=int, default=480, help="capture height (match the robot)")
    ap.add_argument("--cols", type=int, default=9, help="INNER corners per row (9 for a 10-square row)")
    ap.add_argument("--rows", type=int, default=6, help="INNER corners per column (6 for a 7-square col)")
    ap.add_argument("--out", default="images", help="output folder for saved frames")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    pattern = (args.cols, args.rows)  # OpenCV wants (points_per_row, points_per_col)

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera index {args.camera}")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (actual_w, actual_h) != (args.width, args.height):
        print(f"WARNING: camera gave {actual_w}x{actual_h}, not {args.width}x{args.height}. "
              f"Calibrate and run the robot at the SAME resolution the camera actually delivers.")

    saved = []          # paths of saved images, for undo
    auto = False
    last_auto = 0.0
    print("Live view open. Fill the frame with the board from many angles. See --help for keys.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame grab failed; retrying...")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, pattern,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )

        view = frame.copy()
        if found:
            cv2.drawChessboardCorners(view, pattern, corners, found)

        status = f"saved:{len(saved)}  board:{'YES' if found else 'no '}  auto:{'ON' if auto else 'off'}"
        cv2.putText(view, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("calibration capture (q to quit)", view)

        now = time.time()
        do_save = False
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key in (ord(" "), ord("c")):
            do_save = True
        elif key == ord("a"):
            auto = not auto
            last_auto = now
        elif key == ord("u") and saved:
            os.remove(saved.pop())
            print(f"Undid last capture. Now have {len(saved)}.")

        if auto and found and (now - last_auto) > 1.0:
            do_save = True
            last_auto = now

        if do_save:
            if not found:
                print("No full board detected in this frame — not saved.")
            else:
                path = os.path.join(args.out, f"calib_{len(saved):03d}.png")
                cv2.imwrite(path, frame)  # save the RAW frame, not the annotated one
                saved.append(path)
                print(f"Saved {path}  (total {len(saved)})")

    cap.release()
    cv2.destroyAllWindows()
    print(f"\nDone. {len(saved)} images in '{args.out}'. Aim for 15-25 varied shots, then run calibrate.py.")


if __name__ == "__main__":
    main()
