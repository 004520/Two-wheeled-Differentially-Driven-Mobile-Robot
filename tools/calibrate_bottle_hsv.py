#!/usr/bin/env python3
"""Derive the bottle HSV range for the vision_detector colour fallback.

Why this exists
---------------
YOLO stops seeing a 0.22 m bottle once it is smaller than ~50 px on the
640x480 frame (about 1.6 m).  The HSV path keeps working further out, but only
if the hue/saturation/value window actually matches *your* bottle -- and a
transparent bottle, a blue cap and a printed label all sit in completely
different places in HSV space.  Guessing produces false positives, so the
``bottle`` profile ships disabled and this tool produces the numbers.

Usage
-----
Offline, on frames you already captured (recommended -- no display needed)::

    python3 calibrate_bottle_hsv.py --images ~/bottle_frames

Let YOLO find the bottle (put it CLOSE, ~0.5 m, so YOLO is reliable)::

    python3 calibrate_bottle_hsv.py --images ~/bottle_frames --auto

Or point at a known box::

    python3 calibrate_bottle_hsv.py --images ~/bottle_frames --box 300,180,60,160

With a display you can draw the box instead::

    python3 calibrate_bottle_hsv.py --images ~/bottle_frames --roi

It prints a ready-to-paste ``COLOR_PROFILES["bottle"]`` snippet.
"""

from __future__ import annotations

import argparse
import glob
import os
import sys

import cv2
import numpy as np


def load_images(path: str) -> list:
    if os.path.isdir(path):
        files = sorted(
            f for f in glob.glob(os.path.join(path, "*"))
            if f.lower().endswith((".jpg", ".jpeg", ".png", ".bmp"))
        )
    else:
        files = [path]
    images = []
    for f in files:
        img = cv2.imread(f)
        if img is not None:
            images.append((f, img))
    return images


def box_from_yolo(images, weights: str, conf: float) -> list:
    from ultralytics import YOLO  # imported lazily, only when needed

    model = YOLO(weights)
    boxes = []
    for name, img in images:
        res = model.predict(img, imgsz=640, conf=conf, verbose=False)[0]
        best, bh = None, -1.0
        for b in getattr(res, "boxes", None) or []:
            if int(b.cls) != 39:          # COCO bottle
                continue
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            if (y2 - y1) > bh:
                best, bh = (int(x1), int(y1), int(x2), int(y2)), y2 - y1
        boxes.append(best)
        if best is None:
            print(f"  ! no bottle found in {os.path.basename(name)}", file=sys.stderr)
    return boxes


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--images", required=True,
                    help="image file or directory of bottle frames")
    ap.add_argument("--auto", action="store_true",
                    help="locate the bottle with YOLO (place it close, ~0.5 m)")
    ap.add_argument("--roi", action="store_true",
                    help="draw the box by hand (needs a display)")
    ap.add_argument("--box", default="",
                    help="fixed box as x,y,w,h applied to every image")
    ap.add_argument("--weights", default="yolo11n.pt", help="weights for --auto")
    ap.add_argument("--conf", type=float, default=0.5)
    ap.add_argument("--shrink", type=float, default=0.25,
                    help="trim this fraction off each side of the box so the "
                         "background is not sampled (default 0.25)")
    ap.add_argument("--lo", type=float, default=2.0,
                    help="lower percentile (default 2)")
    ap.add_argument("--hi", type=float, default=98.0,
                    help="upper percentile (default 98)")
    ap.add_argument("--margin", type=int, default=10,
                    help="widen the resulting window by this much")
    args = ap.parse_args()

    images = load_images(args.images)
    if not images:
        raise SystemExit(f"no readable images under {args.images}")

    if args.box:
        x, y, w, h = [int(v) for v in args.box.split(",")]
        boxes = [(x, y, x + w, y + h)] * len(images)
    elif args.auto:
        boxes = box_from_yolo(images, args.weights, args.conf)
    elif args.roi:
        boxes = []
        for name, img in images:
            cv2.imshow("draw the bottle, then SPACE/ENTER", img)
            r = cv2.selectROI("draw the bottle, then SPACE/ENTER", img,
                              fromCenter=False, showCrosshair=True)
            cv2.destroyAllWindows()
            if r[2] <= 0 or r[3] <= 0:
                boxes.append(None)
                continue
            boxes.append((int(r[0]), int(r[1]), int(r[0] + r[2]), int(r[1] + r[3])))
    else:
        raise SystemExit("pick one of --auto, --roi or --box")

    pixels = []
    used = 0
    for (name, img), box in zip(images, boxes):
        if box is None:
            continue
        x1, y1, x2, y2 = box
        dx = int((x2 - x1) * args.shrink / 2.0)
        dy = int((y2 - y1) * args.shrink / 2.0)
        patch = img[y1 + dy:y2 - dy, x1 + dx:x2 - dx]
        if patch.size == 0:
            continue
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).reshape(-1, 3)
        pixels.append(hsv)
        used += 1

    if not pixels:
        raise SystemExit("no bottle pixels sampled -- check the boxes")

    data = np.concatenate(pixels, axis=0).astype(np.float32)
    print(f"# sampled {data.shape[0]} px from {used} image(s)\n")

    lo_p = np.percentile(data, args.lo, axis=0)
    hi_p = np.percentile(data, args.hi, axis=0)
    lower = np.floor(lo_p - args.margin).clip(0, 255)
    upper = np.ceil(hi_p + args.margin).clip(0, 179 if False else 255)
    upper[0] = min(upper[0], 179)          # OpenCV hue range is 0..179

    h_med = float(np.median(data[:, 0]))
    print("# channel summary (H,S,V):")
    print(f"#   lower percentile : {lo_p.round(1).tolist()}")
    print(f"#   upper percentile : {hi_p.round(1).tolist()}")
    print(f"#   median hue       : {h_med:.1f}")

    if h_med < 10 or h_med > 170:
        print("\n# WARNING: the hue sits at the red wrap-around.  A single "
              "cv2.inRange window cannot cover it; use two masks or pick a "
              "different part of the bottle (cap/label).")

    print("\n# paste into COLOR_PROFILES['bottle'] in "
          "mycar_vision/detection_utils.py:")
    print(f'    "hsv_lower": ({int(lower[0])}, {int(lower[1])}, {int(lower[2])}),')
    print(f'    "hsv_upper": ({int(upper[0])}, {int(upper[1])}, {int(upper[2])}),')
    print("\n# then add \"bottle\" to color_classes in "
          "config/vision_detector.yaml")


if __name__ == "__main__":
    main()
