#!/usr/bin/env python3
"""Add reviewed real glideslope frames to the robust training dataset."""

import glob
import os

import cv2
import numpy as np
from ultralytics import YOLO


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AUDIT = os.path.join(ROOT, "datasets", "descent_audit")
DATASET = os.path.join(ROOT, "datasets", "vehicle_robust")
WEIGHT = os.path.join(ROOT, "training", "vehicle_yolo11s_robust", "weights", "best.pt")


def model_box(model, image):
    height, width = image.shape[:2]
    result = model.predict(image, imgsz=1280, conf=0.005, device=0, verbose=False)[0]
    candidates = []
    for box in result.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        cx, cy = (x1 + x2) / (2 * width), (y1 + y2) / (2 * height)
        if 0.25 < cx < 0.75 and 0.1 < cy < 0.95 and x2 - x1 < 0.8 * width:
            candidates.append(((x2 - x1) * (y2 - y1), (x1, y1, x2, y2)))
    return max(candidates)[1]


def segmented_box(image):
    """Reviewed color segmentation for the close 10--4 m audit frames."""
    b, g, r = cv2.split(image)
    maximum = np.maximum.reduce([b, g, r]).astype(int)
    minimum = np.minimum.reduce([b, g, r]).astype(int)
    gray = (b.astype(int) + g.astype(int) + r.astype(int)) / 3
    mask = ((gray > 58) & (gray < 180) & (maximum - minimum < 28)).astype("uint8")
    mask[:130, :] = 0
    mask[:, :180] = 0
    mask[:, 700:] = 0
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    parts = []
    for index in range(1, count):
        x, y, width, height, area = stats[index]
        if area > 8 and 230 < x + width / 2 < 560 and y > 100:
            parts.append((x, y, x + width, y + height))
    return (min(p[0] for p in parts), min(p[1] for p in parts),
            max(p[2] for p in parts), max(p[3] for p in parts))


def main():
    model = YOLO(WEIGHT)
    image_dir = os.path.join(DATASET, "images", "train")
    label_dir = os.path.join(DATASET, "labels", "train")
    written = 0
    for path in sorted(glob.glob(os.path.join(AUDIT, "roof_*.jpg"))):
        altitude = int(os.path.basename(path)[5:7])
        if not 4 <= altitude <= 22:
            continue
        image = cv2.imread(path)
        height, width = image.shape[:2]
        x1, y1, x2, y2 = model_box(model, image) if altitude >= 11 else segmented_box(image)
        pad_x, pad_y = 0.04 * (x2 - x1), 0.03 * (y2 - y1)
        x1, y1 = max(0, x1 - pad_x), max(0, y1 - pad_y)
        x2, y2 = min(width - 1, x2 + pad_x), min(height - 1, y2 + pad_y)
        label = ((x1 + x2) / (2 * width), (y1 + y2) / (2 * height),
                 (x2 - x1) / width, (y2 - y1) / height)
        for variant in range(10):
            output = image.copy()
            if variant:
                alpha = 0.82 + 0.04 * variant
                beta = (variant % 3 - 1) * 8
                output = cv2.convertScaleAbs(output, alpha=alpha, beta=beta)
            if variant % 3 == 0:
                # Retain the full-car target while removing the roof-code cue.
                tx1 = int((label[0] - 0.10 * label[2]) * width)
                tx2 = int((label[0] + 0.10 * label[2]) * width)
                ty1 = int((label[1] - 0.29 * label[3]) * height)
                ty2 = int((label[1] - 0.08 * label[3]) * height)
                cv2.rectangle(output, (tx1, ty1), (tx2, ty2), (55, 55, 55), -1)
            stem = "descent_{:02d}_{:02d}".format(altitude, variant)
            cv2.imwrite(os.path.join(image_dir, stem + ".jpg"), output)
            with open(os.path.join(label_dir, stem + ".txt"), "w") as stream:
                stream.write("0 {:.6f} {:.6f} {:.6f} {:.6f}\n".format(*label))
            written += 1
    print("added {} reviewed real-descent samples".format(written))


if __name__ == "__main__":
    main()
