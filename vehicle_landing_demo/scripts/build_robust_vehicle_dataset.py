#!/usr/bin/env python3
"""Build multi-scale vehicle data plus isolated-AprilTag hard negatives."""

import glob
import os
import shutil

import cv2
import numpy as np


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOURCE = os.path.join(ROOT, "datasets", "vehicle")
OUTPUT = os.path.join(ROOT, "datasets", "vehicle_robust")


def save(split, stem, image, label=None):
    image_dir = os.path.join(OUTPUT, "images", split)
    label_dir = os.path.join(OUTPUT, "labels", split)
    os.makedirs(image_dir, exist_ok=True)
    os.makedirs(label_dir, exist_ok=True)
    cv2.imwrite(os.path.join(image_dir, stem + ".jpg"), image)
    with open(os.path.join(label_dir, stem + ".txt"), "w") as stream:
        if label is not None:
            stream.write("0 {:.6f} {:.6f} {:.6f} {:.6f}\n".format(*label))


def scale_about_box(image, label, target_height):
    height, width = image.shape[:2]
    cx, cy, bw, bh = label
    scale = target_height / bh
    matrix = np.array([[scale, 0.0, width * 0.5 - scale * cx * width],
                       [0.0, scale, height * 0.5 - scale * cy * height]], dtype=np.float32)
    transformed = cv2.warpAffine(
        image, matrix, (width, height), flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT, borderValue=(170, 170, 170))
    return transformed, (0.5, 0.5, min(0.96, bw * scale), min(0.96, bh * scale))


def main():
    if os.path.isdir(OUTPUT):
        shutil.rmtree(OUTPUT)
    index = 0
    for split in ("train", "val"):
        files = sorted(glob.glob(os.path.join(SOURCE, "images", split, "*.jpg")))
        for image_path in files:
            image = cv2.imread(image_path)
            label_path = image_path.replace(os.sep + "images" + os.sep,
                                            os.sep + "labels" + os.sep)
            label_path = os.path.splitext(label_path)[0] + ".txt"
            values = [float(value) for value in open(label_path).read().split()[1:]]
            cx, cy, bw, bh = values

            # Deliberately vary apparent range; the original collection was
            # effectively 200 copies at one scale.
            scales = (0.07, 0.12, 0.22, 0.38) if split == "train" else (0.09, 0.28)
            for variant, target_height in enumerate(scales):
                augmented, new_label = scale_about_box(image, values, target_height)
                if variant % 2:
                    # Remove the tag's high-contrast shortcut while retaining
                    # the vehicle label, forcing the network to use body shape.
                    h, w = augmented.shape[:2]
                    ncx, ncy, nbw, nbh = new_label
                    x1 = int((ncx - 0.11 * nbw) * w)
                    x2 = int((ncx + 0.11 * nbw) * w)
                    y1 = int((ncy - 0.30 * nbh) * h)
                    y2 = int((ncy - 0.08 * nbh) * h)
                    cv2.rectangle(augmented, (x1, y1), (x2, y2), (55, 55, 55), -1)
                save(split, "vehicle_{:06d}".format(index), augmented, new_label)
                index += 1

            if split == "train":
                # Isolate the roof marker and explicitly label the image as
                # background. These are hard negatives for the observed fault.
                h, w = image.shape[:2]
                x1 = max(0, int((cx - 0.18 * bw) * w))
                x2 = min(w, int((cx + 0.18 * bw) * w))
                y1 = max(0, int((cy - 0.38 * bh) * h))
                y2 = min(h, int((cy - 0.02 * bh) * h))
                marker = image[y1:y2, x1:x2]
                canvas = np.full_like(image, 160)
                size = max(48, min(320, marker.shape[0] * 3))
                marker = cv2.resize(marker, (size, size), interpolation=cv2.INTER_CUBIC)
                top, left = (h - size) // 2, (w - size) // 2
                canvas[top:top + size, left:left + size] = marker
                save(split, "tag_negative_{:06d}".format(index), canvas)
                index += 1
    print("wrote {} robust samples to {}".format(index, OUTPUT))


if __name__ == "__main__":
    main()
