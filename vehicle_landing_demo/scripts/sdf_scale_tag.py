#!/usr/bin/env python3
"""Emit a tagged-vehicle SDF with its geometry-built AprilTag scaled down."""

import sys
import xml.etree.ElementTree as ET


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: sdf_scale_tag.py MODEL.sdf SCALE")
    scale = float(sys.argv[2])
    tree = ET.parse(sys.argv[1])
    root = tree.getroot()
    changed = 0
    for visual in root.iter("visual"):
        if visual.get("name", "") == "landing_plate":
            # Keep the 1.8 m collision surface, but avoid hiding the SUV roof
            # behind a large white rectangle that resembles an appliance.
            plate_size = visual.find("geometry/box/size")
            if plate_size is not None:
                values = [float(value) for value in plate_size.text.split()]
                values[0], values[1] = 0.75, 0.75
                plate_size.text = " ".join("{:.6f}".format(value) for value in values)
            continue
        if not visual.get("name", "").startswith("tag_"):
            continue
        pose = visual.find("pose")
        size = visual.find("geometry/box/size")
        if pose is None or size is None:
            continue
        pose_values = [float(value) for value in pose.text.split()]
        size_values = [float(value) for value in size.text.split()]
        pose_values[0] *= scale
        pose_values[1] *= scale
        size_values[0] *= scale
        size_values[1] *= scale
        pose.text = " ".join("{:.6f}".format(value) for value in pose_values)
        size.text = " ".join("{:.6f}".format(value) for value in size_values)
        changed += 1
    if changed == 0:
        raise SystemExit("no tag visuals found")
    sys.stdout.write(ET.tostring(root, encoding="unicode"))


if __name__ == "__main__":
    main()
