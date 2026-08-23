#!/usr/bin/env python3
"""Expand a xacro and remove every Gazebo ray sensor from the URDF."""

import sys
import xml.etree.ElementTree as ET

import xacro


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: xacro_no_lidar.py MODEL.xacro [name:=value ...]")

    mappings = {}
    for assignment in sys.argv[2:]:
        key, separator, value = assignment.partition(":=")
        if not separator:
            raise SystemExit("invalid xacro assignment: " + assignment)
        mappings[key] = value

    document = xacro.process_file(sys.argv[1], mappings=mappings)
    root = ET.fromstring(document.toxml())
    removed = 0
    for gazebo in root.findall("gazebo"):
        for sensor in list(gazebo.findall("sensor")):
            if sensor.get("type") in ("ray", "gpu_ray"):
                gazebo.remove(sensor)
                removed += 1

    if removed == 0:
        print("warning: no ray sensors found", file=sys.stderr)
    sys.stdout.write(ET.tostring(root, encoding="unicode"))


if __name__ == "__main__":
    main()
