#!/usr/bin/env python3
"""Emit the PX4 Iris SDF with its camera tilted 45 degrees from nadir."""

import sys
import xml.etree.ElementTree as ET


def main():
    if len(sys.argv) != 4:
        raise SystemExit("usage: sdf_tilt_camera.py MODEL.sdf MAVLINK_TCP_PORT GIMBAL_UDP_PORT")

    tree = ET.parse(sys.argv[1])
    root = tree.getroot()

    camera_include = None
    for include in root.iter("include"):
        uri = include.find("uri")
        if uri is not None and uri.text == "model://monocular_camera":
            camera_include = include
            break
    if camera_include is None:
        raise SystemExit("monocular_camera include not found")

    # The original +90 degree pitch points straight down. +45 degrees gives
    # a true rear/side vehicle view from the observation position.
    pose = camera_include.find("pose")
    if pose is None:
        pose = ET.SubElement(camera_include, "pose")
    pose.text = "0 0 -0.05 0 0.7853981634 0"

    for plugin in root.iter("plugin"):
        name = plugin.get("name", "")
        if name == "mavlink_interface":
            element = plugin.find("mavlink_tcp_port")
            if element is not None:
                element.text = sys.argv[2]
        elif name == "gimbal_controller":
            element = plugin.find("udp_gimbal_port_remote")
            if element is None:
                element = ET.SubElement(plugin, "udp_gimbal_port_remote")
            element.text = sys.argv[3]

    sys.stdout.write(ET.tostring(root, encoding="unicode"))


if __name__ == "__main__":
    main()
