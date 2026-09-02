#!/usr/bin/env python3
"""Emit the PX4 Iris SDF with its camera tilted 45 degrees from nadir."""

import sys
import re


def main():
    if len(sys.argv) not in (4, 5):
        raise SystemExit("usage: sdf_tilt_camera.py MODEL.sdf MAVLINK_TCP_PORT GIMBAL_UDP_PORT [PITCH_OFFSET_DEG]")

    with open(sys.argv[1], "r") as stream:
        sdf = stream.read()

    # Keep the original SDF text intact.  XML parse/write changes whitespace,
    # attribute quoting and, more importantly, can alter plugin payloads used
    # by the PX4 Gazebo bridge.  Only replace the fields this wrapper owns.
    camera_pattern = (
        r"(<include\b[^>]*>\s*<uri>\s*model://monocular_camera\s*</uri>\s*"
        r"<pose\b[^>]*>)[^<]*(</pose>)"
    )
    pitch_offset = float(sys.argv[4]) if len(sys.argv) == 5 else 0.0
    pitch = 0.7853981634 + pitch_offset * 3.141592653589793 / 180.0
    sdf, camera_count = re.subn(
        camera_pattern,
        r"\g<1>0 0 -0.05 0 {:.10f} 0\g<2>".format(pitch),
        sdf,
        count=1,
        flags=re.DOTALL,
    )
    if camera_count != 1:
        raise SystemExit("monocular_camera include not found")

    sdf, port_count = re.subn(
        r"(<plugin\b[^>]*name=['\"]mavlink_interface['\"][\s\S]*?"
        r"<mavlink_tcp_port\b[^>]*>)[^<]*(</mavlink_tcp_port>)",
        r"\g<1>{}\g<2>".format(sys.argv[2]), sdf, count=1,
    )
    if port_count != 1:
        raise SystemExit("mavlink_interface port not found")

    # The gimbal plugin is optional in some PX4 model versions.  If present,
    # update it in place; do not synthesize XML nodes or rewrite the document.
    sdf = re.sub(
        r"(<plugin\b[^>]*name=['\"]gimbal_controller['\"][\s\S]*?"
        r"<udp_gimbal_port_remote\b[^>]*>)[^<]*(</udp_gimbal_port_remote>)",
        r"\g<1>{}\g<2>".format(sys.argv[3]), sdf, count=1,
    )

    sys.stdout.write(sdf)


if __name__ == "__main__":
    main()
