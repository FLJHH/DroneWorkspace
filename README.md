# DroneWorkspace

PX4 moving-vehicle perception and precision landing research workspace.

This is a derivative research workspace built on the XTDrone simulation
ecosystem and its ROS, Gazebo, PX4 SITL and MAVROS integration. It is not the
official XTDrone repository, and the maintainers of this repository are not
the original authors of XTDrone.

The principal work added in this repository concerns:

- YOLO and AprilTag based moving-platform perception;
- autonomous precision-landing control and diagnostic state reporting;
- repeatable CSV/JSON experiment recording and diagnostic plots;
- a validated five-run static landing baseline;
- isolated 0.5 m/s vehicle-motion and perception validation.

The current milestone is **“静态 baseline + 移动车辆感知验证通过”**. Moving-target
tracking and autonomous landing on a moving platform are not yet implemented.
See [PROJECT_STATUS.md](PROJECT_STATUS.md) and
[vehicle_landing_demo/NEXT_STAGE_REPORT.md](vehicle_landing_demo/NEXT_STAGE_REPORT.md)
for the evidence and limitations.

## Repository scope

The external XTDrone, PX4 and catkin workspaces are runtime dependencies and
are intentionally not vendored. Local symlinks and generated ROS/Gazebo logs,
training datasets, build products and caches are ignored. Curated experiment
CSV/JSON files and diagnostic PNGs under `vehicle_landing_demo/experiments/`
are versioned.

The required runtime detector weight is:

```text
vehicle_landing_demo/training/vehicle_yolo11s_descent/weights/best.pt
size: 19,151,059 bytes (about 18.26 MiB)
```

It is stored using regular Git because it is below GitHub's 100 MB per-file
limit. It was produced for the current Ultralytics YOLO-based detector and is
required by `vehicle_landing_demo/launch/demo.launch`.

## Attribution and licensing

- XTDrone is copyright Kun Xiao and contributors and is licensed under MIT.
- PX4 is copyright the PX4 Development Team and is licensed under BSD-3-Clause.
- MAVROS is available under BSD/GPLv3/LGPLv3 license alternatives.
- `apriltag_ros` and AprilTag use BSD-2-Clause licenses.
- The installed `yolov11_ros` package declares Apache License 2.0 (`APLv2`).
- Ultralytics is licensed under AGPL-3.0 unless an applicable Enterprise
  License has been obtained.

The original contributions in `vehicle_landing_demo` retain their existing MIT
license. Third-party components remain governed by their own licenses; the MIT
file does not override them. In particular, use or distribution of Ultralytics
software and associated model artifacts must comply with AGPL-3.0 or a valid
commercial license. Private storage for internal research is not a declaration
that the combined project may later be publicly distributed under MIT alone.

See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for source links,
attribution and known obligations. This summary is an engineering inventory,
not legal advice.
