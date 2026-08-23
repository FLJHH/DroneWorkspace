# Third-party notices

This workspace is a derivative research project and does not claim authorship
of XTDrone, PX4, MAVROS, AprilTag, Gazebo, ROS, Ultralytics or their components.
No external dependency repository is vendored through the local workspace
symlinks.

## Main upstream projects

| Project | Upstream/source | License found | Relevance and obligations |
|---|---|---|---|
| XTDrone | <https://github.com/robin-shaun/XTDrone> | MIT; Copyright (c) 2021 Kun Xiao | Foundation/integration environment. Modification and redistribution are allowed, including privately, provided the copyright and MIT permission notice are retained in copies or substantial portions. No same-license requirement. |
| PX4 Autopilot | <https://github.com/PX4/PX4-Autopilot> | BSD-3-Clause; Copyright PX4 Development Team | External SITL dependency. Redistribution permits modification but requires retention of copyright, conditions and disclaimer; contributor names may not be used for endorsement. No same-license requirement. |
| MAVROS | <https://github.com/mavlink/mavros> | Triple licensed BSD/GPLv3/LGPLv3 | External ROS bridge. The upstream project states use is compatible with any of these license alternatives. No MAVROS source is vendored here. |
| apriltag_ros | <https://github.com/AprilRobotics/apriltag_ros> | BSD-2-Clause; Copyright (c) 2017 California Institute of Technology | External ROS detector wrapper. Redistribution requires retention of its copyright, conditions and disclaimer. |
| AprilTag | <https://github.com/AprilRobotics/apriltag> | BSD-2-Clause; Copyright The Regents of the University of Michigan | External detector library/tag system. Redistribution requires retention of its copyright, conditions and disclaimer. |
| yolov11_ros | Local dependency at `/home/fenglijun/catkin_ws/src/yolov11_ros` | Package manifest declares `APLv2` (Apache License 2.0) | Message/node dependency. No separate local LICENSE file was found during this audit, so exact provenance and notice text remain to be confirmed before public redistribution. |
| Ultralytics | <https://github.com/ultralytics/ultralytics> | AGPL-3.0, or a separately obtained Enterprise License | Python inference/training dependency and source of the toolchain used with `best.pt`. Private internal use is distinct from public conveyance. Distribution or network-service use may require the entire covered/derivative work and corresponding source under AGPL-3.0; obtain legal review or an Enterprise License if those obligations cannot be met. |
| ROS Noetic / Gazebo Classic / OpenCV / NumPy | Installed runtime dependencies | Component-specific licenses | No single license applies to the whole ROS distribution. Exact installed package notices were not copied into this repository; downstream packaging must audit each redistributed binary/source component. |

## Repository license

`vehicle_landing_demo/LICENSE` is the existing MIT license for that package's
original contributions. It is preserved without removing third-party
attribution. It does not relicense third-party dependencies, model artifacts,
meshes, textures or upstream-derived material.

## License texts and provenance retained outside this repository

The development environment contains the original XTDrone `LICENSE` (MIT) and
PX4 `LICENSE` (BSD-3-Clause) in their respective external workspaces. Those
workspaces are linked locally but deliberately excluded from this Git
repository. Their upstream links above are the authoritative sources.

## Items requiring confirmation before public release

- The exact source and redistribution license of the vehicle/AprilTag mesh,
  texture and world assets under `vehicle_landing_demo/models/`, `meshes/` and
  `worlds/` could not be established from embedded notices. Their authoring
  metadata is preserved; no original-author claim is made here.
- The provenance/license terms that apply specifically to the trained
  `training/vehicle_yolo11s_descent/weights/best.pt` should be reviewed with
  the Ultralytics license in force when the model was trained.
- `yolov11_ros` declares Apache-2.0 in its package manifest, but no separate
  license file was found locally in this audit.

Because this repository is initially private, these unresolved public-release
items are recorded rather than silently reclassified. Do not make the
repository public or redistribute these assets without resolving them.
