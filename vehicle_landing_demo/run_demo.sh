#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$PROJECT_DIR")"
CATKIN_WS="${CATKIN_WS:-/home/fenglijun/catkin_ws}"
PX4_DIR="${PX4_DIR:-/home/fenglijun/PX4_Firmware}"
XTDRONE_DIR="${XTDRONE_DIR:-/home/fenglijun/XTDrone}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
CATKIN_SETUP="${CATKIN_WS}/devel/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: ROS setup not found: $ROS_SETUP" >&2
  exit 1
fi
if [[ ! -f "$CATKIN_SETUP" ]]; then
  echo "ERROR: catkin setup not found: $CATKIN_SETUP" >&2
  echo "Set CATKIN_WS=/path/to/your/catkin_ws and try again." >&2
  exit 1
fi
if [[ ! -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]]; then
  echo "ERROR: PX4 SITL is not built under: $PX4_DIR" >&2
  echo "Set PX4_DIR=/path/to/PX4_Firmware and try again." >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$CATKIN_SETUP"

export ROS_PACKAGE_PATH="$WORKSPACE_DIR:$PX4_DIR:$XTDRONE_DIR:${ROS_PACKAGE_PATH:-}"
export GAZEBO_MODEL_PATH="$PROJECT_DIR/models:$PX4_DIR/Tools/sitl_gazebo/models:$XTDRONE_DIR/sitl_config/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="$PX4_DIR/build/px4_sitl_default/build_gazebo:${GAZEBO_PLUGIN_PATH:-}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PROJECT_DIR/.sim_ros_logs}"
mkdir -p "$ROS_LOG_DIR"

exec roslaunch vehicle_landing_demo demo.launch "$@"
