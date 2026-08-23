#!/usr/bin/env bash
set -u

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CATKIN_WS="${CATKIN_WS:-/home/fenglijun/catkin_ws}"
PX4_DIR="${PX4_DIR:-/home/fenglijun/PX4_Firmware}"
XTDRONE_DIR="${XTDRONE_DIR:-/home/fenglijun/XTDrone}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
FAILED=0

check_file() {
  if [[ -e "$1" ]]; then
    printf 'OK      %s\n' "$2"
  else
    printf 'MISSING %s: %s\n' "$2" "$1"
    FAILED=1
  fi
}

check_command() {
  if command -v "$1" >/dev/null 2>&1; then
    printf 'OK      %s\n' "$2"
  else
    printf 'MISSING %s (command: %s)\n' "$2" "$1"
    FAILED=1
  fi
}

check_file "/opt/ros/$ROS_DISTRO/setup.bash" "ROS $ROS_DISTRO"
check_file "$CATKIN_WS/devel/setup.bash" "catkin workspace"
check_file "$PX4_DIR/build/px4_sitl_default/bin/px4" "PX4 SITL build"
check_file "$XTDRONE_DIR/sitl_config" "XTDrone"
check_file "$PROJECT_DIR/training/vehicle_yolo11s_descent/weights/best.pt" "YOLO weights"
check_command python3 "Python 3"

if [[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/$ROS_DISTRO/setup.bash"
  if [[ -f "$CATKIN_WS/devel/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "$CATKIN_WS/devel/setup.bash"
  fi
  check_command roslaunch "roslaunch"
  check_command gzserver "Gazebo server"
  for package in mavros apriltag_ros yolov11_ros_msgs; do
    if rospack find "$package" >/dev/null 2>&1; then
      printf 'OK      ROS package %s\n' "$package"
    else
      printf 'MISSING ROS package %s\n' "$package"
      FAILED=1
    fi
  done
fi

if python3 -c 'import cv2, numpy, ultralytics' >/dev/null 2>&1; then
  printf 'OK      Python packages cv2, numpy, ultralytics\n'
else
  printf 'MISSING one or more Python packages: cv2, numpy, ultralytics\n'
  FAILED=1
fi

if (( FAILED )); then
  echo
  echo "Environment check failed. Set CATKIN_WS, PX4_DIR or XTDRONE_DIR if installed elsewhere."
  exit 1
fi

echo
echo "Environment is ready. Start with:"
echo "  $PROJECT_DIR/run_demo.sh gui:=true visualize_yolo:=true"
