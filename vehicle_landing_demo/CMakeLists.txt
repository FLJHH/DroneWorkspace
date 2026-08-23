cmake_minimum_required(VERSION 3.0.2)
project(vehicle_landing_demo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gazebo_msgs
  mavros_msgs
  rospy
  tf2_ros
  yolov11_ros_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/moving_vehicle.py
  scripts/experiment_recorder.py
  scripts/vehicle_motion_recorder.py
  scripts/plot_experiments.py
  scripts/sdf_tilt_camera.py
  scripts/sdf_scale_tag.py
  scripts/vehicle_precision_landing.py
  scripts/vehicle_yolo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/check_environment.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch meshes models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
