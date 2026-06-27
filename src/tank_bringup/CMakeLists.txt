cmake_minimum_required(VERSION 3.8)
project(tank_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  rviz
  docs
  maps
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  tank_bringup
  DESTINATION lib/python3.10/site-packages
)

install(PROGRAMS
  scripts/initialpose_bridge.py
  scripts/cloud_frame_transform.py
  scripts/fastlivo_nav_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
