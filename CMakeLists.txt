cmake_minimum_required(VERSION 3.0.2)
project(rad4d_extract)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS scripts/extract_img.py scripts/extract_pcd.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)