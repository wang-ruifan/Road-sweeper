cmake_minimum_required(VERSION 3.0.2)
project(gnss_matrix)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosmsg
  rospy
)

catkin_package()

install(PROGRAMS
  scripts/gnss_matrix.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


