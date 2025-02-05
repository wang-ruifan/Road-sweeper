cmake_minimum_required(VERSION 3.0.2)
project(road_sweeper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  op_planner
)
catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(auto_sweep_node node/auto_sweep_node.cpp)
target_link_libraries(auto_sweep_node ${catkin_LIBRARIES})

install(TARGETS auto_sweep_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)