cmake_minimum_required(VERSION 3.0.2)
project(road_sweeper)

find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  autoware_msgs
  vector_map_msgs
  socketcan_bridge
  visualization_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs autoware_msgs vector_map_msgs socketcan_bridge visualization_msgs std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

add_executable(auto_sweep_node node/auto_sweep_node.cpp)
target_link_libraries(auto_sweep_node ${catkin_LIBRARIES})

add_executable(vehiclecmd_to_can_node node/vehiclecmd_to_can_node.cpp)
target_link_libraries(vehiclecmd_to_can_node ${catkin_LIBRARIES})

set(road_sweeper_gui_SRCS
  src/road_sweeper_gui.cpp
  include/road_sweeper_gui.hpp
)

add_executable(road_sweeper_gui ${road_sweeper_gui_SRCS})
target_link_libraries(road_sweeper_gui
  ${catkin_LIBRARIES}
  Qt5::Widgets
  Qt5::Core
)

install(TARGETS auto_sweep_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS vehiclecmd_to_can_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS road_sweeper_gui
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)