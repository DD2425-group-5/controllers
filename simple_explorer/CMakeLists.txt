cmake_minimum_required(VERSION 2.8.3)
project(simple_explorer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ras_arduino_msgs
  rosutil
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wallfollower
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib

  DEPENDS orocos_kdl
  CATKIN_DEPENDS roscpp std_msgs ras_arduino_msgs rosutil
  INCLUDE_DIRS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_explorer src/simple_explorer.cpp)
target_link_libraries(simple_explorer ${catkin_LIBRARIES} rosutil)
add_dependencies(simple_explorer ${catkin_EXPORTED_TARGETS})