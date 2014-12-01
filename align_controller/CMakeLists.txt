cmake_minimum_required(VERSION 2.8.3)
project(align_controller)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs ras_arduino_msgs rosutil
hardware_msgs)

catkin_package(
  DEPENDS orocos_kdl
  CATKIN_DEPENDS roscpp std_msgs ras_arduino_msgs rosutil hardware_msgs
  INCLUDE_DIRS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(align_controller src/alignController.cpp)
target_link_libraries(align_controller ${catkin_LIBRARIES} rosutil)
add_dependencies(align_controller ${catkin_EXPORTED_TARGETS})
