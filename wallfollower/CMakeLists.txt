cmake_minimum_required(VERSION 2.8.3)
project(wallfollower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ras_arduino_msgs
  rosutil
)

catkin_package(
  DEPENDS orocos_kdl
  CATKIN_DEPENDS roscpp std_msgs ras_arduino_msgs  rosutil
  INCLUDE_DIRS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(wallfollower src/wallfollower.cpp)
target_link_libraries(wallfollower ${catkin_LIBRARIES} rosutil)
add_dependencies(wallfollower ${catkin_EXPORTED_TARGETS})
