cmake_minimum_required(VERSION 2.8.3)
project(motor_controller3)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs ras_arduino_msgs rosutil)

catkin_package(
  DEPENDS orocos_kdl
  CATKIN_DEPENDS roscpp std_msgs ras_arduino_msgs  rosutil
  INCLUDE_DIRS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_controller3 src/motorController3.cpp)
target_link_libraries(motor_controller3 ${catkin_LIBRARIES} rosutil)
add_dependencies(motor_controller3 ${catkin_EXPORTED_TARGETS})
