cmake_minimum_required(VERSION 2.8.3)
project(motor_controller2)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs ras_arduino_msgs)

catkin_package(
  DEPENDS orocos_kdl
  CATKIN_DEPENDS roscpp std_msgs ras_arduino_msgs
  INCLUDE_DIRS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_executable(motor_controller2 src/motorController2.cpp)
target_link_libraries(motor_controller2 ${catkin_LIBRARIES})
add_dependencies(motor_controller2 ${catkin_EXPORTED_TARGETS})
