cmake_minimum_required(VERSION 2.8.3)
project(motor_controller)

# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ras_arduino_msgs
)

# Declare that this catkin package's runtime dependencies
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(motor_control src/motorController.cpp)
target_link_libraries(motor_control ${catkin_LIBRARIES})
add_dependencies(motor_control ${catkin_EXPORTED_TARGETS})
