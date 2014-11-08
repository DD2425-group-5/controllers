cmake_minimum_required(VERSION 2.8.3)
project(wallfollower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rosutil
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wallfollower
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(wallfollower src/wallfollower.cpp)
target_link_libraries(wallfollower ${catkin_LIBRARIES})
add_dependencies(wallfollower ${catkin_EXPORTED_TARGETS})

add_executable(turner src/turner.cpp)
target_link_libraries(turner ${catkin_LIBRARIES})
add_dependencies(turner ${catkin_EXPORTED_TARGETS})

add_executable(marcher src/marcher.cpp)
target_link_libraries(marcher ${catkin_LIBRARIES})
add_dependencies(marcher ${catkin_EXPORTED_TARGETS})
