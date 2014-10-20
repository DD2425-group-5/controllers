cmake_minimum_required(VERSION 2.8.3)
project(motor_controller2)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs kdl_conversions)
find_package(Boost REQUIRED COMPONENTS random)
find_package(orocos_kdl REQUIRED)

find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

catkin_package(
  DEPENDS orocos_kdl
  CATKIN_DEPENDS roscpp std_msgs kdl_conversions
  INCLUDE_DIRS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

link_directories(${orocos_kdl_LIBRARY_DIRS})

add_executable(motor_tester src/motorTester.cpp)
target_link_libraries(motor_tester ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})
install(TARGETS motor_tester
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



add_executable(motor_controller2 src/motorController2.cpp)
target_link_libraries(motor_controller2 ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})
install(TARGETS motor_controller2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

