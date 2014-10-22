cmake_minimum_required(VERSION 2.8.3)
project(blob_follow_controller)

find_package(catkin REQUIRED roscpp std_msgs blobdetection rosutil)

catkin_package(
   CATKIN_DEPENDS roscpp std_msgs blobdetection rosutil
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(blob_follower src/blobfollower.cpp)
target_link_libraries(blob_follower ${catkin_LIBRARIES} rosutil)
add_dependencies(blob_follower blobfollower_cpp)
