cmake_minimum_required(VERSION 2.8.3)
project(fr01_lum_slam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fr01_ndt_mapping
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS})


# add_executable(lum_slam_node src/lum_slam_node.cpp src/lum_slam.cpp)
# target_link_libraries(lum_slam_node ${catkin_LIBRARIES})

add_executable(lum_slam_replay_node src/lum_slam_replay_node.cpp src/lum_slam.cpp)
target_link_libraries(lum_slam_replay_node ${catkin_LIBRARIES})
