cmake_minimum_required(VERSION 2.8.3)
project(fr01_driver)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_filters
  hardware_interface
  transmission_interface
  controller_manager
  joint_limits_interface
  angles
  control_toolbox
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES trobo_control
  CATKIN_DEPENDS roscpp hardware_interface transmission_interface controller_manager
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fr01_control
  src/fr01_control.cpp
  src/fr01_interface/fr01_interface.cpp
  src/fr01_interface/fr01_wheel_interface.cpp
  src/fr01_interface/fr01_steer_interface.cpp
  )

add_executable(fr01_wheel_manager_node
  src/ixis_imcs01/serial.cpp
  src/ixis_imcs01/ixis_imcs01.cpp
  src/fr01_wheel_manager.cpp
  src/fr01_wheel_manager_node.cpp
  )

add_executable(fr01_driver_node
  src/fr01_driver_node.cpp
  src/fr01_driver.cpp
  src/Fr01Interface/Fr01Interface.cpp
  )

add_executable(fr01_controller_node
  src/fr01_controller_node.cpp
  src/fr01_controller.cpp
  )

target_link_libraries(fr01_driver_node
  ${catkin_LIBRARIES}
  )
target_link_libraries(fr01_controller_node
  ${catkin_LIBRARIES}
  )
target_link_libraries(fr01_control
  ${catkin_LIBRARIES}
  )
target_link_libraries(fr01_wheel_manager_node
  ${catkin_LIBRARIES}
  )

#############
## Testing ##
#############

find_package(roslaunch)
roslaunch_add_file_check(test/fr01_driver_test.xml)
