cmake_minimum_required(VERSION 2.8.3)
project(fr01_gazebo)
find_package(catkin REQUIRED)
catkin_package()

find_package(roslaunch)
roslaunch_add_file_check(test/fr01_empty_world_launch_test.xml)
