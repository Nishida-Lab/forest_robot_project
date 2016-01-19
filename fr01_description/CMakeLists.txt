cmake_minimum_required(VERSION 2.8.3)
project(fr01_description)
find_package(catkin REQUIRED)

find_package(roslaunch)
roslaunch_add_file_check(test/fr01_launch_test.xml)
