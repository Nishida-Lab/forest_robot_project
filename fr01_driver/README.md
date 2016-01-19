# FR01(Forest Robot 01) driver

This package contains follow two nodes. 
- fr01_driver_node
- fr01_wheel_controller(not yet)

## fr01_driver_node
* subscribe
  - /cmd_vel
  - /wheel_state
  - /steer_state

* publish
  - /wheel_joint_ctrl
  - /steer_joint_ctrl
  - /odom

## fr01_wheel_controller_node
* subscribe
  - /wheel_state
  - /wheel_joint_ctrl

* publish
  - /motor_input(topic name is undecided)

## Preparation
In order to run this package, we need follow packages.
Please clone these packages manually or use wstool.

* [ros_ixis_imcs01_driver_beta](https://github.com/AriYu/ros_ixis_imcs01_driver_beta)
* [ros-kondo-b3m-driver-beta](https://github.com/AriYu/ros-kondo-b3m-driver-beta)

## How to run
Not yet.
