#include <ros/ros.h>
#include <fr01_interface/fr01_interface.hpp>

#include <angles/angles.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>


Fr01Interface::Fr01Interface()
{
  ros::NodeHandle n("~");

  n.getParam("steer_joint_names", steer_joint_names_);
  n.getParam("wheel_joint_names", wheel_joint_names_);
 
  fr01_wheel_ptr_.reset(new Fr01WheelInterface(wheel_joint_names_));
  fr01_steer_ptr_.reset(new Fr01SteerInterface(steer_joint_names_));

  fr01_wheel_ptr_->register_interface(wheel_joint_state_interface_,
				     wheel_vel_joint_interface_);
  fr01_steer_ptr_->register_interface(steer_joint_state_interface_,
				      steer_pos_joint_interface_);
  
  registerInterface(&wheel_joint_state_interface_);
  registerInterface(&wheel_vel_joint_interface_);
  registerInterface(&steer_joint_state_interface_);
  registerInterface(&steer_pos_joint_interface_);

  // Position joint limits interface
  // TODO

  
}

void Fr01Interface::read(ros::Time now, ros::Duration period)
{
  // TODO
}

void Fr01Interface::write(ros::Time now, ros::Duration period)
{
  // TODO
}
