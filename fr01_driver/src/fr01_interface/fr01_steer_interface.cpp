#include <ros/ros.h>
#include <fr01_interface/fr01_steer_interface.hpp>

#include <angles/angles.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>


Fr01SteerInterface::Fr01SteerInterface()
{
  ros::NodeHandle n("~");
  n.getParam("angular_limit_max", angular_limit_max_);
  n.getParam("angular_limit_min", angular_limit_min_);
  n.getParam("linear_limit_max", linear_limit_max_);
  n.getParam("linear_limit_min", linear_limit_min_);
  n.param("two_steer_mode", two_steer_mode_, false);

  n.getParam("steer_joint_names", joint_names_);
  // Cleanup
  joint_pos_.clear();
  joint_vel_.clear();
  joint_eff_.clear();
  joint_pos_cmd_.clear();

  n_dof_ = joint_names_.size();
  
  joint_pos_.resize(n_dof_);
  joint_vel_.resize(n_dof_);
  joint_eff_.resize(n_dof_);
  joint_pos_cmd_.resize(n_dof_);

  // Hardware interfaces
  for (size_t i = 0; i < n_dof_; ++i) {
    hardware_interface::JointStateHandle state_handle(joint_names_[i],
						      &joint_pos_[i],
						      &joint_vel_[i],
						      &joint_eff_[i]);
    joint_state_interface_.registerHandle(state_handle);
    hardware_interface::JointHandle pos_handle(joint_state_interface_.getHandle(joint_names_[i]),
					       &joint_pos_cmd_[i]);
    joint_pos_interface_.registerHandle(pos_handle);

    ROS_DEBUG_STREAM("Registered joint '" << joint_names_[i] << " ' in the PositionJointInterface");
  }
  
  registerInterface(&joint_state_interface_);
  registerInterface(&joint_pos_interface_);

  // Position joint limits interface
  // TODO

  
}

void Fr01SteerInterface::read(ros::Time now, ros::Duration period)
{
  // TODO
}

void Fr01SteerInterface::write(ros::Time now, ros::Duration period)
{
  // TODO
}
