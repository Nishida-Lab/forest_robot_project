#include <ros/ros.h>
#include <fr01_interface/fr01_steer_interface.hpp>

#include <angles/angles.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>


Fr01SteerInterface::Fr01SteerInterface(std::vector<std::string> joint_names)
  : joint_names_(joint_names)
{
  ros::NodeHandle n("~");
  n_dof_ = joint_names_.size();
  this->cleanup();
  this->resize();

  commands_.name = joint_names_;

  steer_pos_pub_ = nh_.advertise<sensor_msgs::JointState>(n.param<std::string>("steer_cmd_topic_name", "/steer_pos_cmd"), 10);
}

void Fr01SteerInterface::register_interface(hardware_interface::JointStateInterface &joint_state_interface,
					    hardware_interface::PositionJointInterface &pos_joint_interface)
{
  // Hardware interfaces
  for (size_t i = 0; i < n_dof_; ++i) {
    hardware_interface::JointStateHandle state_handle(joint_names_[i],
						      &joint_pos_[i],
						      &joint_vel_[i],
						      &joint_eff_[i]);
    joint_state_interface.registerHandle(state_handle);
    hardware_interface::JointHandle pos_handle(joint_state_interface.getHandle(joint_names_[i]),
					       &joint_pos_cmd_[i]);
    pos_joint_interface.registerHandle(pos_handle);

    ROS_INFO_STREAM("Registered joint '" << joint_names_[i] << " ' in the PositionJointInterface");
  }

}

void Fr01SteerInterface::cleanup()
{
  // Cleanup
  joint_pos_.clear();
  joint_vel_.clear();
  joint_eff_.clear();
  joint_pos_cmd_.clear();
  commands_.velocity.clear();
  commands_.position.clear();
  commands_.effort.clear();
}

void Fr01SteerInterface::resize()
{
  joint_pos_.resize(n_dof_);
  joint_vel_.resize(n_dof_);
  joint_eff_.resize(n_dof_);
  joint_pos_cmd_.resize(n_dof_);
  commands_.velocity.resize(n_dof_);
  commands_.position.resize(n_dof_);
  commands_.effort.resize(n_dof_);
}

void Fr01SteerInterface::write()
{
  commands_.position = joint_pos_cmd_;
  steer_pos_pub_.publish(commands_);
}

void Fr01SteerInterface::read(const sensor_msgs::JointState& state)
{
  for (size_t i = 0; i < n_dof_; ++i) {
    joint_pos_[i] = state.position[i];
    joint_vel_[i] = state.velocity[i];
    joint_eff_[i] = state.effort[i];
  }
}
