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

  wheel_vel_sub_ = nh_.subscribe(n.param<std::string>("wheel_state_topic_name", "/wheel_states"), 100, &Fr01Interface::wheelStateCallback, this);
  steer_pos_sub_ = nh_.subscribe(n.param<std::string>("steer_state_topic_name", "/steer_states"), 100, &Fr01Interface::steerStateCallback, this);


  // Position joint limits interface
  // TODO  
}

void Fr01Interface::read(ros::Time now, ros::Duration period)
{
  {
    boost::mutex::scoped_lock(wheel_state_access_mutex_);
    fr01_wheel_ptr_->read(wheel_state_);
  }
  {
    boost::mutex::scoped_lock(steer_state_access_mutex_);
    fr01_steer_ptr_->read(steer_state_);
  }
}

void Fr01Interface::write(ros::Time now, ros::Duration period)
{
  fr01_wheel_ptr_->write();
  fr01_steer_ptr_->write();
}

void Fr01Interface::wheelStateCallback(const sensor_msgs::JointStateConstPtr& wheel_state)
{
  {
    boost::mutex::scoped_lock(wheel_state_access_mutex_);
    for (size_t i = 0; i < wheel_state_.name.size(); ++i) {
      wheel_state_.position[i] = wheel_state->position[i];
      wheel_state_.velocity[i] = wheel_state->velocity[i];
      wheel_state_.effort[i]   = wheel_state->effort[i];
    }
  }
}

void Fr01Interface::steerStateCallback(const sensor_msgs::JointStateConstPtr& steer_state)
{
  {
    boost::mutex::scoped_lock(steer_state_access_mutex_);
    for (size_t i = 0; i < steer_state_.name.size(); ++i) {
      steer_state_.position[i] = steer_state->position[i];
      steer_state_.velocity[i] = steer_state->velocity[i];
      steer_state_.effort[i]   = steer_state->effort[i];
    }
  }
}
