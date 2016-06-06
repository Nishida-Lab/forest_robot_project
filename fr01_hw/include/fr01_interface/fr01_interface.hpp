#ifndef FR01_INTERFACE_H
#define FR01_INTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

#include <fr01_interface/fr01_wheel_interface.hpp>
#include <fr01_interface/fr01_steer_interface.hpp>

#include <memory>
#include <boost/thread.hpp>

#ifndef MIN
#define MIN(a,b) ((a < b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a > b) ? (a) : (b))
#endif
#ifndef NORMALIZE
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif

class Fr01Interface
  : public hardware_interface::RobotHW
{
 public:
  Fr01Interface();

  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.01); }

  void read(ros::Time time, ros::Duration period);

  void write(ros::Time time, ros::Duration period);

  void wheelStateCallback(const sensor_msgs::JointStateConstPtr& wheel_state);

  void steerStateCallback(const sensor_msgs::JointStateConstPtr& steer_state);

  bool getWheelNames(ros::NodeHandle& nh, const std::string& steer_joint_name_param,
		     std::vector<std::string>& steer_names);

  bool getSteerNames(ros::NodeHandle& nh, const std::string& steer_joint_name_param,
		     std::vector<std::string>& steer_names);

 protected:
  ros::NodeHandle nh_;

  boost::shared_ptr<Fr01WheelInterface> fr01_wheel_ptr_;
  boost::shared_ptr<Fr01SteerInterface> fr01_steer_ptr_;

  std::vector<std::string> transmission_names_;
  std::vector<std::string> steer_joint_names_;
  std::vector<std::string> wheel_joint_names_;

  hardware_interface::JointStateInterface wheel_joint_state_interface_;
  hardware_interface::VelocityJointInterface wheel_vel_joint_interface_;

  hardware_interface::JointStateInterface steer_joint_state_interface_;
  hardware_interface::PositionJointInterface steer_pos_joint_interface_;

  hardware_interface::JointStateInterface joint_state_interface_;

  ros::Subscriber steer_pos_sub_;
  ros::Subscriber wheel_vel_sub_;

  ros::Publisher steer_pos_pub_;

  // sensor_msgs::JointState steer_state_;
  // sensor_msgs::JointState wheel_state_;

  sensor_msgs::JointStateConstPtr steer_state_ptr_;
  sensor_msgs::JointStateConstPtr wheel_state_ptr_;

  ros::Time steer_state_timestamp_;
  ros::Time wheel_state_timestamp_;

  boost::mutex wheel_state_access_mutex_;
  boost::mutex steer_state_access_mutex_;
};

#endif /* FR01_INTERFACE_H */
