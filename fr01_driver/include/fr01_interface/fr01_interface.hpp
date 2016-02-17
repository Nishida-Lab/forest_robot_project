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
 
 protected:

  std::shared_ptr<Fr01WheelInterface> fr01_wheel_ptr_;
  std::shared_ptr<Fr01SteerInterface> fr01_steer_ptr_;
  
  std::vector<std::string> transmission_names_;
  std::vector<std::string> steer_joint_names_;
  std::vector<std::string> wheel_joint_names_;

  hardware_interface::JointStateInterface wheel_joint_state_interface_;
  hardware_interface::VelocityJointInterface wheel_vel_joint_interface_;

  hardware_interface::JointStateInterface steer_joint_state_interface_;
  hardware_interface::PositionJointInterface steer_pos_joint_interface_;

  ros::Subscriber kondo_b3m_subscriber_;
  ros::Subscriber ixis_imcs_subscriber_;

  ros::Publisher kondo_b3m_publisher_;
  ros::Publisher ixis_imcs_subscriber_;
  
};

#endif /* FR01_INTERFACE_H */
