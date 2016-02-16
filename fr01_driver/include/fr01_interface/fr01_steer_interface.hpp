#ifndef FR01_STEER_INTERFACE_H
#define FR01_STEER_INTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

#ifndef MIN
#define MIN(a,b) ((a < b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a > b) ? (a) : (b))
#endif
#ifndef NORMALIZE
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif

class Fr01SteerInterface
  : public hardware_interface::RobotHW
{
 public:
  Fr01SteerInterface();

  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.01); }
  
  void read(ros::Time time, ros::Duration period);

  void write(ros::Time time, ros::Duration period);
  
 protected:
  unsigned int n_dof_;
  
  bool two_steer_mode_;
  double angular_limit_max_;
  double angular_limit_min_;
  double linear_limit_max_;
  double linear_limit_min_;

  std::vector<std::string> transmission_names_;

  std::vector<std::string> joint_names_;
  std::vector<double> joint_pos_;
  std::vector<double> joint_vel_;
  std::vector<double> joint_eff_;
  std::vector<double> joint_pos_cmd_;
  
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface joint_pos_interface_;
  
};


#endif /* FR01_STEER_INTERFACE_H */
