#ifndef FR01_WHEEL_INTERFACE_H
#define FR01_WHEEL_INTERFACE_H

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

class Fr01WheelInterface
  : public hardware_interface::RobotHW
{
 public:
  Fr01WheelInterface();

  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.01); }
  
  void read(ros::Time time, ros::Duration period);

  void write(ros::Time time, ros::Duration period);
  
 protected:
  unsigned int n_dof_;
  
  // 0 : front, 1 : middle, 2 : rear
  std::vector<double> tred_width_;
  
  // 0 : right front, 1 : left front, 2 : right rear, 3 : left rear
  std::vector<double> tred_length_;
  
  // 0 : right front, 1 : left front, 2 : right middle, 3 : left middle
  // 4 : right rear, 5 : left rear
  std::vector<double> wheel_diameters_;

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
  std::vector<double> joint_vel_cmd_;
  
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface joint_vel_interface_;
  
};

#endif /* FR01_WHEEL_INTERFACE_H */
