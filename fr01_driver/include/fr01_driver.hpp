#ifndef FR01_DRIVER_H
#define FR01_DRIVER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Fr01Interface/Fr01Interface.hpp>

class fr01_driver
{
 public:
  fr01_driver(ros::NodeHandle nh);
  ~fr01_driver();
  void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel);
  void run();
 private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher odom_pub_;
  ros::Publisher steer_pub_;
  ros::Publisher wheel_pub_;
  ros::Subscriber cmd_vel_sub_;
  Fr01Interface *fr01_;
  sensor_msgs::JointState steer_joint_ctrl_;
  sensor_msgs::JointState wheel_joint_ctrl_;
};


#endif /* FR01_DRIVER_H */
