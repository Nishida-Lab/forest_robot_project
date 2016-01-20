#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <fr01_driver.hpp>

fr01_driver::fr01_driver(ros::NodeHandle nh)
    : nh_(nh), rate_(100)
{
  ros::NodeHandle n("~");
  steer_joint_ctrl_.name.resize(4);
  steer_joint_ctrl_.position.resize(4);
  steer_joint_ctrl_.velocity.resize(4);
  wheel_joint_ctrl_.name.resize(6);
  wheel_joint_ctrl_.position.resize(6);
  wheel_joint_ctrl_.velocity.resize(6);

  steer_joint_ctrl_.name[0] = "steer_front_right";
  steer_joint_ctrl_.name[1] = "steer_front_left";
  steer_joint_ctrl_.name[2] = "steer_rear_right";
  steer_joint_ctrl_.name[3] = "steer_rear_left";

  wheel_joint_ctrl_.name[0] = "wheel_front_right";
  wheel_joint_ctrl_.name[1] = "wheel_front_left";
  wheel_joint_ctrl_.name[2] = "wheel_middle_right";
  wheel_joint_ctrl_.name[3] = "wheel_middle_left";
  wheel_joint_ctrl_.name[4] = "wheel_rear_right";
  wheel_joint_ctrl_.name[5] = "wheel_rear_left";

  fr01_ = new Fr01Interface();
  
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
  steer_pub_ = nh_.advertise<sensor_msgs::JointState>("/steer_joint_ctrl", 10);
  wheel_pub_ = nh_.advertise<sensor_msgs::JointState>("/wheel_joint_ctrl", 10);
  cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(&fr01_driver::cmdVelReceived, this, _1));
}

fr01_driver::~fr01_driver()
{
  delete fr01_;
}

void fr01_driver::run()
{
  while(nh_.ok())
  {
    //
    // TODO : calculateodometry(), tf
    //

    ros::spinOnce();
    rate_.sleep();
  }
}

void fr01_driver::cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  fr01_->drive(cmd_vel->linear.x, cmd_vel->angular.z, wheel_joint_ctrl_, steer_joint_ctrl_);
  steer_joint_ctrl_.header.stamp = ros::Time::now();
  steer_pub_.publish(steer_joint_ctrl_);
  wheel_joint_ctrl_.header.stamp = ros::Time::now();
  wheel_pub_.publish(wheel_joint_ctrl_);
}
