#ifndef FR01_CONTROLLER_H
#define FR01_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class fr01_controller
{
 public:
  fr01_controller(ros::NodeHandle nh);
  ~fr01_controller();
  void control_cb(const sensor_msgs::JointStateConstPtr& wheel_state,
                  const sensor_msgs::JointStateConstPtr& wheel_joint_ctrl);
  void run();
 private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher motor_input_pub_;
  message_filters::Subscriber<sensor_msgs::JointState> wheel_state_sub_;
  message_filters::Subscriber<sensor_msgs::JointState> wheel_joint_ctrl_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState, sensor_msgs::JointState> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync_;
  
  std_msgs::Int32MultiArray motor_cmd_;
};

#endif /* FR01_CONTROLLER_H */

