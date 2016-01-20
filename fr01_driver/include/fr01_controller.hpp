#ifndef FR01_CONTROLLER_H
#define FR01_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

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
  message_filters::TimeSynchronizer<sensor_msgs::JointState, sensor_msgs::JointState> sync_;
  
  sensor_msgs::JointState motor_input_;
};

#endif /* FR01_CONTROLLER_H */

