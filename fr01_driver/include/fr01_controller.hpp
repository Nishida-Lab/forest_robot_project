#ifndef FR01_CONTROLLER_H
#define FR01_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class PID;

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
  std::vector<double> input_;
  std::vector<PID> pid_controllers_;
};

class PID
{
 public:
  PID(double Kp, double Ki, double Kd, double max, double min);
  double compute(double input, double target);
 private:
  double Kp_;
  double Ki_;
  double Kd_;
  double ITerm_; // Integral Term(積分項)
  double max_;
  double min_;
  double last_input_;
  double output_;
  ros::Time last_time_;
  double sample_time_;
};

#endif /* FR01_CONTROLLER_H */

