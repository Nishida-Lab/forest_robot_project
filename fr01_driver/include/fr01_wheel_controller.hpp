#ifndef FR01_WHEEL_CONTROLLER_H
#define FR01_WHEEL_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class WheelControlPid;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState, sensor_msgs::JointState> MySyncPolicy;

class Fr01WheelController
{
public:
  Fr01WheelController(ros::NodeHandle nh);
 ~Fr01WheelController();
  void controlWheelVelCallback(const sensor_msgs::JointStateConstPtr& wheel_state,
			       const sensor_msgs::JointStateConstPtr& wheel_vel_cmd);
  void run();
  
protected:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Subscriber wheel_cmd_sub_;
  ros::Publisher wheel_pwm_pub_;
   
  std_msgs::Int32MultiArray wheel_cmd_;
  std::vector<WheelControlPid> pid_controllers_;

  message_filters::Subscriber<sensor_msgs::JointState> wheel_state_sub_;
  message_filters::Subscriber<sensor_msgs::JointState> wheel_vel_cmd_sub_;
  message_filters::Synchronizer<MySyncPolicy> sync_;
  
};

class WheelControlPid
{
 public:
  WheelControlPid(double Kp, double Ki, double Kd, double max, double min);
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


#endif /* FR01_WHEEL_CONTROLLER_H */
