#ifndef FR01_WHEEL_CONTROLLER_H
#define FR01_WHEEL_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>

class WheelControlPid;

class Fr01WheelController
{
public:
  Fr01WheelController(ros::NodeHandle nh, ros::NodeHandle n);
 ~Fr01WheelController();
  void targetWheelVelCallback(const sensor_msgs::JointStateConstPtr& wheel_vel_cmd);
  void stateWheelVelCallback(const sensor_msgs::JointStateConstPtr& wheel_state);
  void run();

protected:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Subscriber wheel_cmd_sub_;
  ros::Publisher wheel_pwm_pub_;

  std_msgs::Int32MultiArray wheel_cmd_;
  sensor_msgs::JointState wheel_state_;
  sensor_msgs::JointState wheel_vel_cmd_;
  std::vector<WheelControlPid> pid_controllers_;

  int ticks_since_target_;
  int timeout_ticks_;
  ros::Subscriber wheel_vel_cmd_sub_;
  ros::Subscriber wheel_state_sub_;

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
  double previous_error;
  double integral_;
  double derivative_;
  double max_;
  double min_;
  double last_input_;
  double output_;
  ros::Time last_time_;
  double sample_time_;
};


#endif /* FR01_WHEEL_CONTROLLER_H */
