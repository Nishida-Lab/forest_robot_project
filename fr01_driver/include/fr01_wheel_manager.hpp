#ifndef FR01_WHEEL_MANAGER_H
#define FR01_WHEEL_MANAGER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <ixis_imcs01/ixis_imcs01.hpp>
#include <boost/thread.hpp>

class WheelControlPid;

class Fr01WheelManager
{
public:
  Fr01WheelManager(ros::NodeHandle nh);
 ~Fr01WheelManager();
  void controlWheelVelCallback(const sensor_msgs::JointStateConstPtr& wheel_vel_cmd);
  void setState(sensor_msgs::JointState &right_wheels,
		sensor_msgs::JointState &left_wheels);
  void run();
  
protected:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Subscriber wheel_cmd_sub_;
  ros::Publisher wheel_pwm_pub_;
  ros::Publisher wheel_state_pub_;
  
  std_msgs::Int32MultiArray wheel_cmd_;
  std::vector<WheelControlPid> pid_controllers_;
 
  IxisImcs01 imcs01_right_;
  IxisImcs01 imcs01_left_;
  SerialPort imcs01_right_port_;
  SerialPort imcs01_left_port_;

  sensor_msgs::JointState wheel_state_;

  boost::mutex access_mutex_;
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


#endif /* FR01_WHEEL_MANAGER_H */
