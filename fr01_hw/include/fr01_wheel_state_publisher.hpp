#ifndef FR01_WHEEL_STATE_PUBLISHER_H
#define FR01_WHEEL_STATE_PUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <ixis_imcs01/ixis_imcs01.hpp>
#include <boost/thread.hpp>


class Fr01WheelStatePublisher
{
public:
  Fr01WheelStatePublisher(ros::NodeHandle nh);
 ~Fr01WheelStatePublisher();
  void setState(sensor_msgs::JointState &right_wheels,
		sensor_msgs::JointState &left_wheels);
  void run();
  
protected:
  ros::NodeHandle nh_;
  ros::Rate rate_;

  ros::Publisher wheel_state_pub_;
   
  IxisImcs01 imcs01_right_;
  IxisImcs01 imcs01_left_;
  SerialPort imcs01_right_port_;
  SerialPort imcs01_left_port_;

  sensor_msgs::JointState wheel_state_;

};

#endif /* FR01_WHEEL_STATE_PUBLISHER_H */
