#ifndef IXIS_IMCS01_H_
#define IXIS_IMCS01_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include "serial.hpp"

#define ROBOT_MAX_ENCODER_COUNTS 65535

class IxisImcs01{
 public:
  struct uin received_data_;
  std::vector<int> encoder_counts_;
  std::vector<int> last_encoder_counts_;
  std::vector<int> delta_encoder_counts_;
  double delta_encoder_time_;
  double last_encoder_time_;
  sensor_msgs::JointState state_;
  IxisImcs01();

  int update(SerialPort *port);
  int parseEncoderCounts();
  void calculateAngularPosition();
  void calculateAngularVelocity();
  sensor_msgs::JointState getState();
};


#endif
