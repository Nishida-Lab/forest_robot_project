#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <ixis_imcs01/serial.hpp>
#include <ixis_imcs01/ixis_imcs01.hpp>
#include <string>

#include <boost/thread.hpp>

IxisImcs01::IxisImcs01()
      : delta_encoder_time_(0), last_encoder_time_(0.0)
  {
    encoder_counts_.resize(3);
    last_encoder_counts_.resize(3);
    delta_encoder_counts_.resize(3);
    state_.position.resize(3);
    state_.velocity.resize(3);
    state_.name.resize(3);
    state_.effort.resize(3);
    
    for (int i = 0; i < 3; ++i) {
      encoder_counts_[i] = 0;
      last_encoder_counts_[i] = 0;
      delta_encoder_counts_[i] = -1;
      state_.position[i] = 0.0;
      state_.velocity[i] = 0.0;
    }

  }

int IxisImcs01::update(SerialPort *port)
{
  if(read(port->fd_, &received_data_, sizeof(received_data_)) != sizeof(received_data_)){
    ROS_WARN_STREAM("IxisImcs01::update() read error");
    return -1;
  }else{
    parseEncoderCounts();
    calculateAngularPosition();
    calculateAngularVelocity();
    return 0;
  }
}

int IxisImcs01::parseEncoderCounts()
{
  for (int i = 0; i < encoder_counts_.size(); ++i) {
    encoder_counts_[i] = (int)received_data_.ct[i+1];
  }
  delta_encoder_time_ = (double)(received_data_.time) - last_encoder_time_;

  if(delta_encoder_time_ < 0){
    delta_encoder_time_ = 65535 - (last_encoder_time_ - received_data_.time);
  }
  delta_encoder_time_ = delta_encoder_time_ / 1000.0; // [ms] -> [s]
  last_encoder_time_ = (double)(received_data_.time);
    
  for(int i = 0; i < delta_encoder_counts_.size(); ++i){
    if(delta_encoder_counts_[i] == -1 
       || encoder_counts_[i] == last_encoder_counts_[i]){ // First time.

      delta_encoder_counts_[i] = 0;

    }else{
      delta_encoder_counts_[i] = encoder_counts_[i] - last_encoder_counts_[i];

      // checking imcs01 counter overflow.
      if(delta_encoder_counts_[i] > ROBOT_MAX_ENCODER_COUNTS/10){
	delta_encoder_counts_[i] = delta_encoder_counts_[i] - ROBOT_MAX_ENCODER_COUNTS;
      }
      if(delta_encoder_counts_[i] < -ROBOT_MAX_ENCODER_COUNTS/10){
	delta_encoder_counts_[i] = delta_encoder_counts_[i] + ROBOT_MAX_ENCODER_COUNTS;
      }
    }
    last_encoder_counts_[i] = encoder_counts_[i];
  }

  return 0;
}

// edge evaluation     : 4(quad)
// gear reduction rate : 50
// encoder             : 100 pulse / motor axis revolution
// As a result         : 20000 pulse / wheel revolution
// angular positon     : (gained pulse / 20000) * \pi
//                       => gained pulse * 0.00015707963
void IxisImcs01::calculateAngularPosition()
{
  for(int i = 0; i < 3; i++){
    state_.position[i] += delta_encoder_counts_[i]*0.000157;
  }
}

void IxisImcs01::calculateAngularVelocity()
{
   for(int i = 0; i < 3; i++){
      state_.velocity[i] = (delta_encoder_counts_[i]*0.0000157/delta_encoder_time_);
    }
}

sensor_msgs::JointState IxisImcs01::getState()
{
  return state_;
}
