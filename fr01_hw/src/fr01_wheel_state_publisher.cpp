#include <fr01_wheel_state_publisher.hpp>

Fr01WheelStatePublisher::Fr01WheelStatePublisher(ros::NodeHandle nh)
  : nh_(nh),
    rate_(100)
{
  ros::NodeHandle n("~");
  imcs01_right_port_.openPort(n.param<std::string>("imcs01_right_port_name", "/dev/urbtc0"));
  imcs01_left_port_.openPort(n.param<std::string>("imcs01_left_port_name", "/dev/urbtc1"));

  wheel_state_.velocity.resize(6);
  wheel_state_.position.resize(6);
  for (size_t i = 0; i < wheel_state_.velocity.size(); ++i) {
    wheel_state_.velocity[i] = 0.0;
    wheel_state_.position[i] = 0.0;  
  }

  wheel_state_.name.resize(6);
  wheel_state_.name[0] = "left_rear";
  wheel_state_.name[1] = "right_rear";
  wheel_state_.name[2] = "left_middle";
  wheel_state_.name[3] = "right_middle";
  wheel_state_.name[4] = "left_front";
  wheel_state_.name[5] = "right_front";
  wheel_state_pub_ = nh_.advertise<sensor_msgs::JointState>(n.param<std::string>("wheel_state_topic_name", "/wheel_states"), 10);
  
}

Fr01WheelStatePublisher::~Fr01WheelStatePublisher()
{
  
}

void Fr01WheelStatePublisher::run()
{
  static sensor_msgs::JointState right_wheels;
  static sensor_msgs::JointState left_wheels;
  
  while(nh_.ok())
    {
      imcs01_left_.update(&imcs01_left_port_);
      imcs01_right_.update(&imcs01_right_port_);
      
      right_wheels = imcs01_right_.getState();
      left_wheels  = imcs01_left_.getState();

      setState(right_wheels, left_wheels);

      wheel_state_.header.stamp = ros::Time::now();
      wheel_state_pub_.publish(wheel_state_);

      ros::spinOnce();
      rate_.sleep();
    }
}

void Fr01WheelStatePublisher::setState(sensor_msgs::JointState &right_wheels,
				sensor_msgs::JointState &left_wheels)
{
  // right is opposite of Left. So, right add minus sign
  for(int i = 0; i < 3; i++){
    wheel_state_.velocity[2*i] = -right_wheels.velocity[i];
    wheel_state_.position[2*i] = -right_wheels.position[i];
    wheel_state_.velocity[2*i+1] = left_wheels.velocity[i];
    wheel_state_.position[2*i+1] = left_wheels.position[i];
  }
}
