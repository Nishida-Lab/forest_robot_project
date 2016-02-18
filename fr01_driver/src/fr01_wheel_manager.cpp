#include <fr01_wheel_manager.hpp>

Fr01WheelManager::Fr01WheelManager(ros::NodeHandle nh)
  : nh_(nh),
    rate_(100)
{
  ros::NodeHandle n("~");
  imcs01_right_port_.openPort(n.param<std::string>("imcs01_right_port_name", "/dev/urbtc0"));
  imcs01_left_port_.openPort(n.param<std::string>("imcs01_left_port_name", "/dev/urbtc1"));

  wheel_cmd_.data.resize(6);
  WheelControlPid pid_controller(80.0, 30.0, 0.0, 100, -100);
  for (size_t i = 0; i < wheel_cmd_.data.size(); ++i) {
    pid_controllers_.push_back(pid_controller);
  }

  wheel_pwm_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/motor_input", 10);

}

Fr01WheelManager::~Fr01WheelManager()
{
  
}

void Fr01WheelManager::controlWheelVelCallback(const sensor_msgs::JointStateConstPtr &wheel_vel_cmd)
{
  
}

void Fr01WheelManager::run()
{
  sensor_msgs::JointState right_wheels;
  sensor_msgs::JointState left_wheels;
  
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

void Fr01WheelManager::setState(sensor_msgs::JointState &right_wheels,
				sensor_msgs::JointState &left_wheels)
{
  for(int i = 0; i < 3; i++){
    wheel_state_.velocity[2*i] = right_wheels.velocity[i];
    wheel_state_.position[2*i] = right_wheels.position[i];
    wheel_state_.velocity[2*i+1] = left_wheels.velocity[i];
    wheel_state_.position[2*i+1] = left_wheels.velocity[i];
  }
}

WheelControlPid::WheelControlPid(double Kp, double Ki, double Kd, double max, double min)
    :Kp_(Kp), Ki_(Ki), Kd_(Kd), max_(max), min_(min), ITerm_(0)
{
  output_ = 0;
  sample_time_ = 0.01; //[s]
}

// input means controlled variable.
// target means reference input.
// output manipulated variable.
double WheelControlPid::compute(double input, double target)
{
  ros::Time now = ros::Time::now();
  ros::Duration timeChange = now - last_time_;
 
  if(timeChange.toSec() >= sample_time_)
    {
      double error = target - input;
      ITerm_ += Ki_ * error;
      if(ITerm_ > max_)
	{
	  ITerm_ = max_;
	}else if(ITerm_ < min_)
	{
	  ITerm_ = min_;
	}
      double dInput = input - last_input_;
    
      output_ = Kp_ * error + ITerm_ - Kd_ * dInput;

      if(output_ > max_){
	output_ = max_;
      }else if(output_ < min_)
	{
	  output_ = min_;
	}

      last_input_ = input;
      last_time_ = now;

    }
}
