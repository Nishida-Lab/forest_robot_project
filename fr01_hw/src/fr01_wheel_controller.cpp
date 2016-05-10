#include <fr01_wheel_controller.hpp>

Fr01WheelController::Fr01WheelController(ros::NodeHandle nh, ros::NodeHandle n)
  : nh_(nh),
    rate_(100)
{
  ticks_since_target_ = 0;
  timeout_ticks_ = 4;

  wheel_cmd_.data.resize(6);
  wheel_vel_cmd_.velocity.resize(6);
  wheel_state_.velocity.resize(6);

  WheelControlPid pid_controller(80.0, 30.0, 0.0, 100, -100);
  for (size_t i = 0; i < wheel_cmd_.data.size(); ++i) {
    pid_controllers_.push_back(pid_controller);
  }

  wheel_pwm_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/motor_input", 10);

  wheel_vel_cmd_sub_ = nh_.subscribe<sensor_msgs::JointState>(n.param<std::string>("wheel_cmd_topic_name", "/wheel_vel_cmd"), 1, boost::bind(&Fr01WheelController::targetWheelVelCallback, this, _1));
  wheel_state_sub_ = nh_.subscribe<sensor_msgs::JointState>(n.param<std::string>("wheel_state_topic_name", "/wheel_states"), 1, boost::bind(&Fr01WheelController::stateWheelVelCallback, this, _1));

}

Fr01WheelController::~Fr01WheelController()
{

}

void Fr01WheelController::targetWheelVelCallback(const sensor_msgs::JointStateConstPtr& wheel_vel_cmd)
{
  for (size_t i = 0;  i < wheel_vel_cmd_.velocity.size(); ++ i) {
    wheel_vel_cmd_.velocity[i] = wheel_vel_cmd->velocity[i];
  }
  ticks_since_target_ = 0;
}

void Fr01WheelController::stateWheelVelCallback(const sensor_msgs::JointStateConstPtr& wheel_state)
{
  for(size_t i = 0; i < wheel_state_.velocity.size(); ++i){
    wheel_state_.velocity[i] = wheel_state->velocity[i];
  }
}


void Fr01WheelController::run()
{
  while(nh_.ok())
    {
      while(ticks_since_target_ < timeout_ticks_)
	{
	    for (size_t i = 0; i < wheel_cmd_.data.size(); ++i) {
	      wheel_cmd_.data[i] = (int)pid_controllers_[i].compute(wheel_state_.velocity[i], wheel_vel_cmd_.velocity[i]);
	    }
	    wheel_pwm_pub_.publish(wheel_cmd_);
	    rate_.sleep();
	    ticks_since_target_ += 1;
	    if(ticks_since_target_ == timeout_ticks_)
	      {
		for (size_t i = 0; i < wheel_cmd_.data.size(); ++i) {
		  wheel_cmd_.data[i] = 0;
		}
		wheel_pwm_pub_.publish(wheel_cmd_);
	      }
	}
      ros::spinOnce();
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
  return output_;
}
