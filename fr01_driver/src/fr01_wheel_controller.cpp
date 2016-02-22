#include <fr01_wheel_controller.hpp>

Fr01WheelController::Fr01WheelController(ros::NodeHandle nh)
  : nh_(nh),
    rate_(100),
    sync_(MySyncPolicy(10), wheel_state_sub_, wheel_vel_cmd_sub_)
{
  ros::NodeHandle n("~");

  wheel_cmd_.data.resize(6);

  
  WheelControlPid pid_controller(80.0, 30.0, 0.0, 100, -100);
  for (size_t i = 0; i < wheel_cmd_.data.size(); ++i) {
    pid_controllers_.push_back(pid_controller);
  }

  wheel_pwm_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/motor_input", 10);

  sync_.registerCallback(boost::bind(&Fr01WheelController::controlWheelVelCallback, this, _1, _2));
}

Fr01WheelController::~Fr01WheelController()
{
  
}

void Fr01WheelController::controlWheelVelCallback(const sensor_msgs::JointStateConstPtr& wheel_state, const sensor_msgs::JointStateConstPtr &wheel_vel_cmd)
{
  for (size_t i = 0; i < wheel_cmd_.data.size(); ++i) {
    wheel_cmd_.data[i] = (int)pid_controllers_[i].compute(wheel_state->velocity[i], wheel_vel_cmd->velocity[i]);
  }
  wheel_pwm_pub_.publish(wheel_cmd_);
}

void Fr01WheelController::run()
{
  sensor_msgs::JointState right_wheels;
  sensor_msgs::JointState left_wheels;
  
  while(nh_.ok())
    {
      ros::spinOnce();
      rate_.sleep();
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
