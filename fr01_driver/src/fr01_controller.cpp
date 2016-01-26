#include <fr01_controller.hpp>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

fr01_controller::fr01_controller(ros::NodeHandle nh)
    : nh_(nh), rate_(100), wheel_state_sub_(nh_, "/wheel_state", 1),
      wheel_joint_ctrl_sub_(nh_, "/wheel_joint_ctrl", 1),
      sync_(MySyncPolicy(10), wheel_state_sub_, wheel_joint_ctrl_sub_)
{
  input_.resize(6);
  motor_cmd_.data.resize(6);
  PID pid_contoller(80.0, 30.0, 0.0, 100, -100);
  for (int i = 0; i < 6; ++i) {
    pid_controllers_.push_back(pid_contoller);
  }

  motor_input_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/motor_input", 10);
  sync_.registerCallback(boost::bind(&fr01_controller::control_cb, this, _1, _2));
}

fr01_controller::~fr01_controller()
{
}

void fr01_controller::run()
{
  while(nh_.ok())
  {
    //
    // TODO : calculateodometry(), tf
    //

    ros::spinOnce();
    rate_.sleep();
  }
}


void fr01_controller::control_cb(const sensor_msgs::JointStateConstPtr& wheel_state,
                                 const sensor_msgs::JointStateConstPtr& wheel_joint_ctrl)
{
  for (int i = 0; i < motor_cmd_.data.size(); ++i) {
    motor_cmd_.data[i] = (int)pid_controllers_[i].compute(-wheel_state->velocity[i],
                                                          wheel_joint_ctrl->velocity[i]);
  }
  motor_input_pub_.publish(motor_cmd_); 
}

PID::PID(double Kp, double Ki, double Kd, double max, double min)
    :Kp_(Kp), Ki_(Ki), Kd_(Kd), max_(max), min_(min), ITerm_(0)
{
  output_ = 0;
  sample_time_ = 0.01; //[s]
}

// input means controlled variable.
// target means reference input.
// output manipulated variable.
double  PID::compute(double input, double target)
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
