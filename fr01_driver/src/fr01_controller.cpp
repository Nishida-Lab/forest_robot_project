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
  gain_p_ = 1.0;
  motor_cmd_.data.resize(6);
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
    input_[i] += gain_p_*(wheel_joint_ctrl->velocity[i] - wheel_state->velocity[i]);
    motor_cmd_.data[i] = (int)input_[i];
  }
  motor_input_pub_.publish(motor_cmd_);
  

}
