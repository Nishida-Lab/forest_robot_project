#include <fr01_controller.hpp>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

fr01_controller::fr01_controller(ros::NodeHandle nh)
    : nh_(nh), rate_(100), wheel_state_sub_(nh_, "/wheel_state", 1),
      wheel_joint_ctrl_sub_(nh_, "/wheel_joint_ctrl", 1),
      sync_(wheel_state_sub_, wheel_joint_ctrl_sub_, 10)
{
  motor_input_.name.resize(6);
  motor_input_.position.resize(6);
  motor_input_.velocity.resize(6);
  
  motor_input_pub_ = nh_.advertise<sensor_msgs::JointState>("/motor_input", 1);
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
  double gain_p = 1.0;
  double input = 0;
  for (int i = 0; i < motor_input_.name.size(); ++i) {
    input = gain_p*(wheel_joint_ctrl->velocity[i] - wheel_state->velocity[i]);
    motor_input_.position[i] = sgn(input);
    motor_input_.velocity[i] = fabs(input);
  }
  motor_input_.header.stamp = ros::Time::now();
  motor_input_pub_.publish(motor_input_);
}
