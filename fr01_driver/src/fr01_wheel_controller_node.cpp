#include <fr01_wheel_controller.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "fr01_wheel_controller_node");
  ROS_INFO("FR01 Wheel Controller Node");

  ros::NodeHandle nh;

  Fr01WheelController wheel_controller(nh);
  wheel_controller.run();

  return 0;
}
