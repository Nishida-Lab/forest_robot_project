#include <fr01_wheel_manager.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "fr01_wheel_manager_node");
  ROS_INFO("FR01 Wheel Manager");

  ros::NodeHandle nh;

  Fr01WheelManager wheel_manager(nh);
  wheel_manager.run();

  return 0;
}
