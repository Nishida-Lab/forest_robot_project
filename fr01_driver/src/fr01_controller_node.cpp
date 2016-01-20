#include <fr01_controller.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fr01_controller_node");
  ROS_INFO("FR01 Controller for ROS.");

  ros::NodeHandle nh;

  fr01_controller controller(nh);
  controller.run();

  return 0;
}
