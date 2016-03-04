#include <fr01_wheel_state_publisher.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "fr01_wheel_state_publisher_node");
  ROS_INFO("FR01 Wheel State Publisher");

  ros::NodeHandle nh;

  Fr01WheelStatePublisher wheel_state_publisher(nh);
  wheel_state_publisher.run();

  return 0;
}
