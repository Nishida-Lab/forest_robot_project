#include <ros/ros.h>
#include <fr01_driver.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fr01_driver_node");
  ROS_INFO("FR01 driver for ROS.");

  ros::NodeHandle nh;
  
  fr01_driver driver(nh);
  driver.run();

  return 0;
}
