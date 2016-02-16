#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <fr01_interface/fr01_wheel_interface.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fr01_control");
  ROS_INFO("FR01 control node");
  
  ros::NodeHandle nh;
  Fr01WheelInterface wheel_interface;
  controller_manager::ControllerManager cm(&wheel_interface, nh);

  ros::Rate rate(1.0 / wheel_interface.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok()) {
    ros::Time now = wheel_interface.getTime();
    ros::Duration dt = wheel_interface.getPeriod();

    wheel_interface.read(now, dt);
    cm.update(now, dt);

    wheel_interface.write(now, dt);
    rate.sleep();
  }
  spinner.stop();
  

  return 0;
}
