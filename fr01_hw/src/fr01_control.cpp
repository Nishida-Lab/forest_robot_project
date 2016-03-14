#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <fr01_interface/fr01_interface.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fr01_control");
  ROS_INFO("FR01 control node");
  
  ros::NodeHandle nh;
  Fr01Interface fr01_interface;
  controller_manager::ControllerManager cm(&fr01_interface, nh);

  ros::Rate rate(1.0 / fr01_interface.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok()) {
    ros::Time now = fr01_interface.getTime();
    ros::Duration dt = fr01_interface.getPeriod();

    fr01_interface.read(now, dt);
    cm.update(now, dt);

    fr01_interface.write(now, dt);
    rate.sleep();
  }
  spinner.stop();
  

  return 0;
}
