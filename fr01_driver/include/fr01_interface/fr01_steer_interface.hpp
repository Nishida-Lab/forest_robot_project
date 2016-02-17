#ifndef FR01_STEER_INTERFACE_H
#define FR01_STEER_INTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

class Fr01SteerInterface
{
 public:
  Fr01SteerInterface(std::vector<std::string> joint_names);
  void cleanup();
  void resize();
  void register_interface(hardware_interface::JointStateInterface &joint_state_interface,
			  hardware_interface::PositionJointInterface &pos_joint_interface);
 protected:
  unsigned int n_dof_;
  
  std::vector<std::string> joint_names_;
  std::vector<double> joint_pos_;
  std::vector<double> joint_vel_;
  std::vector<double> joint_eff_;
  std::vector<double> joint_pos_cmd_;  

};


#endif /* FR01_STEER_INTERFACE_H */
