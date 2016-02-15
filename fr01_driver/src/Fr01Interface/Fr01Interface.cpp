#include <ros/ros.h>
#include <Fr01Interface/Fr01Interface.hpp>

#include <angles/angles.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <pluginlib/class_list_macros.h>

Fr01Interface::Fr01Interface()
{
  ros::NodeHandle n("~");
  n.getParam("tred_width", tred_width_);
  n.getParam("tred_length", tred_length_);
  n.getParam("wheel_diameters", wheel_diameters_);
  n.getParam("angular_limit_max", angular_limit_max_);
  n.getParam("angular_limit_min", angular_limit_min_);
  n.getParam("linear_limit_max", linear_limit_max_);
  n.getParam("linear_limit_min", linear_limit_min_);
  n.param("two_steer_mode", two_steer_mode_, false);

  n.getParam("wheel_joint_names", wheel_joint_names_);
  n.getParam("steer_joint_names", steer_joint_names_);
  // Cleanup
  wheel_joint_pos_.clear();
  wheel_joint_vel_.clear();
  wheel_joint_eff_.clear();
  wheel_joint_vel_cmd_.clear();
  steer_joint_pos_.clear();
  steer_joint_vel_.clear();
  steer_joint_eff_.clear();
  steer_joint_pos_cmd_.clear();

  wheel_n_dof_ = wheel_joint_names_.size();
  steer_n_dof_ = steer_joint_names_.size();
  
  wheel_joint_pos_.resize(wheel_n_dof_);
  wheel_joint_vel_.resize(wheel_n_dof_);
  wheel_joint_eff_.resize(wheel_n_dof_);
  wheel_joint_vel_cmd_.resize(wheel_n_dof_);
  steer_joint_pos_.resize(steer_n_dof_);
  steer_joint_vel_.resize(steer_n_dof_);
  steer_joint_eff_.resize(steer_n_dof_);
  steer_joint_pos_cmd_.resize(steer_n_dof_);

  // Hardware interfaces
  for (size_t i = 0; i < wheel_n_dof_; ++i) {
    wheel_joint_state_interface_.registerHandle(JointStateHandle(wheel_joint_names_[i],
								 &wheel_joint_pos_[i],
								 &wheel_joint_vel_[i],
								 &wheel_joint_eff_[i]));
    wheel_joint_vel_interface_.registerHandle(JointHandle(wheel_joint_state_interface_.getHandle(wheel_joint_names_[i]),
							&wheel_joint_vel_cmd_[i]));

    ROS_DEBUG_STREAM("Registered joint '", << wheel_joint_names_[i], " ' in the PositionJointInterface");
  }
  for (size_t i = 0; i < steer_n_dof_; ++i) {
    steer_joint_state_interface_.registerHandle(JointStateHandle(steer_joint_names_[i],
								 &steer_joint_pos_[i],
								 &steer_joint_vel_[i],
								 &steer_joint_eff_[i]));
    steer_joint_pos_interface_.registerHandle(JointHandle(steer_joint_state_interface_.getHandle(steer_joint_names_[i]),
							&steer_joint_pos_cmd_[i]));

    ROS_DEBUG_STREAM("Registered joint '", << steer_joint_names_[i], " ' in the PositionJointInterface");
  }
  
  registerInterface(&wheel_joint_state_interface_);
  registerInterface(&steer_joint_state_interface_);
  registerInterface(&wheel_joint_vel_interface_);
  registerInterface(&steer_joint_pos_interface_);

  // Position joint limits interface
  // TODO
  
}
Fr01Interface::~Fr01Interface()
{

}

void Fr01Interface::read()
{
  // TODO
}

void Fr01Interface::write()
{
  // TODO
}

void Fr01Interface::calculateOdometry(const sensor_msgs::JointStateConstPtr& wheel_state,
                                      const sensor_msgs::JointStateConstPtr& steer_state)
{

}

void Fr01Interface::setParams(std::vector<double> wheel_diameters,
                              std::vector<double> tred_width)
{
  for (int i = 0; i < wheel_diameters_.size(); ++i) {
    wheel_diameters_[i] = wheel_diameters[i];
  }
  for (int i = 0; i < tred_width_.size(); ++i) {
    tred_width_[i] = tred_width[i];
  }
}

// linear_speed : [m/s], angular_speed : [rad/s]
void Fr01Interface::drive(double linear_speed, double angular_speed, 
                          sensor_msgs::JointState& wheel_input,
                          sensor_msgs::JointState& steer_input,
                          bool isPivotTurn=false)
{
  if(!isPivotTurn)
  {
    if(two_steer_mode_)
    {
      steer_input.position[2] = atan2(2.0*(tred_length_[2]+tred_length_[2]/2.0)*tan(angular_speed),
                                      2.0*(tred_length_[2]+tred_length_[2]/2.0)+2.0*tred_width_[2]*tan(angular_speed));
      steer_input.position[3] = atan2(2.0*(tred_length_[3]+tred_length_[3]/2.0)*tan(angular_speed),
                                      2.0*(tred_length_[3]+tred_length_[3]/2.0)-2.0*tred_width_[2]*tan(angular_speed));
      steer_input.position[0] = 0;
      steer_input.position[1] = 0;

      if(angular_speed == 0.0)
      {
        for (size_t i = 0; i < 6; ++i)
        {
          wheel_input.velocity[i] = linear_speed;
        }
      }
      else
      {
        // middle
        wheel_input.velocity[2] = (sin(angular_speed)/tan(steer_input.position[3]))*linear_speed;
        wheel_input.velocity[3] = (sin(angular_speed)/tan(steer_input.position[2]))*linear_speed;
        // right
        wheel_input.velocity[5] = (sin(angular_speed)/sin(steer_input.position[2]))*linear_speed;
        wheel_input.velocity[1] = wheel_input.velocity[3];
        // left
        wheel_input.velocity[4] = (sin(angular_speed)/sin(steer_input.position[3]))*linear_speed;
        wheel_input.velocity[0] = wheel_input.velocity[2];
      }

    }
    else
    {
      // Using four steer
      steer_input.position[2] = atan2(2.0*tred_length_[2]*tan(angular_speed),
                                      2.0*tred_length_[2]+2.0*tred_width_[2]*tan(angular_speed));
      steer_input.position[3] = atan2(2.0*tred_length_[3]*tan(angular_speed),
                                      2.0*tred_length_[3]-2.0*tred_width_[2]*tan(angular_speed));

      steer_input.position[0] = -steer_input.position[3];
      steer_input.position[1] = -steer_input.position[2];

      // wheel
      // 0 : left rear,   1 : right rear, 
      // 2 : left middle, 3 : right middle, 
      // 4 : left front,  5 : right front
      if(angular_speed == 0.0)
      {
        for (size_t i = 0; i < 6; ++i)
        {
          wheel_input.velocity[i] = linear_speed;
        }
      }
      else
      {
        // right
        wheel_input.velocity[5] = (sin(angular_speed)/sin(steer_input.position[2]))*linear_speed;
        wheel_input.velocity[1] = wheel_input.velocity[5];
        // left
        wheel_input.velocity[4] = (sin(angular_speed)/sin(steer_input.position[3]))*linear_speed;
        wheel_input.velocity[0] = wheel_input.velocity[4];
        // middle
        wheel_input.velocity[2] = (sin(angular_speed)/tan(steer_input.position[3]))*linear_speed;
        wheel_input.velocity[3] = (sin(angular_speed)/tan(steer_input.position[2]))*linear_speed;
      }
    }    
  }else // pivot turn
  {
    //
    // TODO
    //
  }
  
  for (size_t i = 0; i < steer_input.position.size(); ++i) {
    steer_input.position[i] = MAX(steer_input.position[i], angular_limit_min_);
    steer_input.position[i] = MIN(steer_input.position[i], angular_limit_max_);
  }
  for (size_t i = 0; i < wheel_input.velocity.size(); ++i) {
    wheel_input.velocity[i] = MAX(wheel_input.velocity[i], linear_limit_min_);
    wheel_input.velocity[i] = MIN(wheel_input.velocity[i], linear_limit_max_);
  }

}
