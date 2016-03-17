#include <ros/ros.h>
#include <Fr01Interface/Fr01Interface.hpp>

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

}
Fr01Interface::~Fr01Interface()
{

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
// wheel
// 0 : left rear,   1 : right rear, 
// 2 : left middle, 3 : right middle, 
// 4 : left front,  5 : right front
void Fr01Interface::drive(double linear_speed, double angular_speed, 
                          sensor_msgs::JointState& wheel_input,
                          sensor_msgs::JointState& steer_input,
                          bool isPivotTurn=false)
{
  if(!isPivotTurn)
  {
    angular_speed = MAX(angular_speed, angular_limit_min_);
    angular_speed = MIN(angular_speed, angular_limit_max_);
    linear_speed  = MAX(linear_speed, linear_limit_min_);
    linear_speed  = MIN(linear_speed, linear_limit_max_);

    if(two_steer_mode_)
    {
      steer_input.position[2] = atan2(2.0*(tred_length_[2]+tred_length_[2]/2.0)*tan(angular_speed),
                                      2.0*(tred_length_[2]+tred_length_[2]/2.0)-2.0*tred_width_[2]*tan(angular_speed));
      steer_input.position[3] = atan2(2.0*(tred_length_[3]+tred_length_[3]/2.0)*tan(angular_speed),
                                      2.0*(tred_length_[3]+tred_length_[3]/2.0)+2.0*tred_width_[2]*tan(angular_speed));
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
        wheel_input.velocity[2] = (sin(angular_speed)/tan(steer_input.position[2]))*linear_speed;
        wheel_input.velocity[3] = (sin(angular_speed)/tan(steer_input.position[3]))*linear_speed;
        // right
        wheel_input.velocity[5] = (sin(angular_speed)/sin(steer_input.position[3]))*linear_speed;
        wheel_input.velocity[1] = wheel_input.velocity[3];
        // left
        wheel_input.velocity[4] = (sin(angular_speed)/sin(steer_input.position[2]))*linear_speed;
        wheel_input.velocity[0] = wheel_input.velocity[2];
      }
    }
    else
    {
      // Using four steer
      steer_input.position[2] = atan2(2.0*tred_length_[2]*tan(angular_speed),
                                      2.0*tred_length_[2]-2.0*tred_width_[2]*tan(angular_speed));
      steer_input.position[3] = atan2(2.0*tred_length_[3]*tan(angular_speed),
                                      2.0*tred_length_[3]+2.0*tred_width_[2]*tan(angular_speed));
      steer_input.position[0] = -steer_input.position[3];
      steer_input.position[1] = -steer_input.position[2];
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
        wheel_input.velocity[5] = (sin(angular_speed)/sin(steer_input.position[3]))*linear_speed;
        wheel_input.velocity[1] = wheel_input.velocity[5];
        // left
        wheel_input.velocity[4] = (sin(angular_speed)/sin(steer_input.position[2]))*linear_speed;
        wheel_input.velocity[0] = wheel_input.velocity[4];
        // middle
        wheel_input.velocity[2] = (sin(angular_speed)/tan(steer_input.position[2]))*linear_speed;
        wheel_input.velocity[3] = (sin(angular_speed)/tan(steer_input.position[3]))*linear_speed;
      }
    }    
  }else // pivot turn
  {
    //
    // TODO
    //
  }
  


}
