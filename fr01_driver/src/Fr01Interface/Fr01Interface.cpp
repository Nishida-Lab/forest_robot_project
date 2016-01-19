#include <ros/ros.h>
#include <Fr01Interface/Fr01Interface.hpp>


Fr01Interface::Fr01Interface()
{
  ros::NodeHandle n("~");
  n.getParam("tred_width", tred_width_);
  n.getParam("tred_length", tred_length_);
  n.getParam("wheel_diameters", wheel_diameters_);
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
void Fr01Interface::drive(double linear_speed, double angular_speed, 
                           sensor_msgs::JointState& wheel_input,
                           sensor_msgs::JointState& steer_input)
{
  if(fabs(linear_speed) < 0.1)
  {
    if(angular_speed > 0)
    {
      // steering
      steer_input.position[0] = atan2(2.0*tred_length_[0]*tan(angular_speed),
                                      2.0*tred_length_[0]+tred_width_[0]*tan(angular_speed));
      steer_input.position[1] = atan2(2.0*tred_length_[1]*tan(angular_speed),
                                      2.0*tred_length_[1]-tred_width_[0]*tan(angular_speed));
      steer_input.position[2] = -atan2(2.0*tred_length_[2]*tan(angular_speed),
                                       2.0*tred_length_[2]+tred_width_[2]*tan(angular_speed));
      steer_input.position[3] = -atan2(2.0*tred_length_[3]*tan(angular_speed),
                                       2.0*tred_length_[3]-tred_width_[2]*tan(angular_speed));
      
      // wheel
      wheel_input.velocity[0] = linear_speed + (tred_width_[0]/2.0)*angular_speed;
      wheel_input.velocity[1] = linear_speed - (tred_width_[0]/2.0)*angular_speed;
      wheel_input.velocity[2] = linear_speed + (tred_width_[1]/2.0)*angular_speed;
      wheel_input.velocity[3] = linear_speed - (tred_width_[1]/2.0)*angular_speed;
      wheel_input.velocity[4] = linear_speed + (tred_width_[2]/2.0)*angular_speed;
      wheel_input.velocity[5] = linear_speed - (tred_width_[2]/2.0)*angular_speed;
    }
    else
    {
      steer_input.position[0] = atan2(2.0*tred_length_[0]*tan(angular_speed),
                                      2.0*tred_length_[0]-tred_width_[0]*tan(angular_speed));
      steer_input.position[1] = atan2(2.0*tred_length_[1]*tan(angular_speed),
                                      2.0*tred_length_[1]+tred_width_[0]*tan(angular_speed));
      steer_input.position[2] = -atan2(2.0*tred_length_[2]*tan(angular_speed),
                                       2.0*tred_length_[2]-tred_width_[2]*tan(angular_speed));
      steer_input.position[3] = -atan2(2.0*tred_length_[3]*tan(angular_speed),
                                       2.0*tred_length_[3]+tred_width_[2]*tan(angular_speed));

      wheel_input.velocity[0] = linear_speed - (tred_width_[0]/2.0)*angular_speed;
      wheel_input.velocity[1] = linear_speed + (tred_width_[0]/2.0)*angular_speed;
      wheel_input.velocity[2] = linear_speed - (tred_width_[1]/2.0)*angular_speed;
      wheel_input.velocity[3] = linear_speed + (tred_width_[1]/2.0)*angular_speed;
      wheel_input.velocity[4] = linear_speed - (tred_width_[2]/2.0)*angular_speed;
      wheel_input.velocity[5] = linear_speed + (tred_width_[2]/2.0)*angular_speed;
    }
  }else // pivot turn
  {
    //
    // TODO
    //
  }
}
