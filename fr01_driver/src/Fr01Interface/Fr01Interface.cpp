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
                          sensor_msgs::JointState& steer_input,
                          bool isPivotTurn=false)
{
  if(!isPivotTurn)
  {
    angular_speed = -angular_speed;
    steer_input.position[2] = atan2(2.0*tred_length_[2]*tan(angular_speed),
                                    2.0*tred_length_[2]+2.0*tred_width_[2]*tan(angular_speed));
    steer_input.position[3] = atan2(2.0*tred_length_[3]*tan(angular_speed),
                                    2.0*tred_length_[3]-2.0*tred_width_[2]*tan(angular_speed));

    steer_input.position[0] = -steer_input.position[2];
    steer_input.position[1] = -steer_input.position[3];
      
    // wheel
    // 0 : left rear,   1 : right rear, 
    // 2 : left middle, 3 : right middle, 
    // 4 : left front,  5 : right front
    if(angular_speed == 0.0)
    {
      for (size_t i = 0; i < 6; ++i) {
        wheel_input.velocity[i] = linear_speed;
      }
    }else{
      wheel_input.velocity[4] = (sin(angular_speed)/sin(steer_input.position[2]))*linear_speed;
      wheel_input.velocity[0] = wheel_input.velocity[4];
      wheel_input.velocity[5] = (sin(angular_speed)/sin(steer_input.position[3]))*linear_speed;
      wheel_input.velocity[1] = wheel_input.velocity[5];
    
      wheel_input.velocity[2] = (sin(angular_speed)/tan(steer_input.position[2]))*linear_speed;
      wheel_input.velocity[3] = (sin(angular_speed)/tan(steer_input.position[3]))*linear_speed;
    }
    
  }else // pivot turn
  {
    //
    // TODO
    //
  }
}
