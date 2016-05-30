#include <ros/ros.h>
#include <fr01_interface/fr01_interface.hpp>

#include <angles/angles.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>


Fr01Interface::Fr01Interface()
{
  ros::NodeHandle n("~");
  std::vector<std::string> left_wheel_names;
  std::vector<std::string> right_wheel_names;
  std::vector<std::string> left_steer_names;
  std::vector<std::string> right_steer_names;

  // Wheel
  if(!getWheelNames(n, "fr01_rocker_bogie_controller/left_wheel", left_wheel_names) |
     !getWheelNames(n, "fr01_rocker_bogie_controller/right_wheel", right_wheel_names))
    {
      ROS_ERROR_STREAM("getWheelNames() error");
    }
  else
    {
      wheel_joint_names_.reserve(left_wheel_names.size() + right_wheel_names.size());
      wheel_joint_names_.insert(wheel_joint_names_.end(), left_wheel_names.begin(), left_wheel_names.end());
      wheel_joint_names_.insert(wheel_joint_names_.end(), right_wheel_names.begin(), right_wheel_names.end());
    }
  // Steer
  if(!getSteerNames(n, "fr01_rocker_bogie_controller/left_steer", left_steer_names) |
     !getSteerNames(n, "fr01_rocker_bogie_controller/right_steer", right_steer_names))
    {
      ROS_ERROR_STREAM("getSteerNames() error");
    }
  else
    {
      steer_joint_names_.reserve(left_steer_names.size() + right_steer_names.size());
      steer_joint_names_.insert(steer_joint_names_.end(), left_steer_names.begin(), left_steer_names.end());
      steer_joint_names_.insert(steer_joint_names_.end(), right_steer_names.begin(), right_steer_names.end());
    }

  wheel_vel_sub_ = nh_.subscribe(n.param<std::string>("wheel_state_topic_name", "/wheel_states"), 1, &Fr01Interface::wheelStateCallback, this);
  steer_pos_sub_ = nh_.subscribe(n.param<std::string>("steer_state_topic_name", "/steer_states"), 1, &Fr01Interface::steerStateCallback, this);

  do
    {
      while (ros::ok() && wheel_state_timestamp_.toSec() == 0) {
	ROS_INFO_STREAM_NAMED("hardware_interface", "Waiting for first wheel state message to be recieved");
	ros::spinOnce();
	ros::Duration(0.25).sleep();
      }
    } while (wheel_state_ptr_->name.size() != 6);
  ROS_INFO_STREAM_NAMED("hardware_interface", "Done for first wheel state message to be recieved");
  do
    {
      while (ros::ok() && steer_state_timestamp_.toSec() == 0) {
	ROS_INFO_STREAM_NAMED("hardware_interface", "Waiting for first steer state message to be recieved");
	ros::spinOnce();
	ros::Duration(0.25).sleep();
      }
    } while (steer_state_ptr_->name.size() != 4);
  ROS_INFO_STREAM_NAMED("hardware_interface", "Done for first steer state message to be recieved");

  fr01_wheel_ptr_.reset(new Fr01WheelInterface(wheel_joint_names_));
  fr01_steer_ptr_.reset(new Fr01SteerInterface(steer_joint_names_));

  fr01_wheel_ptr_->register_interface(joint_state_interface_,
  				      wheel_vel_joint_interface_);
  fr01_steer_ptr_->register_interface(joint_state_interface_,
  				      steer_pos_joint_interface_);

  registerInterface(&joint_state_interface_);
  registerInterface(&wheel_vel_joint_interface_);
  registerInterface(&steer_pos_joint_interface_);

  ROS_INFO_STREAM_NAMED("hardware_interface", "Complete register Interfaces.");

  // Position joint limits interface
  // TODO
}

void Fr01Interface::read(ros::Time now, ros::Duration period)
{
  ROS_INFO_STREAM_NAMED("hardware_interface", "Fr01Interface::read() is called");
  fr01_wheel_ptr_->read(wheel_state_ptr_);
  ROS_INFO_STREAM_NAMED("hardware_interface", "wheel_ptr read");
  fr01_steer_ptr_->read(steer_state_ptr_);
  ROS_INFO_STREAM_NAMED("hardware_interface", "steer_ptr read");
}

void Fr01Interface::write(ros::Time now, ros::Duration period)
{
  ROS_INFO_STREAM_NAMED("hardware_interface", "Fr01Interface::write() is called");
  fr01_wheel_ptr_->write();
  ROS_INFO_STREAM_NAMED("hardware_interface", "wheel_ptr wrote");
  fr01_steer_ptr_->write();
  ROS_INFO_STREAM_NAMED("hardware_interface", "steer_ptr wrote");
}

void Fr01Interface::wheelStateCallback(const sensor_msgs::JointStateConstPtr& wheel_state)
{
  if (wheel_state->name.size() != 6) {
    return;
  }
  wheel_state_ptr_ = wheel_state;
  wheel_state_timestamp_ = ros::Time::now();
}

void Fr01Interface::steerStateCallback(const sensor_msgs::JointStateConstPtr& steer_state)
{
  if(steer_state->name.size() != 4){
    return;
  }
  steer_state_ptr_ = steer_state;
  steer_state_timestamp_ = ros::Time::now();
}

bool Fr01Interface::getWheelNames(ros::NodeHandle& nh, const std::string& wheel_joint_name_param,
				  std::vector<std::string>& wheel_names)
{
  XmlRpc::XmlRpcValue wheel_list;
  if (!nh.getParam(wheel_joint_name_param, wheel_list))
    {
      ROS_ERROR_STREAM("Couldn't retrieve wheel param '" << wheel_joint_name_param << "'.");
      return false;
    }

  if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      if (wheel_list.size() == 0)
        {
          ROS_ERROR_STREAM("Wheel param '" << wheel_joint_name_param << "' is an empty list");
          return false;
        }
      for (int i = 0; i < wheel_list.size(); ++i)
        {
          if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
	    {
	      ROS_ERROR_STREAM("Wheel param '" << wheel_joint_name_param << "' #" << i <<
			       " isn't a string.");
	      return false;
	    }
        }

      wheel_names.resize(wheel_list.size());
      for (int i = 0; i < wheel_list.size(); ++i)
        {
          wheel_names[i] = static_cast<std::string>(wheel_list[i]);
        }
    }
  else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      wheel_names.push_back(wheel_list);
    }
  else
    {
      ROS_ERROR_STREAM("Wheel param '" << wheel_joint_name_param <<
		       "' is neither a list of strings nor a string.");
      return false;
    }

  return true;
}

bool Fr01Interface::getSteerNames(ros::NodeHandle& nh, const std::string& steer_joint_name_param,
				  std::vector<std::string>& steer_names)
{
  XmlRpc::XmlRpcValue steer_list;
  if (!nh.getParam(steer_joint_name_param, steer_list))
    {
      ROS_ERROR_STREAM("Couldn't retrieve steer param '" << steer_joint_name_param << "'.");
      return false;
    }
  if (steer_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      if (steer_list.size() == 0)
        {
          ROS_ERROR_STREAM("Steer param '" << steer_joint_name_param << "' is an empty list");
          return false;
        }
        for (int i = 0; i < steer_list.size(); ++i)
	  {
	    if (steer_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
	      {
		ROS_ERROR_STREAM("Steer param '" << steer_joint_name_param << "' #" << i <<
				 " isn't a string.");
		return false;
          }
        }

        steer_names.resize(steer_list.size());
        for (int i = 0; i < steer_list.size(); ++i)
	  {
	    steer_names[i] = static_cast<std::string>(steer_list[i]);
	  }
    }
  else if (steer_list.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      steer_names.push_back(steer_list);
    }
  else
    {
      ROS_ERROR_STREAM("Steer param '" << steer_joint_name_param <<
		       "' is neither a list of strings nor a string.");
      return false;
    }

  return true;
}
