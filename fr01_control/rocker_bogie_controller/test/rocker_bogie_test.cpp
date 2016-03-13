#include<stdio.h>
#include<iostream>
#include<ros/ros.h>
#include<rocker_bogie_controller/rocker_bogie_controller.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>

int configureVelocityJointInterface(hardware_interface::VelocityJointInterface&);

class RockerBogieBot: public hardware_interface::RobotHW
{
    // methods
public:
    RockerBogieBot(ros::NodeHandle &_nh)
        : ns_("fr01_rocker_bogie_controller/")
    {
        this->CleanUp();
        this->GetJointNames(_nh);
        this->Resize();
        this->RegisterHardwareInterfaces();
    }

    void read()
    {
        std::ostringstream os;
        for (unsigned int i = 0; i < cnt_wheel_joints_ - 1; ++i)
        {
            os << wheel_joint_vel_cmd_[i] << ", ";
        }
        os << wheel_joint_vel_cmd_[cnt_wheel_joints_ - 1];

        ROS_INFO_STREAM("Commands for wheel joints: " << os.str());
    }

    ros::Duration getPeriod() const {return ros::Duration(0.01);}

    void write()
    {
        bool running_ = true;
        if (running_)
        {
            for (unsigned int i = 0; i < cnt_wheel_joints_; ++i)
            {
                // Note that pos_[i] will be NaN for one more cycle after we start(),
                // but that is consistent with the knowledge we have about the state
                // of the robot.
                wheel_joint_pos_[i] += wheel_joint_vel_[i]*getPeriod().toSec(); // update position
                wheel_joint_vel_[i] = wheel_joint_vel_cmd_[i]; // might add smoothing here later
            }
        }
#if 0
        else
        {
            std::fill_n(joint_pos_, cnt_wheel_joints_, std::numeric_limits<double>::quiet_NaN());
            std::fill_n(joint_vel_, cnt_wheel_joints_, std::numeric_limits<double>::quiet_NaN());
        }
#endif
    }
private:
    void CleanUp()
    {
       // wheel joints
       wheel_joint_names_.clear();
       wheel_joint_pos_.clear();
       wheel_joint_vel_.clear();
       wheel_joint_eff_.clear();
       wheel_joint_vel_cmd_.clear();

       // steer joints
       steer_joint_names_.clear();
       steer_joint_pos_.clear();
       steer_joint_vel_.clear();
       steer_joint_eff_.clear();
       steer_joint_vel_cmd_.clear();
    }

    void Resize()
    {
       // wheel joints
       wheel_joint_pos_.resize(cnt_wheel_joints_);
       wheel_joint_vel_.resize(cnt_wheel_joints_);
       wheel_joint_eff_.resize(cnt_wheel_joints_);
       wheel_joint_vel_cmd_.resize(cnt_wheel_joints_);

       // steer joints
       steer_joint_pos_.resize(cnt_steer_joints_);
       steer_joint_vel_.resize(cnt_steer_joints_);
       steer_joint_eff_.resize(cnt_steer_joints_);
       steer_joint_vel_cmd_.resize(cnt_steer_joints_);
    }

    void GetJointNames(ros::NodeHandle &_nh)
    {
        this->GetWheelJointNames(_nh);
        this->GetSteerJointNames(_nh);
    }

    void GetWheelJointNames(ros::NodeHandle &_nh)
    {
        // wheel names
        std::vector<std::string> right_wheel_joint_names;
        std::vector<std::string> left_wheel_joint_names;

        _nh.getParam(ns_ + "right_wheel", right_wheel_joint_names);
        _nh.getParam(ns_ + "left_wheel", left_wheel_joint_names);

        wheel_joint_names_ = right_wheel_joint_names;
        std::copy(left_wheel_joint_names.begin(), left_wheel_joint_names.end(),
                  std::back_inserter(wheel_joint_names_));
        cnt_wheel_joints_ = wheel_joint_names_.size();
    }

    void GetSteerJointNames(ros::NodeHandle &_nh)
    {
        // steer names
        std::vector<std::string> right_steer_joint_names;
        std::vector<std::string> left_steer_joint_names;

        _nh.getParam(ns_ + "right_steer", right_steer_joint_names);
        _nh.getParam(ns_ + "left_steer", left_steer_joint_names);

        steer_joint_names_ = right_steer_joint_names;
        std::copy(left_steer_joint_names.begin(), left_steer_joint_names.end(),
                  std::back_inserter(steer_joint_names_));
        cnt_steer_joints_ = steer_joint_names_.size();
    }

    void RegisterHardwareInterfaces()
    {
      this->RegisterSteerInterfaces();
      this->RegisterWheelInterfaces();
    }

    void RegisterWheelInterfaces()
    {
        // map members to hardware interfaces
        for (size_t i = 0; i < cnt_wheel_joints_; ++i) {
            // joint states
            hardware_interface::JointStateHandle state_handle(wheel_joint_names_[i],
                                                              &wheel_joint_pos_[i],
                                                              &wheel_joint_vel_[i],
                                                              &wheel_joint_eff_[i]);
            wheel_joint_state_interface_.registerHandle(state_handle);

            // joint velocity command
            hardware_interface::JointHandle vel_handle(wheel_joint_state_interface_.getHandle(wheel_joint_names_[i]),
                                                       &wheel_joint_vel_cmd_[i]);
            wheel_vel_joint_interface_.registerHandle(vel_handle);

            ROS_DEBUG_STREAM("Registered joint '" << wheel_joint_names_[i] << " ' in the VelocityJointInterface");
        }

        // register mapped interface to ros_control
        registerInterface(&wheel_joint_state_interface_);
        registerInterface(&wheel_vel_joint_interface_);
    }

    void RegisterSteerInterfaces()
    {
        // map members to hardware interfaces
        for (size_t i = 0; i < cnt_steer_joints_; ++i) {
            // joint states
            hardware_interface::JointStateHandle state_handle(steer_joint_names_[i],
                                                              &steer_joint_pos_[i],
                                                              &steer_joint_vel_[i],
                                                              &steer_joint_eff_[i]);
            steer_joint_state_interface_.registerHandle(state_handle);

            // joint position command
            hardware_interface::JointHandle pos_handle(steer_joint_state_interface_.getHandle(steer_joint_names_[i]),
                                                       &steer_joint_vel_cmd_[i]);
            steer_pos_joint_interface_.registerHandle(pos_handle);

            ROS_DEBUG_STREAM("Registered joint '" << steer_joint_names_[i] << " ' in the PositionJointInterface");
        }

        // register mapped interface to ros_control
        registerInterface(&steer_joint_state_interface_);
        registerInterface(&steer_pos_joint_interface_);
    }

    // member variables
public:
    hardware_interface::VelocityJointInterface wheel_vel_joint_interface_;
    hardware_interface::PositionJointInterface steer_pos_joint_interface_;

private:
    ros::NodeHandle nh_;

    std::string ns_;
    //
    std::vector<std::string> wheel_joint_names_;
    std::vector<std::string> steer_joint_names_;

    // interface variables
    //-- wheel
    int cnt_wheel_joints_;
    std::vector<double> wheel_joint_pos_;
    std::vector<double> wheel_joint_vel_;
    std::vector<double> wheel_joint_eff_;
    std::vector<double> wheel_joint_vel_cmd_;

    //-- steer
    int cnt_steer_joints_;
    std::vector<double> steer_joint_pos_;
    std::vector<double> steer_joint_vel_;
    std::vector<double> steer_joint_eff_;
    std::vector<double> steer_joint_vel_cmd_;

    //
    hardware_interface::JointStateInterface wheel_joint_state_interface_;
    hardware_interface::JointStateInterface steer_joint_state_interface_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rocker_bogie_test");
    ros::NodeHandle nh_main;
    ros::NodeHandle nh_control;

    RockerBogieBot* rocker_bogie_bot = new RockerBogieBot(nh_control);

    rocker_bogie_controller::RockerBogieController rb_controller;
#if 0
    bool rt_code = rb_controller.init(//&rocker_bogie_bot->wheel_vel_joint_interface_,
                                      rocker_bogie_bot,
                                      nh_main, nh_control);
#endif
    std::set<std::string> claimed_resources; // Gets populated during initRequest call
    bool rt_code = rb_controller.init(rocker_bogie_bot, nh_main, nh_control);//, claimed_resources);

    ros::Rate rate(100);
    ros::Time last_time = ros::Time::now();

    while(nh_main.ok())
    {
        ros::Time now = ros::Time::now();
        ros::Duration period = now - last_time;
        last_time = now;

        rocker_bogie_bot->read();
        rb_controller.update(now, period);
        rocker_bogie_bot->write();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
