/**
* https://github.com/husky/husky/  
* https://github.com/jackal/jackal_robot/
* https://github.com/Brazilian-Institute-of-Robotics/doogie_robot/
*/

#include "upmoon_base/upmoon_hardware.h"

namespace upmoon_base {

UPMoonHardware::UPMoonHardware(ros::NodeHandle &nh)
{
    std::string drive_names[6] = {"wheel_lf_joint", "wheel_lm_joint", "wheel_lb_joint",
                                  "wheel_rf_joint", "wheel_rm_joint", "wheel_rb_joint"};

    std::string ankle_names[6] = {"ankle_lf_joint", "ankle_lm_joint", "ankle_lb_joint",
                                  "ankle_rf_joint", "ankle_rm_joint", "ankle_rb_joint"};

    // drive wheels use a velocity controller
    for (int i = 0; i < 6; i++) {
        std::string topic_name = "/motor/" + drive_names[i];
        drive_joints_[i].topic = nh.advertise<std_msgs::Float64>(topic_name, 10);

        hardware_interface::JointStateHandle joint_state_handle(drive_names[i],
                                                                &drive_joints_[i].position,
                                                                &drive_joints_[i].velocity,
                                                                &drive_joints_[i].effort);
        joint_state_interface_.registerHandle(joint_state_handle);

        hardware_interface::JointHandle joint_handle(joint_state_handle, &drive_joints_[i].command);
        velocity_joint_interface_.registerHandle(joint_handle);
    }

    // ankle motors use a position controller
    for (int i = 0; i < 6; i++) {
        std::string topic_name = "/motor/" + ankle_names[i];
        ankle_joints_[i].topic = nh.advertise<std_msgs::Float64>(topic_name, 10);

        hardware_interface::JointStateHandle joint_state_handle(ankle_names[i],
                                                                &ankle_joints_[i].position,
                                                                &ankle_joints_[i].velocity,
                                                                &ankle_joints_[i].effort);
        joint_state_interface_.registerHandle(joint_state_handle);

        hardware_interface::JointHandle joint_handle(joint_state_handle, &ankle_joints_[i].command);
        position_joint_interface_.registerHandle(joint_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&position_joint_interface_);
}

void UPMoonHardware::write()
{
    //TODO: setup position limits for ankles: https://github.com/ros-controls/ros_control/wiki/joint_limits_interface

    for (int i = 0; i < 6; i++) {
        std_msgs::Float64 msg;
        msg.data = drive_joints_[i].command;
        drive_joints_[i].topic.publish(msg);
    }

    for (int i = 0; i < 6; i++) {
        std_msgs::Float64 msg;
        msg.data = ankle_joints_[i].command;
        ankle_joints_[i].topic.publish(msg);
    }

}

void UPMoonHardware::read()
{

}


} // namespace upmoon_base