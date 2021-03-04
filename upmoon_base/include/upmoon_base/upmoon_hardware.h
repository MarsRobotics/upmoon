#ifndef UPMOON_BASE_UPMOON_HARDWARE_H
#define UPMOON_BASE_UPMOON_HARDWARE_H

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"

namespace upmoon_base {

class UPMoonHardware : public hardware_interface::RobotHW
{
public:
    UPMoonHardware(ros::NodeHandle &nh);
    void write();
    void read();

private:
    struct Joint
    {
        double position;
        double velocity;
        double effort;
        double command;

        Joint() : position(0), velocity(0), effort(0), command(0)
        {
        }
    };

    struct Joint drive_joints_[6];
    struct Joint ankle_joints_[6];

    // velocity interface is for drive train and position is for ankle motors
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;   
};

} // namespace upmoon_base

#endif