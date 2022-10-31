#ifndef UPMOON_BASE_UPMOON_HARDWARE_H
#define UPMOON_BASE_UPMOON_HARDWARE_H

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "std_msgs/Float64.h"
#include "ros/ros.h"

namespace upmoon_base {

class UPMoonHardware : public hardware_interface::RobotHW
{
public:
    UPMoonHardware(ros::NodeHandle &nh);
    void write();
    void enforceLimits(const ros::Time &time);

private:
    struct Joint
    {
        double position;
        double velocity;
        double effort;
        double command;
        ros::Publisher topic;

        Joint() : position(0), velocity(0), effort(0), command(0)
        {
        }
    };

    struct Joint drive_joints_[6];
    struct Joint ankle_joints_[4];
    struct Joint dig_angle_joint_;

    // velocity interface is for drive train and position is for ankle motors
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    const double DIG_ANGLE_MAX = 100;
    const double DIG_ANGLE_MIN = 0;

    ros::Time prev_time;
};

} // namespace upmoon_base

#endif