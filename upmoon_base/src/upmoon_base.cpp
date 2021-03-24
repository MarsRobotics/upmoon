#include <ros/ros.h>
#include "upmoon_base/upmoon_hardware.h"
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "upmoon_base");
    ros::NodeHandle nh;
    
    upmoon_base::UPMoonHardware upmoon_hardware(nh);
    controller_manager::ControllerManager cm(&upmoon_hardware, nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Rate loop_rate(10);
    ros::Time last_time = ros::Time::now();

    while (ros::ok())
    {
        ros::Time time = ros::Time::now();

        upmoon_hardware.enforceLimits(time);
        cm.update(ros::Time::now(), time - last_time);
        upmoon_hardware.write();

        loop_rate.sleep();
        last_time = time;
    }

    return 0;
}