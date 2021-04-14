# upmoon
ROS packages for a Lunabotics mining robot. Each ROS package is defined below. Further details on the robot's hardware and configuration are included in the repo's [wiki](https://github.com/MarsRobotics/upmoon/wiki).

## upmoon_base
Maintains a [ROS Control](http://wiki.ros.org/ros_control) hardware interface for connecting ROS controller to physical hardware. This package is written in C++. This also includes the base launch script for starting physical robot. The script starts the ROS controllers, the hardware interface, and the GPIO motor communications. Because the GPIO package is defined in Python and there's no easy way to communicate to Python objects in C++ without learning special wrappers, the hardware interface passes all motor commands to the GPIO motor controller on a ROS topic prefixed with "/motor".

## upmoon_control
Defines [ROS controllers](http://wiki.ros.org/ros_control#Controllers) and launches the [controller_manager](http://wiki.ros.org/controller_manager) and [robot_state_publisher](http://wiki.ros.org/robot_state_publisher). The ROS controller offer the software means to instantiate position, velocity, and efforts of a joint. High level controller, such as the [differential drive controller](http://wiki.ros.org/diff_drive_controller) are configured here. The controller manager node "provides the infrastructure to load, unload, start and stop controllers", while the robot state publisher tracks the joint positions, with respect to a coordinate frame.

## upmoon_description
Loads the robot [URDF](http://wiki.ros.org/urdf) model into the ROS parameter server. ROS uses the model to understand the relationships between static and moving parts on the robot, such as dimensions, limits, and controllers. An important aspect of the model is defining the coordinate frame of sensors in relationship to the absolute position on the robot. Also, parameters for a Gazebo simulation are configured in this model.

The initial contents of this package were generated from the Solidworks CAD model, using an exporter called [solidworks_urdf_exporter](https://github.com/ros/solidworks_urdf_exporter). The only additional aspect added to the model manually was a digging angle controller, as the digging system was not in the CAD model at the time.

## upmoon_gazebo
Defines a launch file to start a Gazebo simulation world with the robot's URDF model. The upmoon_control setup is included in this file.

## upmoon_gpio
Controls the Raspberry Pi's GPIO pins for interacting with motors and motor controllers. Each motor and motor controller are abstracted into a Python class in `upmoon_gpio/src/upmoon_gpio/`. There should be a one-to-one ratio between Python motor objects and physical devices. Each device is configured in the ROS node under `upmoon_gpio/scripts/motor_drivers` and is controlled with individual ROS topics. These topics are defined below. All topics take a Float64 message.

| Topic | Description |
|-------|-------------|
|/motor/wheel_lf_joint|Sets the angular velocity of the left-front Trianmics motor.|
|/motor/wheel_lm_joint|Sets the angular velocity of the left-middle Trianmics motor.|
|/motor/wheel_lb_joint|Sets the angular velocity of the left-back Trianmics motor.|
|/motor/wheel_rf_joint|Sets the angular velocity of the right-front Trianmics motor.|
|/motor/wheel_rm_joint|Sets the angular velocity of the right-middle Trianmics motor.|
|/motor/wheel_rb_joint|Sets the angular velocity of the right-back Trianmics motor.|
|/motor/ankle_lf_joint|Sets the angle of the left-front stepper motor (in radians).|
|/motor/ankle_lm_joint|Sets the angle of the left-middle stepper motor (in radians).|
|/motor/ankle_lb_joint|Sets the angle of the left-back stepper motor (in radians).|
|/motor/ankle_rf_joint|Sets the angle of the right-front stepper motor (in radians).|
|/motor/ankle_rm_joint|Sets the angle of the right-middle stepper motor (in radians).|
|/motor/ankle_rb_joint|Sets the angle of the right-back stepper motor (in radians).|
|/motor/dig_spin|Sets the power of the bucket chain's digging motor (0-100).|
|/motor/dig_angle_joint|Sets the length of the bucket chain's linear actuators. 0 is fully retracted and 100 is fully extended.|
|/motor/depositor_actuator|Sets the position of the deposition arm. 0 is fully retracted and 100 is fully extended.|


## upmoon_navigation
Defines a launch file to start the Intel Realsense D435 (depth sensing) and T265 (tracking) [camera nodes](https://github.com/IntelRealSense/realsense-ros) and the [rtabmap](http://wiki.ros.org/rtabmap_ros) node. These nodes will host map (obstacles around the robot) and odometry (the robot's location in the map) data. Due to time constrains, nothing has been implemented to use this data. It was intended to be used with ROS's [move_base](http://wiki.ros.org/move_base) node to be able to pick a goal on the map for the robot to move to, where move_base will output a Twist message for how the robot should move to reach the goal.

## upmoon_teleop
Allows a gamepad to control the robot via ROS messages. To properly connect a gamepad to ROS, it needs to be configured with these steps [here](https://github.com/MarsRobotics/upmoon/wiki/Basic-Usage#starting-remote-computer). A table of published topics is given below.

|Topic|Type|Description|
|-|-|-|
|/diff_drive_controller/cmd_vel|Twist|Controls movement of robot. ROS Control converts this to drive motor commands using a differential drive controller.|
|/dig_angle_controller/command|Float64|Sets the velocity of the bucket chain's linear actuators. ROS Control converts this to motor position commands.|
|/ankle_lf_controller/command|Float64|Sets the angle of the left-front ankle joint (in radians). ROS Control forwards this to motor position commands.|
|/ankle_lm_controller/command|Float64|Sets the angle of the left-middle ankle joint (in radians). ROS Control forwards this to motor position commands.|
|/ankle_lb_controller/command|Float64|Sets the angle of the left-back ankle joint (in radians). ROS Control forwards this to motor position commands.|
|/ankle_rf_controller/command|Float64|Sets the angle of the right-front ankle joint (in radians). ROS Control forwards this to motor position commands.|
|/ankle_rm_controller/command|Float64|Sets the angle of the right-middle ankle joint (in radians). ROS Control forwards this to motor position commands.|
|/ankle_rb_controller/command|Float64|Sets the angle of the right-back ankle joint (in radians). ROS Control forwards this to motor position commands.|
|/motor/dig_spin|Float64|Sets the bucket chains' digging power (0-100).
|/motor/depositor_actuator|Float64|Sets the position of the deposition arm. 0 is fully retracted and 100 is fully extended.|

## upmoon_viz
Define two launch files for starting [rviz](http://wiki.ros.org/rviz), a visualization tool. `view_model.launch` is an independent script for debugging URDF models. `view_robot.launch` is more likely to be used. It starts rviz with the saved rviz configuration file for viewing robot data with Gazebo the physical robot.
