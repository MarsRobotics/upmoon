#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Float64

class TeleopJoy:

    SPEED_LINEAR = 1    # Max velocity for a linear twist vector
    SPEED_ANGULAR = 1   # Max velocity for an angular twist vector
    ANKLE_PACK_IN = 0
    ANKLE_PACK_OUT = 3.14159265

    def __init__(self):
        self.pub_twist              = rospy.Publisher('/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        
        self.pub_excavate_spin      = rospy.Publisher('/motor/excavate_spin', Float64, queue_size=10)
        self.pub_excavate_actuator  = rospy.Publisher('/motor/excavate_actuator', Float64, queue_size=10)
        self.pub_depositor_actuator = rospy.Publisher('/motor/depositor_actuator', Float64, queue_size=10)
        self.pub_ankle_lf           = rospy.Publisher('/ankle_lf_controller/command', Float64, queue_size=10)
        self.pub_ankle_lm           = rospy.Publisher('/ankle_lm_controller/command', Float64, queue_size=10)
        self.pub_ankle_lb           = rospy.Publisher('/ankle_lb_controller/command', Float64, queue_size=10)
        self.pub_ankle_rf           = rospy.Publisher('/ankle_rf_controller/command', Float64, queue_size=10)
        self.pub_ankle_rm           = rospy.Publisher('/ankle_rm_controller/command', Float64, queue_size=10)
        self.pub_ankle_rb           = rospy.Publisher('/ankle_rb_controller/command', Float64, queue_size=10)

        rospy.Subscriber('joy', Joy, self.joy_callback)
    

    """
    Set all A joints to a radian
    """
    def ankle_set(self, rad):
        payload = Float64()
        payload.data = rad
        self.pub_ankle_lf.publish(payload)
        self.pub_ankle_lm.publish(payload)
        self.pub_ankle_lb.publish(payload)
        self.pub_ankle_rf.publish(payload)
        self.pub_ankle_rm.publish(payload)
        self.pub_ankle_rb.publish(payload)


    """
    Example Twist Message:
        twist.linear.x = 0 # Forward/back
        twist.linear.y = 0 # Strafe left/right (not currently implemented)
        twist.linear.z = 0 # We don't go up and down
        twist.angular.x = 0 # Ignore
        twist.angular.y = 0 # Ignore
        twist.angular.z = 0 # Turn left/right (control this with left stick)
    """
    def joy_callback(self, joy_msg):
        twist = Twist()

        # log joystick data for learning
        rospy.loginfo("axes: {}".format(str(joy_msg.axes)))
        rospy.loginfo("buttons: {}".format(str(joy_msg.buttons)))
        
        # Right Trigger: Accelerate
        # joy_msg.axes[5] (default = 1.0, fully pressed = -1.0)
        r_trig = self.normalize_trigger(joy_msg.axes[5])

        # Left Trigger: Decelerate
        # joy_msg.axes[2] (default = 1.0, fully pressed = -1.0)
        l_trig = -self.normalize_trigger(joy_msg.axes[2])

        # Scale input to actual velocity values
        rospy.loginfo("l_trig: {} r_trig: {}".format(l_trig, r_trig))
        twist.linear.x = r_trig*self.SPEED_LINEAR + l_trig*self.SPEED_LINEAR
        twist.linear.y = twist.linear.z = 0

        # Left Stick Y-Axis: Steer
        # joy_msg.axes[0] (default = 0, up = 1, down = 0)
        l_stick_x = joy_msg.axes[0]
        twist.angular.z = l_stick_x*self.SPEED_ANGULAR
        twist.angular.x = twist.angular.y = 0
        
        # Right Stick X-Axis: Straif
        # FIXME Goal: Be able to reset "North" for straif
        # Requires sensor implemenation (to determine which direction we are facing)
        # As such, this will not be implemented until later
        
        # joy_msg.axes[3] (default = 0, left = 1, right = -1)
        # rs_x = joy_msg.axes[3]
        # rs_y = joy_msg.axes[4]
        # twist.linear.x = total_speed * rs_x
        # twist.linear.y = total_speed * rs_y
        
        if (joy_msg.buttons[15] and joy_msg.buttons[16]):
            # ignore as both are pressed
            pass
        elif (joy_msg.buttons[15]):
            # left d pad was pressed => pack out
            self.ankle_set(self.ANKLE_PACK_OUT)
        elif (joy_msg.buttons[16]):
            # right d pad was pressed => pack in
            self.ankle_set(self.ANKLE_PACK_IN)

        # Publish twist message
        self.pub_twist.publish(twist)

    def normalize_trigger(self, trig):
        trig = float(trig)
        trig +=1
        trig = 2-trig
        return trig/2 # optional conversion to 0 to 1 instead of 0 to 2


if __name__ == '__main__':
    try:
        rospy.init_node('teleop_joy')
        joy = TeleopJoy()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

