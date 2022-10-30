#!/usr/bin/env python3

import threading
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from actionlib import SimpleActionClient
from upmoon_action_msg.msg import ArticulateAction, ArticulateGoal, ArticulateFeedback

class TeleopJoy:

    SPEED_LINEAR = 1    # Max velocity for a linear twist vector
    SPEED_ANGULAR = 1   # Max velocity for an angular twist vector
    ANKLE_PACK_IN = 0
    ANKLE_PACK_OUT = 3.14
    ANKLE_TURN = 2.36
    DIGGING_ANGLE_COEF = -5
    DIGGING_SPEED_MAX = 10 # Max percentage of power for digging
    DIGGING_SPEED_MIN = 0
    DEPOSIT_ACT_TOGGLE_INIT = 0
    DEPOSIT_LIFT_TOGGLE_INIT = 100

    def __init__(self):
        self.pub_twist              = rospy.Publisher('/diff_drive_controller/cmd_vel', Twist, queue_size=10)

        self.articulate_action      = SimpleActionClient('action_articulate', ArticulateAction)
        self.articulate_event       = threading.Event()
        
        self.pub_dig_spin           = rospy.Publisher('/motor/dig_spin', Float64, queue_size=10)
        self.pub_dig_angle_speed    = rospy.Publisher('/dig_angle_controller/command', Float64, queue_size=10)
        self.pub_depositor_actuator = rospy.Publisher('/motor/depositor_actuator', Float64, queue_size=10)
        self.pub_depositor_lift     = rospy.Publisher('/motor/depositor_lift', Float64, queue_size=10)

        self.deposit_act_toggle = self.DEPOSIT_ACT_TOGGLE_INIT
        self.deposit_lift_toggle = self.DEPOSIT_LIFT_TOGGLE_INIT

        self.pub_depositor_actuator.publish(self.deposit_act_toggle)
        self.pub_depositor_lift.publish(self.deposit_lift_toggle)

        self.prev_joy_msg = None

        rospy.Subscriber('joy', Joy, self.joy_callback)
    

    """
    Set all A joints to a radian
    """
    def ankle_set(self, rad: float):
        goal = ArticulateGoal(lf=rad,
                              lm=rad,
                              lb=rad,
                              rf=rad,
                              rm=rad,
                              rb=rad)

        self.articulate_event.set()

        # Send goal to the action server and ignore the passed args in the callback.
        self.articulate_action.send_goal(goal, done_cb=lambda *_: self.articulate_event.clear())


    def turn_state(self):
        goal = ArticulateGoal(lf=self.ANKLE_TURN,
                              lm=self.ANKLE_PACK_OUT,
                              lb=self.ANKLE_TURN,
                              rf=self.ANKLE_TURN,
                              rm=self.ANKLE_PACK_OUT,
                              rb=self.ANKLE_TURN)

        self.articulate_event.set()
        self.articulate_action.send_goal(goal, done_cb=lambda *_: self.articulate_event.clear())


    # Stops the drive and digger
    def soft_stop(self):
        if (self.articulate_event.is_set()):
            self.articulate_action.cancel_all_goals()
            self.articulate_event.clear()
        else:
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.pub_twist.publish(twist)

        dig = Float64()
        dig.data = 0
        self.pub_dig_spin.publish(dig)
        self.pub_dig_angle_speed.publish(Float64(0))


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
        # X causes soft stop
        if (joy_msg.buttons[0]):
            self.soft_stop()
            return
        
        # Control driving when we are not articulating A-joints
        if (not self.articulate_event.is_set()):
            # Create Twist message
            twist = Twist()

            # Right Trigger: Accelerate
            # joy_msg.axes[5] (default = 1.0, fully pressed = -1.0)
            r_trig = self.normalize_trigger(joy_msg.axes[5])

            # Left Trigger: Decelerate
            # joy_msg.axes[2] (default = 1.0, fully pressed = -1.0)
            l_trig = -self.normalize_trigger(joy_msg.axes[2])

            # Scale input to actual velocity values
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

            # Publish twist message
            self.pub_twist.publish(twist)
        
        # Control ankles
        if (joy_msg.buttons[14] and not self.prev_joy_msg.buttons[14]):
            # bottom d pad was pressed => turn state
            self.turn_state()
        elif (joy_msg.buttons[15] and not self.prev_joy_msg.buttons[15]):
            # left d pad was pressed => pack out
            self.ankle_set(self.ANKLE_PACK_OUT)
        elif (joy_msg.buttons[16] and not self.prev_joy_msg.buttons[16]):
            # right d pad was pressed => pack in
            self.ankle_set(self.ANKLE_PACK_IN)

        # Control digger speed
        if (joy_msg.buttons[4] and joy_msg.buttons[5]):
            # ignore as both bumpers are pressed
            pass
        elif (joy_msg.buttons[4]):
            # left bumper was pressed => reverse digger
            self.pub_dig_spin.publish(Float64(0.2*-self.DIGGING_SPEED_MAX))
        elif (joy_msg.buttons[5]):
            # right bumper was pressed => forward
            self.pub_dig_spin.publish(Float64(self.DIGGING_SPEED_MAX))

        # Right analog stick to raise/lower digger
        r_stick_y = joy_msg.axes[4]
        dig_angle_speed = r_stick_y * self.DIGGING_ANGLE_COEF
        self.pub_dig_angle_speed.publish(Float64(dig_angle_speed))

        # Use Triangle / Y Button for opening/closing deposition (press to open and press to close)
        if (joy_msg.buttons[2]):
            if (self.deposit_act_toggle == 0) :
                self.deposit_act_toggle = 100
            else:
                self.deposit_act_toggle = 0
            self.pub_depositor_actuator.publish(self.deposit_act_toggle)

        # Use the Square Button for raising/lowering deposition lift
        if (joy_msg.buttons[3]):
            if (self.deposit_lift_toggle == 100):
                self.deposit_lift_toggle = -100
            else:
                self.deposit_lift_toggle = 100
            self.pub_depositor_lift.publish(self.deposit_lift_toggle)

        self.prev_joy_msg = joy_msg
            

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
