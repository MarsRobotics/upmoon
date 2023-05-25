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
    ANKLE_PACK_IN = 3.14
    ANKLE_PACK_OUT = 3.14
    ANKLE_TURN = 2.00
    DIGGING_ANGLE_COEF = -5
    DIGGING_SPEED_MAX = 15 # Max percentage of power for digging
    DIGGING_SPEED_INCREMENT = 5 # increments dig speed
    DIGGING_SPEED_MIN = 10 
    DEPOSIT_ACT_TOGGLE_INIT = 0
    DEPOSIT_LIFT_TOGGLE_INIT = 100
    
    WHEEL_TURN_COEF = 1

    #conditions for dumping system
    #reset: step up: 3.14
    #step down: 0
    DEPOSIT_STEPPER_UP = 2.2
    #DEPOSIT_STEPPER_UP = -0.5
    DEPOSIT_STEPPER_DOWN = 0.25
    #DEPOSIT_STEPPER_DOWN = 0
    DEPOSIT_STEPPER_PACK = 0
    
    DEPOSIT_TOGGLE = 0

    def __init__(self):
        self.pub_twist              = rospy.Publisher('/diff_drive_controller/cmd_vel', Twist, queue_size=10)

        self.articulate_action      = SimpleActionClient('action_articulate', ArticulateAction)
        self.articulate_event       = threading.Event()
        
        self.pub_dig_spin           = rospy.Publisher('/motor/dig_spin', Float64, queue_size=10)
        #self.pub_dig_angle_speed    = rospy.Publisher('/motor/dig_angle_joint', Float64, queue_size=10)
        self.pub_dig_angle_speed    = rospy.Publisher('/dig_angle_controller/command', Float64, queue_size=10)

        #these were for old dump system
        # self.pub_depositor_actuator = rospy.Publisher('/motor/depositor_actuator', Float64, queue_size=10)
        # self.pub_depositor_lift     = rospy.Publisher('/motor/depositor_lift', Float64, queue_size=10)
        
        #these are for the stepper motor lift implementation
        self.pub_depositor_lift_l = rospy.Publisher('/motor/deposit_lift_l', Float64, queue_size=10)
        self.pub_depositor_lift_r = rospy.Publisher('/moror/deposit_lift_r', Float64, queue_size=10)
        self.pub_depositor_actuator = rospy.Publisher('/motor/dump_door', Float64, queue_size=10) #not sure if this is right

        # self.deposit_act_toggle = self.DEPOSIT_ACT_TOGGLE_INIT
        # self.deposit_lift_toggle = self.DEPOSIT_LIFT_TOGGLE_INIT
        #start in the down position
        #self.deposit_lift_toggle = self.DEPOSIT_STEPPER_DOWN
        self.deposit_lift_toggle = self.DEPOSIT_STEPPER_PACK
        self.digging_counter = 0
        self.wheel_control = False

        # self.pub_depositor_actuator.publish(self.deposit_act_toggle)
        # self.pub_depositor_lift.publish(self.deposit_lift_toggle)

        self.prev_joy_msg = None
        
        #initial radians of all motors being saved
        #used to keep track of ankle radian
        self.lf_prev_rad = 3.14
        self.lb_prev_rad = 3.14
        self.rf_prev_rad = 3.14
        self.rb_prev_rad = 3.14
        
        #booleans for deciding which ankle is being controlled individually
        self.lf_on = False
        self.rf_on = False
        self.lb_on = False
        self.rb_on = False
        
        #value for ankle to turn
        self.wheel_value = 0
        
        #value we want ankle to be at
        self.goal_val = 0.0
        
        #values for how much the ankle should rotate
        self.small_change = 0.1
        self.big_change = 0.5

        rospy.Subscriber('joy', Joy, self.joy_callback)
    

    """
    Set all A joints to a radian
    """
    #Allows for individual ankle motor to be changed (DOES NOT FUNCTION PERFECTLY)
    #stored desired radian in global value and uses those values accordingly
    def ankle_set_individual(self,ankle: int, rad: float):
            
        if ankle == 0: #no ankle selected
            return
             
        elif ankle == 1: #left front ankle
            self.lf_prev_rad = rad

        elif ankle == 2: #right front ankle
            self.rf_prev_rad = rad 
     	     
        elif ankle == 3: #left back ankle
            self.lb_prev_rad = rad
     	     
        else: #right back ankle
            self.rb_prev_rad = rad
     	     
        goal = ArticulateGoal(lf=self.lf_prev_rad,
                              lb=self.lb_prev_rad,
                              rf=self.rf_prev_rad,
                              rb=self.rb_prev_rad)

        self.articulate_event.set()

         # Send goal to the action server and ignore the passed args in the callback.
        self.articulate_action.send_goal(goal, done_cb=lambda *_: self.articulate_event.clear())
         
    def ankle_set(self, rad: float):
        self.lf_prev_rad = rad
        self.lb_prev_rad = rad
        self.rf_prev_rad = rad
        self.rb_prev_rad = rad
        goal = ArticulateGoal(lf=rad,
                              lb=rad,
                              rf=rad,
                              rb=rad)

        self.articulate_event.set()

        # Send goal to the action server and ignore the passed args in the callback.
        self.articulate_action.send_goal(goal, done_cb=lambda *_: self.articulate_event.clear())
       


    def turn_state(self):
        self.lf_prev_rad = self.ANKLE_TURN
        self.lb_prev_rad = self.ANKLE_TURN
        self.rf_prev_rad = self.ANKLE_TURN
        self.rb_prev_rad = self.ANKLE_TURN
        goal = ArticulateGoal(lf=self.ANKLE_TURN,
                              lb=self.ANKLE_TURN,
                              rf=self.ANKLE_TURN,
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
        
        if joy_msg.buttons[9]: #if start then control each wheel
            if not self.wheel_control:
                self.wheel_control = True
                self.wheel_value = 0
                self.goal_val = 3.14
            else:
                self.wheel_control = False
            return
        #if in secondary wheel
        if self.wheel_control:
            if joy_msg.buttons[4]: #left front wheel
                self.wheel_value = 1
                self.goal_val = self.lf_prev_rad
            
            if joy_msg.buttons[5]: #right front wheel
                self.wheel_value = 2
                self.goal_val = self.rf_prev_rad
                   
            if joy_msg.buttons[6]: #left back wheel
                self.wheel_value = 3
                self.goal_val = self.lb_prev_rad
                   
            if joy_msg.buttons[7]: #right back wheel
                self.wheel_value = 4
                self.goal_val = self.rb_prev_rad
            
            
            if joy_msg.buttons[15]: #hit left move -.1 rads
                self.goal_val -= self.small_change
            if joy_msg.buttons[16]: #hit right move .1 rads
                self.goal_val += self.small_change
            if joy_msg.buttons[13]: #hit up move up .5
                self.goal_val += self.big_change
            if joy_msg.buttons[14]: #hit down by .5
                self.goal_val -= self.big_change
            
            #move ankle
            self.ankle_set_individual(self.wheel_value, self.goal_val)
            
            #sets the dumping box to the starting position and resting position
            if joy_msg.buttons[3]:
                if (self.deposit_lift_toggle == self.DEPOSIT_STEPPER_PACK):
                    self.deposit_lift_toggle = self.DEPOSIT_STEPPER_DOWN
                else:
                    self.deposit_lift_toggle = self.DEPOSIT_STEPPER_PACK
                self.pub_depositor_lift_r.publish(self.deposit_lift_toggle)
                self.pub_depositor_lift_l.publish(self.deposit_lift_toggle)

            
	        
            
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
            # stop as both bumpers are pressed
            self.digging_counter = 0
            self.pub_dig_spin.publish(Float64(0))
            pass
        elif (joy_msg.buttons[4]):
            # left bumper was pressed => stop digger
            self.digging_counter = 0
            self.pub_dig_spin.publish(Float64(0))
        elif (joy_msg.buttons[5]):
            # right bumper was pressed => forward
	    # increments diggering power by 5. Starts from 0, to 10, to 15, 20, 25, 30
            self.digging_counter += 1
            if self.digging_counter > 5:
                self.digging_counter = 1
            self.pub_dig_spin.publish(Float64(10 + ((self.digging_counter - 1) * 5)))

        # Right analog stick to raise/lower digger
        r_stick_y = joy_msg.axes[4]
        dig_angle_speed = r_stick_y * self.DIGGING_ANGLE_COEF
        self.pub_dig_angle_speed.publish(Float64(dig_angle_speed))

        # Use Triangle / Y Button for opening/closing deposition (press to open and press to close)
        if (joy_msg.buttons[2]):
            # if (self.deposit_act_toggle == 0) :
                # self.deposit_act_toggle = 100
            # else:
                # self.deposit_act_toggle = 0
            # self.pub_depositor_actuator.publish(self.deposit_act_toggle)
            if (self.DEPOSIT_TOGGLE == 0):
                self.DEPOSIT_TOGGLE = 10
                self.pub_depositor_actuator.publish(1.0)
            else:
                self.DEPOSIT_TOGGLE = 0
                self.pub_depositor_actuator.publish(0.0)
            #self.pub_depositor_actuator.publish(1.0)
        #self.pub_depositor_actuator.publish(0)

        # Use the Square Button for raising/lowering deposition lift
        if (joy_msg.buttons[3]):
        #roll back: lift toggle = -2.25
        # and 0 for the second
            if (self.deposit_lift_toggle == self.DEPOSIT_STEPPER_DOWN):
                self.deposit_lift_toggle = self.DEPOSIT_STEPPER_UP
            else:
                self.deposit_lift_toggle = self.DEPOSIT_STEPPER_DOWN
            self.pub_depositor_lift_r.publish(self.deposit_lift_toggle)
            self.pub_depositor_lift_l.publish(self.deposit_lift_toggle)


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
