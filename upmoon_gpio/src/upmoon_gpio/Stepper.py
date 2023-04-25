from .MotorListener import MotorListener
import RPi.GPIO as GPIO
from rospy import sleep
import rospy
from std_msgs.msg import Float64
import math
from threading import Lock

class Stepper(MotorListener):

    # Keeps track of total motors spawned and disabed
    motor_count_lock = Lock()
    total_motors = 0
    disabled_motors = 0
    current_encoder_angle = 0

    def __init__(self, topic, disable_pin, dir_pin, step_pin, sleep_rate: rospy.Rate, steps_per_rev=200, revs_per_turn=60, delay=0.3):
        """
        A stepper motor class originally made for the Geckodrive G213V
            
        Dependencies:
            RPi.GPIO

        Instance Variables:
            topic: The ROS topic to subscribe to
            encoder_topi: the encoder to subscribe to
            disable_pin: The pin for disabling the stepper
            dir_pin: Pin for setting the turning direction of the stepper
            step_pin: Pin for stepping the motor
            steps_per_rev: the number of steps the stepper the stepper must take to complete one revolution of its rotor
            revs_per_turn: How many revolutions of the motor rotor for one full rotation of the geabox rotor
            delay: the smallest delay achievable by the motor. Will not allow a delay smaller than this
            [TODO] reverse_dir: If true, reverses the default direction of the motor.
        """
        super().__init__(topic)
        GPIO.setwarnings(False)  # Disable warnings from multiple motors using same enable pin.
        self.steps_per_turn = steps_per_rev*revs_per_turn #How many steps the motor must turn to turn the output device one full revolution
        self.curr_angle = 0
        self.step_count = 0
        self.position = 0
        self.direction = 0
        self.running = False
        GPIO.setmode(GPIO.BCM)

        encoder_topic = ""
        if "ankle_lf_joint" in topic:
            encoder_topic = "/motor/encoder_lf"
            rospy.Subscriber(encoder_topic, Float64, self.encoderCall)
        if "ankle_lb_joint" in topic:
            encoder_topic = "/motor/encoder_lb"
            rospy.Subscriber(encoder_topic, Float64, self.encoderCall)
        if "ankle_rf_joint" in topic:
            encoder_topic = "/motor/encoder_rf"
            rospy.Subscriber(encoder_topic, Float64, self.encoderCall)
        if "ankle_rb_joint" in topic:
            encoder_topic = "/motor/encoder_rb"
            rospy.Subscriber(encoder_topic, Float64, self.encoderCall)




        self.dis_pin = disable_pin
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.delay = delay
        try:
            GPIO.setup(disable_pin, GPIO.OUT)
        except Exception:
            print('Disable pin failure')
        try:
            GPIO.setup(dir_pin, GPIO.OUT)
        except Exception:
            print('Direction pin failure')
        try:
            GPIO.setup(step_pin, GPIO.OUT)
        except Exception:
            print('Step pin failure')

        self.position = 0
        self.sleep_rate = sleep_rate

        # Add this stepper to the count
        with Stepper.motor_count_lock:
            Stepper.total_motors += 1
        
        self.disable()
        

    def enable(self):
        with Stepper.motor_count_lock:
            Stepper.disabled_motors -= 1

            if Stepper.disabled_motors == (Stepper.total_motors - 1):
                rospy.logdebug("ENABLE")
                GPIO.output(self.dis_pin, 0)#holds position of the stepper
                sleep(1) # Need to wait until the Gecko is enabled

    def disable(self):
        with Stepper.motor_count_lock:
            Stepper.disabled_motors += 1

            if Stepper.disabled_motors == Stepper.total_motors:
                GPIO.output(self.dis_pin, 1)#disable the stepper. Let's it move freely, but does not take power
                rospy.logdebug("DISABLE")

    def setAngle(self, rad):
        # Convert radians to degrees
        angle = rad * 180 / math.pi

        #does encoder account for negative in other direciton
        self.curr_angle = self.current_encoder_angle * self.position
        # self.curr_angle = self.current_encoder_angle
        self.step_count = int(self.steps_per_turn * (angle - self.curr_angle) / 360)
        
        msg = "ToRad: %f ToDeg: %d Steps: %d CurrDeg: %d" % (rad, angle, self.step_count, self.curr_angle)
        rospy.logdebug(msg)

        self.direction = 0 if self.step_count > 0 else 1
        self.step_count = abs(self.step_count)
        GPIO.output(self.dir_pin, self.direction)
        self.running = True
        self.enable()

    def step(self):
        if self.step_count <= 0:
            self.disable()
            self.running = False
            return
        GPIO.output(self.step_pin, 1)
        sleep(self.delay/1000)
        GPIO.output(self.step_pin, 0)
        sleep(self.delay/1000)
        self.position += 1 if (self.direction == 0) else -1
        self.step_count -= 1

    def getAngle(self):
        # self.curr_angle = self.position * 360 / self.steps_per_turn
        self.curr_angle = self.current_encoder_angle
        return self.curr_angle

    def isRunning(self):
        return self.running

    def update(self, data):
        self.setAngle(data)

    def loop(self):
        if self.running:
            self.step()
        else:
            self.sleep_rate.sleep()

    def on_exit(self):
        pass

    def encoderCall(self, data):
        self.current_encoder_angle = data
