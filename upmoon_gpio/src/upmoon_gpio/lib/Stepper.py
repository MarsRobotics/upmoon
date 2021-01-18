import Jetson.GPIO as GPIO
from rospy import sleep

class Stepper:

    def __init__(self, disable_pin, dir_pin, step_pin, steps_per_rev=200, revs_per_turn=60, delay=0.15):
        """
        A stepper motor class originally made for the Geckodrive G213V
            
        Dependencies:
            Jetson.GPIO

        Instance Variables:
            dis_pin: The pin for disabling the stepper
            dir_pin: Pin for setting the turning direction of the stepper
            step_pin: Pin for stepping the motor
            position: the number of steps the stepper has gone from its initial state
            delay: the smallest delay achievable by the motor. Will not allow a delay smaller than this
        """
        self.steps_per_turn = steps_per_rev*revs_per_turn #How many steps the motor must turn to turn the output device one full revolution
        self.curr_angle = 0
        self.step_count = 0
        self.position = 0
        self.direction = 0
        self.running = False

        self.dis_pin = disable_pin
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.delay = delay
        
        GPIO.setmode(GPIO.BOARD)

        try:
            GPIO.setup(disable_pin, GPIO.OUT, initial=GPIO.LOW)
        except ValueError as e:
            print('Disable pin failure: ', e)
        try:
            GPIO.setup(dir_pin, GPIO.OUT, initial=GPIO.LOW)
        except ValueError as e:
            print('Direction pin failure: ', e)
        try:
            GPIO.setup(step_pin, GPIO.OUT, initial=GPIO.LOW)
        except ValueError as e:
            print('Step pin failure: ', e)

        self.position = 0
        self.disable()#default the stepper to being powered off

    def enable(self):
        GPIO.output(self.dis_pin, GPIO.LOW)#holds position of the stepper

    def disable(self):
        GPIO.output(self.dis_pin, GPIO.HIGH)#disable the stepper. Let's it move freely, but does not take power

    def setAngle(self, angle):
        self.curr_angle = self.position * 360 / self.steps_per_turn
        self.step_count = (int)(self.steps_per_turn * (angle - self.curr_angle) / 360)
        self.direction = 0 if self.step_count > 0 else 1
        self.step_count = abs(self.step_count)
        if self.direction:
            GPIO.output(self.dir_pin, GPIO.HIGH)
        else:
            GPIO.output(self.dir_pin, GPIO.LOW)
        
        self.running = True
        self.enable()

    def step(self):
        if self.step_count <= 0:
            self.disable()
            self.running = False
            return
        GPIO.output(self.step_pin, GPIO.HIGH)
        sleep(self.delay/1000)
        GPIO.output(self.step_pin, GPIO.LOW)
        sleep(self.delay/1000)
        self.position += 1 if (self.direction == 0) else -1
        self.step_count -= 1

    def getAngle(self):
        self.curr_angle = self.position * 360 / self.steps_per_turn
        return self.curr_angle

    def isRunning(self):
        return self.running
