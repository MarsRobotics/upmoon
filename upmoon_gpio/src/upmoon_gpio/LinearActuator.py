from .MotorListener import MotorListener 
import RPi.GPIO as GPIO 
from rospy import sleep #don't actually need this? 
import rospy 
import math 
from threading import Lock #don't actually need this? 

class LinearActuator(MotorListener):

    def __init__(self, topic, act_pin, sleep_rate: rospy.Rate):
        super().__init__(topic) 
        GPIO.setwarnings(False)

        self.act_pin = act_pin
        self.sleep_rate = sleep_rate

        try:
            GPIO.setup(act_pin, GPIO.OUT)
        except Exception: 
            print('Actuator pin failure')

    
    def actuate(self):
        GPIO.output(self.act_pin, 1)
        sleep(10)
        stopActuate()

    def stopActuate(self):
        GPIO.output(self.act_pin, 0)

    def update(self, data):
        if data == 1:
            self.actuate()
        else:
            self.stopActuate()

    def loop(self):
        self.sleep_rate.sleep()

    def on_exit(self):
        pass

