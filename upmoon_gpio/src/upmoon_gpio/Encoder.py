from .MotorListener import MotorListener
import RPi.GPIO as GPIO
from rospy import sleep
import rospy
import math
import Encoder
from std_msgs.msg import Float64, UInt16MultiArray
from threading import Lock

class Encoder(MotorListener):

    def __init__(self, topic, ankle_name, pinA, pinB, sleep_rate: rospy.Rate):
        """
        Encoder class meant to init the encoders which will be used by the stepper motors so that 
        the robot will know its position at all times. Will help with any splaying issues and will 
        make packing in/out and automation work easier
        """
        super().__init__()
        self.pub = rospy.Publisher(topic, Float64, queue_size=10)
        self.currAngle = Float64()
        self.index = self.determineIndex(ankle_name)
        self.updated = True
        self.data = None
        self.encoder = Encoder.Encoder(pinA,pinB)
        self.sleep_rate = sleep_rate

    #called whenever raspPie sends new info
    #do we need with encoder library?
    def topic_callback(self, msg):
        self.data = msg.data[self.index]
        self.updated = False

    #reads the encoder info and then returns the degree
    def update(self, data):
        resolution = 2048
        quadRes = resolution * 4
        anglePerRot = quadRes / 360
        currentPos = self.encoder.read()
        #could have line to check if encoder has done full rotation
        #and have math to account for that but mechanically that is impossible
        return currentPos * anglePerRot

    #map indexes of arduino message to ankle names
    def determineIndex(self, ankle):
        if ankle == "lf":
            return 0
        elif ankle == "lb":
            return 1
        elif ankle == "rf":
            return 2
        elif ankle == "rb":
            return 3
        else:
            print("Invalid ankle name")

    #run function is used for threading
    def run(self):
        while not rospy.is_shutdown():
            if not self.updated:
                self.updated = True
                self.currAngle.data = self.update(self.data)
                self.pub.publish(self.currAngle.data)
            self.sleep_rate.sleep()