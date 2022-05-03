from .MotorListener import MotorListener
from pysabertooth import Sabertooth
import rospy
import math

class DCBrushed(MotorListener):

    def __init__(self, topic, address, motor_num, sleep_rate: rospy.Rate, max_power: float = 100):
        """
        A DC brushed motor class for sabertooth motor controllers in packetized serial
        
        Dependencies: 
            pysabertooth (https://github.com/MomsFriendlyRobotCompany/pysabertooth)

        Instance variables:
            topic = the ROS topic to subscribe to
            address = address of the sabertooth motor controller in packetized serial mode 
            motor_num = 1 or 2; the port number of the motor controller of the sabertooth;
                        there are two motors available for each controller
            max_power = a number between (0, 100] that represents the maximum
                        percent limit of the motor output. (E.g. 50 limits the output to
                        self.drive ranges of [-50, 50])
        """
        super().__init__(topic)
        self.address = address
        self.motor_num = motor_num
        self.motor = Sabertooth('/dev/ttyS0', baudrate=9600, address=self.address)

        self.sleep_rate = sleep_rate

        if max_power <= 0:
            raise RuntimeError("Cannot have a max power of zero or fewer")
        elif max_power > 100:
            raise RuntimeError("Cannot have a max power greater than 100")
        self.max_power = max_power

    def drive(self, speed):
        """
        Sets the motor to a given power

        speed = a value in the range [-100, 100]
        """
        if abs(speed) > self.max_power:
            speed = math.copysign(self.max_power, speed)

        self.motor.drive(self.motor_num, speed)

    def update(self, data):
        self.drive(data)

    def loop(self):
        self.sleep_rate.sleep()

    def on_exit(self):
        pass
