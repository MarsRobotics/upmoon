from abc import ABC, abstractmethod
import rospy
from std_msgs.msg import Float64

class MotorListener(ABC):

    """
    Parmmeters:
        topic: String for ROS topic name
        updated: True if we have a new value to pass to motor, else False
        data: Last message from the topic stored
    """
    def __init__(self, topic: str):
        rospy.Subscriber(topic, Float64, self.topic_callback)
        self.updated = True
        self.data = None


    def topic_callback(self, data):
        if self.data == data.data:
            return
        self.data = data.data # Unpack std_msgs.Float64 to float
        self.updated = False


    """
    Gets called only when we have a new message for motor
    """
    @abstractmethod
    def update(self, data):
        pass

    """
    Gets called on every cycle of loop
    """
    @abstractmethod
    def loop(self):
        pass

    
    def run(self):
        if not self.updated:
            self.updated = True
            self.update(self.data)
        self.loop()

