from abc import ABC, abstractmethod
import rospy
from std_msgs.msg import Float64
from threading import Thread

class MotorListener(ABC, Thread):

    """
    Parmmeters:
        topic: String for ROS topic name
        updated: True if we have a new value to pass to motor, else False
        data: Last message from the topic stored
    """
    def __init__(self, topic: str):
        super().__init__()
        rospy.Subscriber(topic, Float64, self.topic_callback)
        self.updated = True
        self.data = None
        self.stop = False


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

    """
    Gets called when the thread is finished
    """
    @abstractmethod
    def on_exit(self):
        pass
    
    def run(self):
        while not rospy.is_shutdown() and not self.stop:
            if not self.updated:
                self.updated = True
                self.update(self.data)
            self.loop()
        self.on_exit()
