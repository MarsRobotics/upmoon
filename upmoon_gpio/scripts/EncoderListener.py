from rospy import sleep
import rospy
from std_msgs.msg import Float64, UInt16MultiArray
from threading import Thread

class EncoderListener(Thread):
    def __init__(self, pub_topic, ankle_name, sleep_rate: rospy.Rate):
        super().__init__()
        self.pub = rospy.Publisher(pub_topic, Float64, queue_size=10)
        self.currAngle = Float64()
        self.index = self.determineIndex(ankle_name)
        self.updated = True
        self.data = None
        self.sleep_rate = sleep_rate

        rospy.Subscriber('/arduino_potentiometer_values', UInt16MultiArray, self.topic_callback)

    #called whenever arduino sends new info
    def topic_callback(self, msg):
        self.data = msg.data[self.index]
        self.updated = False

    #turn arduino data (int: 0-1023) into an angle in radians
    def update(self, data):
        #TODO

        #temporary:
        return (data * 3.1415) / 1023

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
            try:
                self.sleep_rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                rospy.loginfo("Ended Thread Process")
                

        

        