from .MotorListener import MotorListener
import RPi.GPIO as GPIO
import rospy

class PWM(MotorListener):

    MIN_RANGE = 0
    MAX_RANGE = 100

    def __init__(self, topic, pin, freq, min_dc, max_dc, init_range, sleep_rate: rospy.Rate, invert=False):
        """
        A PWM class that sets RPi pin to specified duty cycle and freqency
            
        Dependencies:
            RPi.GPIO
        Parameters:
            topic: name of ROS topic to subscribe to
            pin: Board pin of signal (BCM)
            min_dc: Minimum duty cycle of motor (%)
            max_dc: Maximum duty cycle of motor (%)
            init_range: Initial % of the duty cycle range (%)
                        0 = min_dc, 100 = max_dc
            freq: frequency of PWM (hz)
            invert: reverses PWM range mapping
        """
        super().__init__(topic)
        if (init_range < self.MIN_RANGE or init_range > self.MAX_RANGE):
            raise ValueError("init DC is not between min and max dc")
        self.min_dc = min_dc
        self.max_dc = max_dc
        self.init_range = init_range
        self.invert = invert
        try:
            GPIO.setmode(GPIO.BCM)
        except Exception:
            print('GPIO failure')
        try:
            GPIO.setup(pin, GPIO.OUT, initial = GPIO.LOW)
        except Exception:
            print('Pin setup failure')

        self.pwm = GPIO.PWM(pin, freq)
        self.pwm.start(self.convertRangeToDutyCycle(init_range))
        self.sleep_rate = sleep_rate

    def convertRangeToDutyCycle(self, percent):
        if (percent < self.MIN_RANGE or percent > self.MAX_RANGE):
            print('Servo set out of bounds')
            percent = self.init_range
        dc = (percent * (self.max_dc - self.min_dc) / self.MAX_RANGE) + self.min_dc
        return self.max_dc + self.min_dc - dc if (self.invert) else (dc)
        
    def setPosition(self, percent):
        self.pwm.ChangeDutyCycle(self.convertRangeToDutyCycle(percent))

    def disable(self):
        # TODO: Doesn't disable if turning. Maybe try setting frequency to 0?
        self.pwm.ChangeDutyCycle(0)

    def update(self, data):
        self.setPosition(data)

    def loop(self):
        self.sleep_rate.sleep()

    def on_exit(self):
        pass
