from .MotorListener import MotorListener
import PyTrinamic
from PyTrinamic.connections.ConnectionManager import ConnectionManager
from PyTrinamic.modules.TMCM1670.TMCM_1670 import TMCM_1670
from PyTrinamic.modules.TMCM1670.TMCM_1670 import _AP_MOTOR_0
from PyTrinamic.modules.TMCM1670.TMCM_1670 import TMCM_1670_motor_interface

class TrinamicsMotor(MotorListener):

    def __init__(self, topic, motor_id=3, diameter=0.5, gear_ratio=60):
        super().__init__(topic)
        self.m_per_sec_convert = 3.14159 * diameter / (gear_ratio * 60)
        self.rpm_convert = gear_ratio * 60 / (3.14159 * diameter)
        msg = "--interface socketcan_tmcl --host-id 2 --module-id {module}".format(module = motor_id)
        connectionManager = ConnectionManager(msg.split())
        self.myInterface = connectionManager.connect()

        self.motorID = motor_id
        self.module = TMCM_1670(self.myInterface)

        # motor configuration
        #self.AP.MaxTorque
        self.module.motor(0).setAxisParameter(_AP_MOTOR_0.MaxTorque, 3000)#Sets max torque (current draw in mA)
        #self.module.showMotorConfiguration()

        # encoder configuration
        #self.module.showEncoderConfiguration()

        # motion settings
        self.module.motor(0).setAxisParameter(_AP_MOTOR_0.MaxVelocity, 4000)#Sets max velocity (in rpms)
        self.module.motor(0).setAxisParameter(_AP_MOTOR_0.Acceleration, 4000)#Sets max accel (in rpms/s)
        self.module.motor(0).setAxisParameter(_AP_MOTOR_0.EnableRamp, 1)#Sets accel ramp enabled
        self.module.motor(0).setAxisParameter(_AP_MOTOR_0.TargetReachedVelocity, 100)#Sets Target Reached Velocity (in rpms)
        self.module.motor(0).setAxisParameter(_AP_MOTOR_0.TargetReachedDistance, 1000)#Sets Target Reached Position (in encoder counts)

        #self.module.showMotionConfiguration()

        # PI configuration
        self.module.motor(0).setAxisParameter(_AP_MOTOR_0.TorqueP, 2000)#Sets Torque P parameter
        self.module.motor(0).setAxisParameter(_AP_MOTOR_0.TorqueI, 2000)#Sets Torque I parameter
        self.module.motor(0).setAxisParameter(_AP_MOTOR_0.VelocityP, 800)#Sets Velocity P parameter
        self.module.motor(0).setAxisParameter(_AP_MOTOR_0.VelocityI, 600)#Sets Velocity I parameter
        self.module.motor(0).setAxisParameter(_AP_MOTOR_0.PositionP, 300)#Sets Position P parameter

#        self.module.setTorquePParameter(2000) #4000 #2:000
#        self.module.setTorqueIParameter(2000) #2000
#        self.module.setVelocityPParameter(800) #1000
#        self.module.setVelocityIParameter(600) #500
        #self.module.showPIConfiguration()

        # use out_0 output for enable input (directly shortened)
#        self.module.setDigitalOutput(0)

        # sync actual position with encoder N-Channel 
        self.module.motor(0).setAxisParameter(_AP_MOTOR_0.ActualPosition, 0)#Sets Position parameter (motor thinks it is at position 0 on startup)
#        self.module.setActualPosition(0)

    def __del__(self):
        try:
            self.setTorque()#Sets the target torque(current) to 0
        except:
            pass
        self.myInterface.close()

    def getID(self):
        return int(self.motorID)

    def getTorque(self):
        val = self.module.motor(0).axisParameter(_AP_MOTOR_0.ActualTorque)
        return val if val < 2147483647 else (val - 2147483647*2)

    def getPosition(self):
        val = self.module.motor(0).axisParameter(_AP_MOTOR_0.ActualPosition)
        return val if val < 2147483647 else (val - 2147483647*2)

    def getVelocity(self):
        val = self.module.motor(0).axisParameter(_AP_MOTOR_0.ActualVelocity)
        return val if val < 2147483647 else (val - 2147483647*2)

    def getVoltage(self):
        return float(self.module.motor(0).axisParameter(_AP_MOTOR_0.SupplyVoltage))/10

    def setPosition(self, pos):
        self.module.motor(0).setAxisParameter(_AP_MOTOR_0.TargetPosition, int(pos))

    def setVelocity(self, rpm):
        self.module.motor(0).setAxisParameter(_AP_MOTOR_0.TargetVelocity, int(rpm))

    """Converts rpms to meters per second"""
    def setVelocityMS(self, vel):
        rpm = vel * self.rpm_convert
        self.module.motor(0).setAxisParameter(_AP_MOTOR_0.TargetVelocity, int(rpm))

    """Disables the motor. Does not allow setting the torque because:
        Setting motor torque is both ineffectual and dangerous"""
    def setTorque(self):
        self.module.motor(0).setAxisParameter(_AP_MOTOR_0.TargetTorque, 0)

    def update(self, data):
        self.setVelocityMS(data)

    def loop(self):
        pass
