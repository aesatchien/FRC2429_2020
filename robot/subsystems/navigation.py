# Attempt to convert 2019 Spartan Java to Python - 11/22/2019 CJH
import wpilib
import navx
from wpilib.command import Subsystem
from wpilib import SmartDashboard

class Navigation(Subsystem):
    def __init__(self, robot):
        super().__init__("navigation")
        self.counter = 0
        # pain to debug this, so use the connected attribute as a debug flag
        self.connected = True
        if self.connected:
            self.navx = navx.AHRS.create_spi()
            # Analog input, if we ever ned it
           # self.analog = wpilib.AnalogInput(navx.getNavxAnalogInChannel(0))
            self.analog = None
        else:
            self.navx = None
            self.analog = None

    def get_roll(self):
        return self.navx.getRoll()

    def get_yaw(self):
        if self.connected:
            return self.navx.getYaw()
        else:
            return 0

    def get_pitch(self):
        return self.navx.getPitch()

    def get_angle(self):
        if self.connected:
            return self.navx.getAngle()
        else:
            return 0

    def log(self):
        self.counter += 1
        if self.counter % 10 == 0:

            if self.connected:
                SmartDashboard.putBoolean("IsConnected", self.navx.isConnected())
                SmartDashboard.putNumber("Angle", self.navx.getAngle())
                SmartDashboard.putNumber("Pitch", self.navx.getPitch())
                SmartDashboard.putNumber("Yaw", self.navx.getYaw())
                SmartDashboard.putNumber("Roll", self.navx.getRoll())
                #SmartDashboard.putNumber("Analog", round(self.analog.getVoltage(),3))
                SmartDashboard.putNumber("Timestamp", self.navx.getLastSensorTimestamp())