from wpilib.command import Subsystem

'''
from adafruit_lidarlite import LIDARLite
import busio
import board
from wpilib import SmartDashboard

class Distance(Subsystem):
    def __init__(self, robot):
        Subsystem.__init__(self, "distance")

        self.robot = robot

        self.lidar_i2c = busio.I2C(board.SCL, board.SDA)
        self.lidar = LIDARLite(self.lidar_i2c)

    def get_distance(self):
        return self.lidar.distance
'''