# Attempt to convert 2019 Spartan Java to Python - 11/22/2019 CJH
import wpilib
from wpilib.command import Subsystem
from wpilib import SmartDashboard
from ctre import VictorSPX
from ctre import ControlMode
import math
from networktables import NetworkTables

class Climber(Subsystem):
    def __init__(self, robot):
        Subsystem.__init__(self, "climber")
        self.hook_lifter = VictorSPX(7)
        self.counter = 0
        self.hook_max_power = 0.5

    def raise_hook(self, power=0):
        self.hook_lifter.set(ControlMode.PercentOutput, power)

    def lower_hook(self, power=0):
        self.hook_lifter.set(ControlMode.PercentOutput, power)

    def stop_hook(self):
        self.hook_lifter.set(ControlMode.PercentOutput, 0)

    def log(self):
        self.counter += 1
        if self.counter % 10 == 0:
            pass
            #SmartDashboard.putNumber("Cam distance", self.ball_table.getNumber("distance", 0))