# Attempt to convert 2019 Spartan Java to Python - 11/22/2019 CJH
import wpilib
from wpilib.command import Subsystem
from wpilib import DoubleSolenoid, Compressor
from wpilib import SmartDashboard

class Pneumatics(Subsystem):
    def __init__(self, robot):
        Subsystem.__init__("pneumatics")
        self.counter = 0
        self.double_solenoid = DoubleSolenoid(0,1)
        self.compressor = Compressor(0)
        self.compressor.setClosedLoopControl(True)

    def actuate_solenoid(self, direction=None):
        if direction == 'open':
            self.double_solenoid.set(DoubleSolenoid.Value.kForward)
        elif direction == 'close':
            self.double_solenoid.set(DoubleSolenoid.Value.kReverse)
        else:
            pass

    def log(self):
        pass