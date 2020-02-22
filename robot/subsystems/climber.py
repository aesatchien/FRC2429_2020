# Attempt to convert 2019 Spartan Java to Python - 11/22/2019 CJH
import wpilib
from wpilib.command import Subsystem
from wpilib import SmartDashboard
from wpilib import Spark
from wpilib import Servo
from wpilib import Encoder
from ctre import VictorSPX
from ctre import ControlMode
import math
from networktables import NetworkTables

class Climber(Subsystem):
    def __init__(self, robot):
        Subsystem.__init__(self, "climber")
        self.hook_lifter = VictorSPX(7)
        self.hook_encoder = Encoder(0, 1)
        self.robot_lifter = Spark(4)
        self.left_robot_roller = VictorSPX(8)
        self.right_robot_roller = VictorSPX(9)
#        self.locking_servo = Servo(1)
        self.counter = 0
        self.hook_max_power = 0.5
        self.hook_encoder.reset()

    def raise_hook(self, power=0):
        self.hook_lifter.set(ControlMode.PercentOutput, power)

    def lower_hook(self, power=0):
        self.hook_lifter.set(ControlMode.PercentOutput, power)

    def stop_hook(self):
        self.hook_lifter.set(ControlMode.PercentOutput, 0)

    def raise_robot(self, power=0):
        self.robot_lifter.set(power)

    def lock_robot(self):
        self.locking_servo.setAngle(0)

    def stop_winch(self):
        self.robot_lifter.set(0)

    def robot_roller_left(self, power=0):
        self.left_robot_roller(ControlMode.PercentOutput, power)

    def robot_roller_right(self, power=0):
        self.right_robot_roller(ControlMode.PercentOutput, power)

    def robot_roller_stop(self):
        self.right_robot_roller(ControlMode.PercentOutput, 0)



    def log(self):
        self.counter += 1
        if self.counter % 10 == 0:
            SmartDashboard.putNumber("climber position", self.hook_encoder.get())
            pass
            #SmartDashboard.putNumber("Cam distance", self.ball_table.getNumber("distance", 0))