# Attempt to convert 2019 Spartan Java to Python - 11/22/2019 CJH
import wpilib
from wpilib.command import Subsystem
from wpilib import Encoder
from wpilib import Spark
from wpilib import SmartDashboard
import math
from networktables import NetworkTables

class Ball_Handler(Subsystem):
    def __init__(self, robot):
        Subsystem.__init__(self, "ball_handler")
        self.intake_spark = Spark(6)
        self.hopper_spark = Spark(7)
        self.counter = 0
        self.gate_encoder = Encoder(8,9)
        self.gate_encoder_initialized = False


    def run_intake(self, power=0):
        self.intake_spark.set(power)

    def close_gate(self):
        self.hopper_spark.set(-0.3)

    def open_gate(self):
        self.hopper_spark.set(0.3)

    def gate_pos(self):
        return self.gate_encoder.get()

    def gate_power(self, power):
        self.hopper_spark.set(power)

    def log(self):
        self.counter += 1
        if self.counter % 10 == 0:
            SmartDashboard.putNumber("Gate Pos", self.gate_encoder.get())
            #SmartDashboard.putNumber("Cam distance", self.ball_table.getNumber("distance", 0))