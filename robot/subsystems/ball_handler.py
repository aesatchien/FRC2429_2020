# Ball Handling subsystem - should be gate and intake
import wpilib
from wpilib.command import Subsystem
from wpilib import Encoder
from wpilib import Spark
from wpilib import SmartDashboard
from commands.actuate_gate import ActuateGate
import math
from networktables import NetworkTables

class Ball_Handler(Subsystem):
    def __init__(self, robot):
        Subsystem.__init__(self, "ball_handler")
        self.robot = robot
        self.intake_spark = Spark(6)
        self.hopper_spark = Spark(7)
        self.counter = 0
        self.gate_encoder = Encoder(8, 9)
        # TODO: set a default command so we are always closing the intake

    def initDefaultCommand(self):
        """
        When other commands aren't using the gate, keep it closed (or rely on spring)
        Let it reset every 60s - should not be a problem
        """
        self.setDefaultCommand(ActuateGate(self.robot, direction='hold', button=None, timeout=60))

    def run_intake(self, power=0):
        self.intake_spark.set(power)

    def hold_gate(self):
        self.hopper_spark.set(0.15)

    def close_gate(self):
        print('closing gate')
        self.hopper_spark.set(0.3)

    def open_gate(self):
        print('opening gate')
        self.hopper_spark.set(-0.3)

    def log(self):
        self.counter += 1
        if self.counter % 10 == 0:
            SmartDashboard.putNumber("Gate Pos", self.gate_encoder.get())
            #SmartDashboard.putNumber("Cam distance", self.ball_table.getNumber("distance", 0))