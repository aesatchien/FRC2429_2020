# Ball Handling subsystem - should be gate and intake
import wpilib
from wpilib.command import Subsystem
from wpilib import Spark


class Intake(Subsystem):
    def __init__(self, robot):
        Subsystem.__init__(self, "intake")
        self.robot = robot
        self.intake_spark = Spark(6)
        self.counter = 0
        # TODO: set a default command so we are always closing the intake

    def initDefaultCommand(self):
        """
        When other commands aren't using the gate, keep it closed (or rely on spring)
        Let it reset every 60s - should not be a problem
        """

    def run_intake(self, power=0):
        self.intake_spark.set(power)


    def log(self):
        self.counter += 1
        if self.counter % 10 == 0:
            pass