# Attempt to convert 2019 Spartan Java to Python - 11/22/2019 CJH
import wpilib
from wpilib.command import Subsystem
from wpilib import Spark
from wpilib.smartdashboard import SmartDashboard

class Peripherals(Subsystem):
    def __init__(self, robot):
        super().__init__()
        self.intake_spark = Spark(1)
        self.control_panel_spark = Spark(0)

    def run_intake(self, power=0):
        self.intake_spark.set(power)

    def run_spinner(self, power=0):
        self.control_panel_spark.set(power)

    def log(self):
        pass