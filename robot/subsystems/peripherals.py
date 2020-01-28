# Attempt to convert 2019 Spartan Java to Python - 11/22/2019 CJH
import wpilib
from wpilib.command import Subsystem
from wpilib import Spark
from wpilib import Servo
from rev.color import ColorSensorV3
from rev.color import ColorMatch
from wpilib import Color
from wpilib import I2C
from wpilib import SmartDashboard

class Peripherals(Subsystem):
    def __init__(self, robot):
        super().__init__("peripherals")
        self.intake_spark = Spark(6)
        self.control_panel_spark = Spark(5)
        self.left_dispenser_gate = Servo(7)
        self.right_dispenser_gate = Servo(8)
        self.color_sensor = ColorSensorV3(I2C.Port.kOnboard)
        self.color_matcher = ColorMatch()


        self.kBlueTarget = Color(0.143, 0.427, 0.429)
        self.kGreenTarget = Color(0.197, 0.561, 0.240)
        self.kRedTarget = Color(0.561, 0.232, 0.114)
        self.kYellowTarget = Color(0.361, 0.524, 0.113)
        self.color_matcher.addColorMatch(self.kBlueTarget)
        self.color_matcher.addColorMatch(self.kGreenTarget)
        self.color_matcher.addColorMatch(self.kRedTarget)
        self.color_matcher.addColorMatch(self.kYellowTarget)

    def run_intake(self, power=0):
        self.intake_spark.set(power)

    def run_spinner(self, power=0):
        self.control_panel_spark.set(power)

    def close_gate(self):
        self.left_dispenser_gate.setAngle(120)
        self.right_dispenser_gate.setAngle(135)

    def open_gate(self):
        self.left_dispenser_gate.setAngle(0)
        self.right_dispenser_gate.setAngle(0)

    def panel_clockwise(self, power):
        print(power)
        self.control_panel_spark.set(power)



    def log(self):
        detected_color = self.color_sensor.getColor()
        match = self.color_matcher.matchClosestColor(detected_color, 0.1)
        color_string = ''
        if match == self.kBlueTarget:
            color_string = 'blue'
        elif match == self.kGreenTarget:
            color_string = 'green'
        elif match == self.kRedTarget:
            color_string = 'red'
        elif match == self.kYellowTarget:
            color_string = 'yellow'
        SmartDashboard.putString('Detected Color', color_string)
        SmartDashboard.putNumber("Red", detected_color.red)
        SmartDashboard.putNumber("Green", detected_color.green)
        SmartDashboard.putNumber("Blue", detected_color.blue)
        #SmartDashboard.putNumber("Confidence", match.confidence)
