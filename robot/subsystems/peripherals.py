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
import math

class Peripherals(Subsystem):
    def __init__(self, robot):
        Subsystem.__init__(self, "peripherals")
        self.intake_spark = Spark(6)
        self.control_panel_spark = Spark(5)
        self.left_dispenser_gate = Servo(7)
        self.right_dispenser_gate = Servo(8)
        self.counter = 0
        self.color_sensor = ColorSensorV3(I2C.Port.kOnboard)
        #self.color_sensor.setGain(ColorSensorV3.GainFactor.k1x)
        self.color_matcher = ColorMatch()
        # we can config the colorsensor resolution and the rate
        #self.color_sensor.configureColorSensor(res=, rate=)

        # need to put these numbers in for ourselves by positioning the sensor over the target and recording the RGB
        self.kBlueTarget = Color(0.181, 0.453, 0.365) #Color(0.143, 0.427, 0.429)
        self.kGreenTarget = Color(0.215, 0.529, 0.255) #Color(0.197, 0.561, 0.240)
        self.kRedTarget = Color(0.417, 0.398, 0.184) #Color(0.561, 0.232, 0.114)
        self.kYellowTarget = Color(0.326, 0.519, 0.154) #Color(0.361, 0.524, 0.113)
        self.color_dict = {"blue":self.kBlueTarget, "green":self.kGreenTarget, "red":self.kRedTarget, "yellow":self.kYellowTarget}
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
        self.control_panel_spark.set(power)

    def get_color_str(self, color=None, match_confidence=0.5):
        detected_color = color or self.color_sensor.getColor()

        '''
        if color: 
            detected_color = color
        else: 
            detected_color = self.color_sensor.getColor() 
        '''

        match = self.color_matcher.matchClosestColor(detected_color, match_confidence)
        color_string = 'No Match'
        if match == self.kBlueTarget:
            color_string = 'blue'
        elif match == self.kGreenTarget:
            color_string = 'green'
        elif match == self.kRedTarget:
            color_string = 'red'
        elif match == self.kYellowTarget:
            color_string = 'yellow'

        for key in self.color_dict:
            match_confidence = self.color_distance(detected_color, self.color_dict[key])
            if match_confidence < 0.05:
                color_string = key
                break
            else:
                color_string = "No Match"

        return color_string

    def log(self):
        self.counter += 1
        if self.counter % 5 == 0:
            detected_color = self.color_sensor.getColor()
            match_confidence = 0.5
            color_string = self.get_color_str(detected_color, match_confidence)

            '''
            match = self.color_matcher.matchClosestColor(match_confidence, detected_color)
            color_string = 'No Match'
            if match == self.kBlueTarget:
                color_string = 'blue'
            elif match == self.kGreenTarget:
                color_string = 'green'
            elif match == self.kRedTarget:
                color_string = 'red'
            elif match == self.kYellowTarget:
                color_string = 'yellow'

            for key in self.color_dict:
                match_confidence = self.color_distance(detected_color, self.color_dict[key])
                if match_confidence < 0.05:
                    color_string = key
                    break
                else:
                    color_string = "No Match"

            '''

            SmartDashboard.putString('Detected Color', color_string)
            SmartDashboard.putNumber("Red", detected_color.red)
            SmartDashboard.putNumber("Green", detected_color.green)
            SmartDashboard.putNumber("Blue", detected_color.blue)
            SmartDashboard.putNumber("Confidence", match_confidence)

    def color_distance(self, color_1, color_2):
        return math.sqrt(
            (color_1.red - color_2.red)**2 + \
               (color_1.green - color_2.green)**2 + \
               (color_1.blue - color_2.blue)**2
        )
