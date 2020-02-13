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
from networktables import NetworkTables

class Peripherals(Subsystem):
    def __init__(self, robot):
        Subsystem.__init__(self, "peripherals")
        self.intake_spark = Spark(6)
        self.control_panel_spark = Spark(5)
        self.left_dispenser_gate = Servo(7)
        self.right_dispenser_gate = Servo(8)
        self.counter = 0
        self.color_sensor = ColorSensorV3(I2C.Port.kOnboard)
        self.match_confidence = 0
        self.ball_table = NetworkTables.getTable("BallCam")
        self.lidar = Lidar()
        self.lidar_meas = None


        # we can config the colorsensor resolution and the rate
        #self.color_sensor.setGain(ColorSensorV3.GainFactor.k1x)
        #self.color_sensor.configureColorSensor(res=, rate=)

        # made these numbers ourselves by positioning the sensor over the target and recording the RGB
        self.kBlueTarget = Color(0.181, 0.453, 0.365)
        self.kGreenTarget = Color(0.215, 0.529, 0.255)
        self.kRedTarget = Color(0.417, 0.398, 0.184)
        self.kYellowTarget = Color(0.326, 0.519, 0.154)
        self.color_dict = {"blue":self.kBlueTarget, "green":self.kGreenTarget, "red":self.kRedTarget, "yellow":self.kYellowTarget}

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

    def get_color_str(self, color=None):
        detected_color = color or self.color_sensor.getColor()

        '''
        if color: 
            detected_color = color
        else: 
            detected_color = self.color_sensor.getColor() 
        '''
        self.match_confidence = 0.5
        for key in self.color_dict:
            self.match_confidence = self.color_distance(detected_color, self.color_dict[key])
            if self.match_confidence < 0.05:
                color_string = key
                break
            else:
                color_string = "No Match"

        return color_string

    def color_distance(self, color_1, color_2):
        return math.sqrt(
            (color_1.red - color_2.red) ** 2 + (color_1.green - color_2.green) ** 2 +(color_1.blue - color_2.blue) ** 2)

    def lidar_distance(self):
        return self.lidar.dist() # distance in cm

    def log(self):
        self.counter += 1
        if self.counter % 5 == 0:
            detected_color = self.color_sensor.getColor()
            color_string = self.get_color_str(detected_color)
            self.lidar_meas = self.lidar_distance()

            SmartDashboard.putString('Detected Color', color_string)
            SmartDashboard.putNumber("Red", detected_color.red)
            SmartDashboard.putNumber("Green", detected_color.green)
            SmartDashboard.putNumber("Blue", detected_color.blue)
            SmartDashboard.putNumber("Confidence", self.match_confidence)
            SmartDashboard.putNumber("Lidar Distance", self.lidar_meas)
            #SmartDashboard.putNumber("Cam distance", self.ball_table.getNumber("distance", 0))


class Lidar:
    def __init__(self):
        self.i2c = wpilib.I2C(wpilib.I2C.Port.kOnboard, 0x62)
        self.i2c.write(0x11, 0xff) # OUTER_LOOP_COUNT; enable free running mode
        self.i2c.write(0x04, 0x28) # ACQ_CONFIG_REG; use MEASURE_DELAY
        self.i2c.write(0x00, 0x04) # ACQ_COMMAND; take distance with receiver bias corr
        self.buf = bytearray(2) # for reading distance 2 bytes

    def dist(self):
        self.i2c.writeBulk(bytearray([0x8f])) # don't use repeated start; write address only, then read
        self.i2c.readOnly(self.buf) # MSB, LSB of distance measurement in cm
        return self.buf[0]*2**8 + self.buf[1]