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
        self.left_dispenser_gate.setAngle(0)
        self.right_dispenser_gate.setAngle(0)

    def open_gate(self):
        self.left_dispenser_gate.setAngle(145)
        self.right_dispenser_gate.setAngle(145)

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
        self.lidar_meas = self.lidar_distance()
        self.counter += 1
        if self.counter % 5 == 0:
            detected_color = self.color_sensor.getColor()
            color_string = self.get_color_str(detected_color)

            SmartDashboard.putString('Detected Color', color_string)
            SmartDashboard.putNumber("Red", round(detected_color.red, 3))
            SmartDashboard.putNumber("Green", round(detected_color.green, 3))
            SmartDashboard.putNumber("Blue", round(detected_color.blue, 3))
            SmartDashboard.putNumber("Confidence", round(self.match_confidence, 3))
            SmartDashboard.putNumber("Lidar Distance", self.lidar_meas)
            #SmartDashboard.putNumber("Cam distance", self.ball_table.getNumber("distance", 0))


class Lidar:
    addr = 0x62 # I2C bus address
    OUTER_LOOP_COUNT = 0x11
    ACQ_CONFIG_REG = 0x04
    ACQ_COMMAND = 0x00
    FULL_DELAYword = 0x8f

    def __init__(self):
        self.i2c = wpilib.I2C(wpilib.I2C.Port.kOnboard, Lidar.addr)
        err = self.i2c.write(Lidar.OUTER_LOOP_COUNT, 0xff) # enable free running mode
        err += self.i2c.write(Lidar.ACQ_CONFIG_REG, 0x28) # use MEASURE_DELAY
        err += self.i2c.write(Lidar.ACQ_COMMAND, 0x04) # take distance with receiver bias corr
        self.buf = bytearray(2) # for reading distance 2 bytes
        if err:
            outstr = "Lidar_Lite_V3 write error or not found"
            wpilib.DriverStation.reportError(outstr, False)
            print(outstr)

    def dist(self):
        err = self.i2c.writeBulk(bytearray([Lidar.FULL_DELAYword]))  # don't use repeated start; write address only, then read
        err += self.i2c.readOnly(self.buf) # MSB, LSB of distance measurement in cm
        return self.buf[0]*2**8 + self.buf[1] if not err else -1