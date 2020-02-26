# Attempt to convert 2019 Spartan Java to Python - 11/22/2019 CJH
import wpilib
from wpilib.command import Subsystem
from wpilib import Color, I2C, PowerDistributionPanel, SmartDashboard, Spark
from rev.color import ColorSensorV3
import math
from networktables import NetworkTables
from wpilib import DriverStation

class Peripherals(Subsystem):
    def __init__(self, robot):
        Subsystem.__init__(self, "peripherals")
        self.robot = robot
        self.control_panel_spark = Spark(8)
        self.counter = 0
        self.color_sensor = ColorSensorV3(I2C.Port.kOnboard)
        self.match_confidence = 0
        self.ball_table = NetworkTables.getTable("BallCam")
        self.lidar = Lidar()
        self.PDB = PowerDistributionPanel()

        # we can config the colorsensor resolution and the rate
        #self.color_sensor.setGain(ColorSensorV3.GainFactor.k1x)
        self.color_sensor.configureColorSensor(res=ColorSensorV3.ColorResolution.kColorSensorResolution16bit,
                                               rate=ColorSensorV3.ColorMeasurementRate.kColorRate25ms)

        # made these numbers ourselves by positioning the sensor over the target and recording the RGB
        self.kBlueTarget = Color(0.181, 0.453, 0.365)
        self.kGreenTarget = Color(0.215, 0.529, 0.255)
        self.kRedTarget = Color(0.417, 0.398, 0.184)
        self.kYellowTarget = Color(0.326, 0.519, 0.154)
        self.color_dict = {"blue":self.kBlueTarget, "green":self.kGreenTarget, "red":self.kRedTarget, "yellow":self.kYellowTarget}

    def run_spinner(self, power=0):
        self.control_panel_spark.set(power)

    def panel_clockwise(self, power):
        self.control_panel_spark.set(power)

    def get_color_str(self, color=None):
        #detected_color = color or self.color_sensor.getColor()
        if color is None:
            detected_color = self.color_sensor.getColor()
        else:
            detected_color = color

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

    def get_fms_color(self):
        """Gets the target panel color from the gameSpecificMessage and converts it to the 90 degree pair
        :return color string for parsing in the spin_to_color command"""
        fms_color = DriverStation.getGameSpecificMessage()
        # Sensor is 90 degrees to us, so Y<->G and B<->R
        if fms_color == 'Y':
            target_color = 'green'
        elif fms_color == 'G':
            target_color = 'yellow'
        elif fms_color == 'R':
            target_color = 'blue'
        elif fms_color == 'B':
            target_color = 'red'
        else:
            print(f"Unexpected result: getGameSpecifcMessage returned {fms_color}")
            return 'red'  #  cop out
        print(f"Converting fms message {fms_color} to color {target_color}")
        return target_color

    def lidar_distance(self):
        return self.lidar.dist()  # distance in cm

    def log(self):
        self.lidar_meas = self.lidar_distance()
        self.counter += 1
        if self.counter % 10 == 0:
            detected_color = self.color_sensor.getColor()
            color_string = self.get_color_str(detected_color)

            SmartDashboard.putString('Detected Color', color_string)
            SmartDashboard.putNumber("Red", round(detected_color.red, 3))
            SmartDashboard.putNumber("Green", round(detected_color.green, 3))
            SmartDashboard.putNumber("Blue", round(detected_color.blue, 3))
            SmartDashboard.putNumber("Confidence", round(self.match_confidence, 3))
            SmartDashboard.putNumber("Lidar Distance", self.lidar_meas)
            #SmartDashboard.putNumber("Cam distance", self.ball_table.getNumber("distance", 0))
            #SmartDashboard.putString('PDB Status', str(self.PDB.getTotalCurrent()))

            currents = ""
            for i in range(16):
                currents = currents + " " + str(round(self.PDB.getCurrent(i), 1))
            currents = currents + " = " + str(int(self.PDB.getTotalCurrent()))
            SmartDashboard.putString("PDB Status", currents)
            #self.PDB.clearStickyFaults()
            joystick_string = f"Y: {-self.robot.oi.stick.getRawAxis(1):+3.2f}  X: {self.robot.oi.stick.getRawAxis(0):+3.2f}  Tw: {self.robot.oi.stick.getRawAxis(4):+3.2f}"
            SmartDashboard.putString("Joystick", joystick_string)


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