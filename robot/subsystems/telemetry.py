# Attempt to convert 2019 Spartan Java to Python - 11/22/2019 CJH

from datetime import datetime
from wpilib.command import Subsystem
from wpilib import SmartDashboard, Timer
from wpilib.kinematics import DifferentialDriveOdometry
import wpilib.geometry as geo

class Telemetry(Subsystem):
    def __init__(self, robot):
        super().__init__("telemetry")
        #Subsystem.__init__("navigation")
        self.counter = 0
        self.robot = robot

        # this might belong in drivetrain, but let's see how it goes - 20210119 CJH
        self.odometry = DifferentialDriveOdometry(gyroAngle=geo.Rotation2d(0))  # this should generate with angle 0 and 0 pose
        self.x = 0
        self.y = 0
        self.rotation = 0
        self.pose = None

        odometry_string = f"X: {self.x:+3.2f}  Y: {self.x:+3.2f}  Rot: {self.rotation:+3.2f}"
        self.odometry_list = 256*[odometry_string]
        SmartDashboard.putString("Odometry", odometry_string)

    def write_telemetry(self):
        file_name = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f'Writing to file with name {file_name}')
        # ToDo: create a list (dictionary?) of values and write to the rio for SFTP retrieval / NetworkTables later

    def log(self):
        """ Intended to be used in teleop to log the pose """
        self.counter += 1
        if self.counter % 10 == 0:
            if self.robot.isReal():
                gyro = geo.Rotation2d.fromDegrees(self.robot.navigation.navx.getAngle())
                self.pose = self.odometry.update(gyro, self.robot.drivetrain.sparkneo_encoder_1.getPosition(),
                                                 self.robot.drivetrain.sparkneo_encoder_3.getPosition())
                # need to think of something that works well in network tables - maybe a formatted string
                self.odometry_list.append()
            else:
                # hmm.  this is in the physics module.  will have to update from there.  can't write a file from there anyway.
                # ToDo: update the network table entries in the physics.py update_sim()
                pass

            self.x = self.pose.translation().x
            self.y = self.pose.translation().y
            self.rotation = self.pose.rotation().degrees()

            odometry_string = f"X: {self.x:+3.2f}  Y: {self.y:+3.2f}  Rot: {self.rotation:+3.2f}"
            self.odometry_list.pop(0)
            self.odometry_list.append(odometry_string)
            SmartDashboard.putString("Odometry", odometry_string)