from wpilib.command import Command
from wpilib import Timer
import math

class AutonomousRotate(Command):
    """
    This command drives the robot over a given distance with simple proportional
    control - handled internally by the sparkMAX
    """
    def __init__(self, robot, setpoint=10):
        """The constructor"""
        super().__init__()
        # Signal that we require ExampleSubsystem
        self.requires(robot.drivetrain)
        self.setpoint = setpoint
        self.robot = robot
        self.tolerance = 1
        self.kp = 0.01
        self.kd = 0.1
        self.kf = 0.3
        self.start_yaw =0
        self.error = 0
        self.power = 0
        self.prev_error = 0

    def initialize(self):
        """Called just before this Command runs the first time."""
        print("\n" + f"Started {self.__class__} with setpoint {self.setpoint} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s")
        self.setTimeout(5)
        self.start_yaw = self.robot.navigation.get_yaw()
        self.error = 0
        self.prev_error = 0

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        self.error = self.setpoint - (self.robot.navigation.get_yaw()-self.start_yaw)
        self.power = self.kp * self.error + self.kf * math.copysign(1, self.error) + self.kd * (self.error - self.prev_error) / 0.02
        self.prev_error = self.error
        self.robot.drivetrain.tank_drive(-self.power, self.power)

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        # somehow need to wait for the error level to get to a tolerance... request from drivetrain?
        return abs(self.setpoint - self.robot.navigation.get_yaw()) <= self.tolerance or self.isTimedOut()


    def end(self):
        """Called once after isFinished returns true"""
        print("\n" + f"Ended {self.__class__} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s")
        self.robot.drivetrain.stop()

    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.end()
        print("\n" + f"Interrupted {self.__class__} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s")
