from wpilib.command import Command
from wpilib import Timer
import math
from wpilib import SmartDashboard

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
        strip_name = lambda x: str(x)[1 + str(x).rfind('.'):-2]
        self.name = strip_name(self.__class__)

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.name} with setpoint {self.setpoint} at {self.start_time} s **")
        SmartDashboard.putString("alert", f"** Started {self.name} with setpoint {self.setpoint} at {self.start_time} s **")
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
        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Ended {self.name} at {end_time} s with a duration of {round(end_time-self.start_time,1)} s **")
        SmartDashboard.putString("alert", f"** Ended {self.name} at {end_time} s with a duration of {round(end_time - self.start_time, 1)} s **")
        self.robot.drivetrain.stop()

    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.robot.drivetrain.stop()
        print("\n" + f"** Interrupted {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
