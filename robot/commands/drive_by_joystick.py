from wpilib.command import Command
from wpilib import Timer

class DriveByJoystick(Command):
    """
    This allows Logitech gamepad to drive the robot. It is always running
    except when interrupted by another command.
    """

    def __init__(self, robot):
        super().__init__()
        self.requires(robot.drivetrain)
        self.robot = robot
        self.twist_sensitivity = 0.5
        strip_name = lambda x: str(x)[1 + str(x).rfind('.'):-2]
        self.name = strip_name(self.__class__)

    def initialize(self):
        """Called just before this Command runs the first time."""
        pass

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        self.robot.drivetrain.smooth_drive(self.robot.oi.stick.getRawAxis(1), -self.twist_sensitivity*self.robot.oi.stick.getRawAxis(4))

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return False

    def end(self):
        """Called once after isFinished returns true"""
        self.robot.drivetrain.stop()
        print("\n" + f"** Ended {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.robot.drivetrain.stop()
        print("\n" + f"** Interrupted {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
