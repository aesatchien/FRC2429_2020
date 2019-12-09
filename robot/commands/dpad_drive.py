from wpilib.command import Command
from wpilib import Timer, SmartDashboard

class DpadDrive(Command):
    """
    This allows Logitech gamepad's dpad to drive the robot. It overrides the stick.
    Change the drive_power and twist_power variables to change how it reacts
    """

    def __init__(self, robot, state, button):
        super().__init__()
        self.requires(robot.drivetrain)
        self.robot = robot
        self.state = state
        self.button = button
        self.drive_power = 0.15
        self.twist_power = 0.1
        self.direction = 1 # change this to -1 change all directions quickly
        strip_name = lambda x: str(x)[1 + str(x).rfind('.'):-2]
        self.name = strip_name(self.__class__)

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time,1)
        print("\n" + f"** Started {self.name} with input {self.state} at {self.start_time} s **")
        SmartDashboard.putString("alert", f"** Started {self.name} with input {self.state} at {self.start_time} s **")
    def execute(self):
        """
        Called repeatedly when this Command is scheduled to run
        Should not have to change this if it works - just change variables above
        """
        if self.state.lower() == "up":
            self.robot.drivetrain.spark_with_stick(-self.drive_power*self.direction, 0)
        if self.state.lower() == "down":
            self.robot.drivetrain.spark_with_stick(self.drive_power*self.direction, 0)
        if self.state.lower() == "right":
            self.robot.drivetrain.spark_with_stick(0, -self.twist_power*self.direction)
        if self.state.lower() == "left":
            self.robot.drivetrain.spark_with_stick(0, self.twist_power*self.direction)

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return not self.button.get()

    def end(self):
        """Called once after isFinished returns true"""
        self.robot.drivetrain.stop()
        print("\n" + f"** Ended {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.robot.drivetrain.stop()
        print("\n" + f"** Interrupted {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
