from wpilib.command import Command
from wpilib import Timer

class AutonomousWait(Command):
    """
    This command opens and closed the piston
    """

    def __init__(self, robot, timeout=5):
        Command.__init__(self, name='Autonomous Wait')
        self.robot = robot
        self.timeout = timeout
        self.setTimeout(timeout)

    def initialize(self):
        """Called just before this Command runs the first time."""

        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} with input {self.timeout} at {self.start_time} s **", flush=True)

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        self.robot.drivetrain.drive.feed()

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return self.isTimedOut()

    def end(self):
        """Called once after isFinished returns true"""
        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Ended {self.getName()} at {end_time} s with a duration of {round(end_time-self.start_time,1)} s **")

    def interrupted(self):
        self.end()

