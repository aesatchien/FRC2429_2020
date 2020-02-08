from wpilib.command import Command
from wpilib import Timer


class PneumaticPiston(Command):
    """
    This command opens and cloed the piston
    """

    def __init__(self, robot, direction=None):
        #super().__init__()
        Command.__init__(self, name='PneumaticPiston')
        self.requires(robot.pneumatics)
        self.robot = robot
        self.direction = direction

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} with input {self.direction} at {self.start_time} s **", flush=True)
        self.robot.pneumatics.actuate_solenoid(self.direction)

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        pass

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return True

    def end(self):
        """Called once after isFinished returns true"""
        #print("\n" + f"** Ended {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        #print("\n" + f"** Interrupted {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
