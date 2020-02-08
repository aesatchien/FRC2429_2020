from wpilib.command import Command
from wpilib import Timer

class PanelSpinner(Command):
    """
    This command opens and closed the piston
    """

    def __init__(self, robot, button=None, power=0):
        #super().__init__()
        Command.__init__(self, name='PanelSpinner')
        self.robot = robot
        self.button = button
        self.power = power

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        self.robot.peripherals.panel_clockwise(self.power)

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return not self.button.get()

    def end(self):
        """Called once after isFinished returns true"""
        print("\n" + f"** Ended {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
        self.robot.peripherals.panel_clockwise(power=0.0)
    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        #print("\n" + f"** Interrupted {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
        self.end()
