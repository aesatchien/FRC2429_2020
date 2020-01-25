from wpilib.command import Command
from wpilib import Timer

class PanelSpinner(Command):
    """
    This command opens and closed the piston
    """

    def __init__(self, robot, button=None, power=0):
        super().__init__()
        self.robot = robot
        self.button = button
        self.power = power
        strip_name = lambda x: str(x)[1 + str(x).rfind('.'):-2]
        self.name = strip_name(self.__class__)

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.name} at {self.start_time} s **", flush=True)

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        if self.button == self.robot.oi.co_buttonA:
            self.robot.peripherals.panel_clockwise(power=0.4)
        else:
            print("Something happened that I didn't understand in Panel Spinner")
        pass

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return not self.button.get()

    def end(self):
        """Called once after isFinished returns true"""
        #print("\n" + f"** Ended {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
        self.robot.peripherals.panel_clockwise(power=0.0)
    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        #print("\n" + f"** Interrupted {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
