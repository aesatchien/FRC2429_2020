from wpilib.command import Command
from wpilib import Timer

class RaiseClimber(Command):
    """
    This command opens and closed the piston
    """

    def __init__(self, robot, direction=None, power=0.2, button=None):
        Command.__init__(self, name='raise_climber')
        self.robot = robot
        self.direction = direction
        self.power = power
        self.button = button

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} with direction {self.direction} and power {self.power} at {self.start_time} s **", flush=True)

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        if self.direction == 'hookup':
            self.robot.climber.raise_hook(self.power)
        if self.direction == 'hookdown':
            self.robot.climber.raise_hook(self.power)
        elif self.direction == 'climb':
            self.robot.climber.raise_robot(self.power)
        elif self.direction == 'left':
            self.robot.climber.roll_robot(self.power)
        elif self.direction == 'right':
            self.robot.climber.roll_robot(self.power)
        else:
            print("!!! Invalid climbing option passed to raise climber !!!")

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return not self.button.get()

    def end(self):
        """Called once after isFinished returns true"""
        print("\n" + f"** Ended {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
        if self.direction == 'hookup':
            self.robot.climber.raise_hook(0.2)
        else:
            self.robot.climber.stop_hook()
        self.robot.climber.stop_winch()
        self.robot.climber.robot_roller_stop()
    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        #print("\n" + f"** Interrupted {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
        self.end()