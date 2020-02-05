from wpilib.command import Command
from wpilib import Timer

class SpinToColor(Command):
    def __init__(self, robot, color_name, power=0.2):
        # sometimes super()__init__ gives an error when Command._init__ does not...
        Command.__init__(self, name='spintocolor')
        self.requires(robot.peripherals)

        # what's this for? - CJH
        self.color = None

        self.robot = robot
        self.color_name = color_name
        self.power = power

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.name} with power {self.power} at {self.start_time} s **", flush=True)

        self.robot.peripherals.panel_clockwise(self.power)

    def isFinished(self):
        return self.robot.peripherals.get_color_str() == self.color_name

    def end(self):
        self.robot.peripherals.panel_clockwise(0)

        print("\n" + f"** Ended {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")

    def interrupted(self):
        self.end()
