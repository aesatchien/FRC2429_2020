from wpilib.command import Command
from wpilib import Timer

class SpinToColor(Command):
    def __init__(self, robot, color_name):
        super().__init__(self, name='intake')
        self.requires(robot.peripherals)

        self.current_color = None
        self.previous_color = None

        self.robot = robot
        self.color_name = color_name

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.name} with power {self.power} at {self.start_time} s **", flush=True)

        self.previous_color = self.current_color = self.robot.peripherals.get_color_str();
