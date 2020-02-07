from wpilib.command import Command
from wpilib import Timer

class SpinToColor(Command):
    def __init__(self, robot, color_name, power=0.2, thrust=0.1, timeout=10):
        # sometimes super()__init__ gives an error when Command._init__ does not...
        Command.__init__(self, name='spintocolor')
        self.requires(robot.peripherals)

        self.robot = robot
        self.color_name = color_name
        self.power = power
        self.thrust = thrust
        self.timeout = timeout

        self.old_color = "No match"
        self.current_color = "No match"

        strip_name = lambda x: str(x)[1 + str(x).rfind('.'):-2]
        self.name = strip_name(self.__class__)

        self.start_time = 0

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.name} with power {self.power} at {self.start_time} s **", flush=True)

        self.setTimeout(self.timeout)
        self.robot.drivetrain.spark_with_stick(thrust=self.thrust)
        self.robot.peripherals.panel_clockwise(self.power)

    def execute(self):
        self.current_color = self.robot.peripherals.get_color_str()

        if self.current_color != self.old_color:
            self.timeout += 1  # increases time left when color changes
            self.setTimeout(self.timeout)
            self.old_color = self.current_color

    def isFinished(self):
        return self.current_color == self.color_name or self.isTimedOut()  # correct color or timed out

    def end(self):
        self.robot.peripherals.panel_clockwise(0)
        self.robot.drivetrain.stop()

        print("\n" + f"** Ended {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")

    def interrupted(self):
        self.end()
