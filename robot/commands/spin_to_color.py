from wpilib.command import Command
from wpilib import Timer
from wpilib import SmartDashboard

class SpinToColor(Command):
    def __init__(self, robot, target_color=None, source='dash', power=0.2, thrust=-0.06, timeout=5):
        # sometimes super()__init__ gives an error when Command._init__ does not...
        Command.__init__(self, name='SpinToColor')
        self.requires(robot.peripherals)
        self.requires(robot.drivetrain)

        self.robot = robot
        self.target_color = target_color
        self.source = source
        self.power = power
        self.thrust = thrust
        self.timeout = timeout

        self.old_color = "No match"
        self.current_color = "No match"

        self.start_time = 0

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        if self.source == 'dash':
            self.target_color = self.robot.oi.color_chooser.getSelected()
        else:
            pass
            # TODO: get games specific message color value here
        self.setTimeout(self.timeout)
        self.robot.peripherals.panel_clockwise(self.power)
        print("\n" + f"** Started {self.getName()} with target color {self.target_color} and power {self.power} at {self.start_time} s **", flush=True)

    def execute(self):
        self.current_color = self.robot.peripherals.get_color_str()
        self.robot.drivetrain.spark_with_stick(thrust=self.thrust)

        if self.current_color != self.old_color:
            self.timeout += 1.0  # increases time left when color changes
            self.setTimeout(self.timeout)
            self.old_color = self.current_color

    def isFinished(self):
        return self.current_color == self.target_color or self.isTimedOut()  # correct color or timed out

    def end(self):
        self.robot.peripherals.panel_clockwise(0)
        self.robot.drivetrain.stop()

        print("\n" + f"** Ended {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")

    def interrupted(self):
        self.end()
