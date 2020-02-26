from wpilib.command import Command
from wpilib import SmartDashboard, Timer

class Spin3x(Command):
    def __init__(self, robot, power=0.2, thrust=-0.06, timeout=5):
        # sometimes super()__init__ gives an error when Command._init__ does not...
        Command.__init__(self, name='Spin3x')
        self.requires(robot.peripherals)
        self.requires(robot.drivetrain)  # lock out the joysticks

        self.robot = robot
        self.power = power  # power for the spinner motor
        self.thrust = thrust  # push the robot just a little bit to maintain contact with the spinner
        self.timeout = timeout  # do not let this be a forever while loop

        self.old_color = "No Match"
        self.current_color = "No Match"
        self.start_time = 0
        self.color_transition_counter = 0

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        self.color_transition_counter = 0
        self.setTimeout(self.timeout)
        self.robot.peripherals.panel_clockwise(self.power)
        self.old_color = "No Match"
        self.telemetry = {'time': [], 'color': []}
        # self.current_color = "No Match"
        self.current_color = self.robot.peripherals.get_color_str()
        print("\n" + f"** Started {self.getName()} with current color {self.current_color} and power {self.power} at {self.start_time} s **", flush=True)

    def execute(self):
        self.current_color = self.robot.peripherals.get_color_str()
        self.robot.drivetrain.spark_with_stick(thrust=self.thrust)

        if self.current_color == "No Match" and self.old_color != "No Match":
            self.timeout += 1.0  # increases time left when color changes
            self.setTimeout(self.timeout)
            self.color_transition_counter += 1

        self.old_color = self.current_color
        self.telemetry['time'].append(self.timeSinceInitialized())
        self.telemetry['color'].append(self.current_color)

    def isFinished(self):
        return self.color_transition_counter >= 25 or self.isTimedOut()  # correct color or timed out

    def end(self):
        self.robot.peripherals.panel_clockwise(0)
        self.robot.drivetrain.stop()
        for key in self.telemetry:
            if key == 'time':
                SmartDashboard.putNumberArray("color_telemetry_" + str(key), self.telemetry[key])
            else:
                SmartDashboard.putStringArray("color_telemetry_" + str(key), self.telemetry[key])

        print("\n" + f"** Ended {self.getName()} with current color {self.current_color} and {self.color_transition_counter} color transitions at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")

    def interrupted(self):
        self.end()
