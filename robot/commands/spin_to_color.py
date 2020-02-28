from wpilib.command import Command
from wpilib import SmartDashboard, Timer

class SpinToColor(Command):
    def __init__(self, robot, target_color=None, source='dash', power=0.25, thrust=-0.10, timeout=5):
        # sometimes super()__init__ gives an error when Command._init__ does not...
        Command.__init__(self, name='SpinToColor')
        self.requires(robot.peripherals)
        self.requires(robot.drivetrain)  # lock out the joysticks

        self.robot = robot
        self.target_color = target_color
        self.source = source  # have to either get this from the dashboard (testing) or FMS (competition)
        self.power = power  # power for the spinner motor
        self.thrust = thrust  # push the robot just a little bit to maintain contact with the spinner
        self.timeout = timeout  # do not let this be a forever while loop

        self.old_color = "No Match"
        self.current_color = "No Match"
        self.start_time = 0

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        self.telemetry = {'time': [], 'color': []}
        if self.source == 'dash':
            self.target_color = self.robot.oi.color_chooser.getSelected()
        elif self.source == 'fms':
            self.target_color = self.robot.peripherals.get_fms_color()

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

        self.telemetry['time'].append(self.timeSinceInitialized())
        self.telemetry['color'].append(self.current_color)

    def isFinished(self):
        return self.current_color == self.target_color or self.isTimedOut()  # correct color or timed out

    def end(self):
        self.robot.peripherals.panel_clockwise(0)
        self.robot.drivetrain.stop()
        for key in self.telemetry:
            if key == 'time':
                SmartDashboard.putNumberArray("color_telemetry_" + str(key), self.telemetry[key])
            else:
                #print(f"color_telemetty is: {self.telemetry['color']} \n and with length {len(self.telemetry['color'])}")
                SmartDashboard.putStringArray("color_telemetry_" + str(key), self.telemetry[key])
        print("\n" + f"** Ended {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")

    def interrupted(self):
        self.end()
