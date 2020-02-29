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

        self.start_time = 0

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        self.telemetry = {'time': [], 'color': [], 'colorraw':[]}
        if self.source == 'dash':
            self.target_color = self.robot.oi.color_chooser.getSelected()
        elif self.source == 'fms':
            self.target_color = self.robot.peripherals.get_fms_color()

        self.setTimeout(self.timeout)
        self.robot.peripherals.panel_clockwise(self.power)
        self.color_next = self.robot.peripherals.color_nextcw # hardcoded for cw
        self.old_color = None
        self.current_color = "No match"
        print("\n" + f"** Started {self.getName()} with target color {self.target_color} and power {self.power} at {self.start_time} s **", flush=True)

    def execute(self):
        colorraw = self.robot.peripherals.get_color_raw()
        colornorm = self.robot.peripherals.color_norm(colorraw)
        self.current_color = self.robot.peripherals.get_color_str(colornorm)
        self.robot.drivetrain.spark_with_stick(thrust=self.thrust)

        # old color and sequence are used only to reset timeout
        nextcolor = self.color_next.get(self.old_color, '') # old_color is none after initialization, don't recognize a "next" yet
        if self.current_color == nextcolor:
            self.timeout = self.timeSinceInitialized() + 1  # increases time left when color changes
            self.setTimeout(self.timeout)
            self.old_color = self.current_color

        if (self.old_color is None) and (self.current_color in self.robot.peripherals.color_dict.keys()):
            self.old_color = self.current_color # first recognized color clears initial None in old_color

        self.telemetry['time'].append(self.timeSinceInitialized())
        self.telemetry['color'].append(self.current_color)
        self.telemetry['colorraw'].append(f"{colorraw.red:5d},{colorraw.green:5d},{colorraw.blue:5d}")

    def isFinished(self):
        return self.current_color == self.target_color or self.isTimedOut()  # correct color or timed out

    def end(self):
        self.robot.peripherals.panel_clockwise(0)
        self.robot.drivetrain.stop()
        for key,value in self.telemetry.items():
            if key == 'time':
                for iarray in range(math.ceil(len(value) / 256.)):
                    SmartDashboard.putNumberArray(f"color_telemetry_{key}_{iarray}", value[256*iarray:min(256*(iarray+1), len(value))])
                open(f'/home/lvuser/{key}.txt','ab').write('\n'.join([f"{t}" for t in value+['']*2]).encode())
            else:
                for iarray in range(math.ceil(len(value) / 256.)):
                    SmartDashboard.putStringArray("color_telemetry_{key}_{iarray}", value[256*iarray:min(256*(iarray+1), len(value))])
                open(f'/home/lvuser/{key}.txt','ab').write('\n'.join(value+['']*2).encode()) # extra empty ensures \n at end
        print("\n" + f"** Ended {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")

    def interrupted(self):
        self.end()
