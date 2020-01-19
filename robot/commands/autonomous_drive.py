from wpilib.command import Command
from wpilib import Timer
from wpilib import SmartDashboard
from commands.track_telemetry import TrackTelemetry

class AutonomousDrive(Command):
    """
    11/26/2019 CJH
    This command drives the robot over a given distance with simple proportional
    control - handled internally by the sparkMAX
    This should be one of the three canonical autonomous functions:
    1) drive a distance
    2) rotate an angle
    3) actuate some mechanism
    """
    # may need to use variables at some point ...
    tolerance = 0.5

    def __init__(self, robot, setpoint=None, control_type='position', button = 'None', timeout=None, from_dashboard = True):
        """The constructor"""
        super().__init__()
        # Signal that we require ExampleSubsystem
        self.requires(robot.drivetrain)
        # little trick here so we can call this either from code explicitly with a setpoint or get from smartdashboard
        self.from_dashboard = from_dashboard
        self.setpoint = setpoint
        if timeout is None:
            self.setTimeout(5)
            self.initial_timeout = 5
        else:
            self.setTimeout(timeout)
            self.initial_timeout = timeout
        self.robot = robot
        self.control_type = control_type
        self.button = button
        strip_name = lambda x: str(x)[1 + str(x).rfind('.'):-2]
        self.name = strip_name(self.__class__)

    def initialize(self):
        """Called just before this Command runs the first time."""
        if self.from_dashboard:
            self.setpoint = SmartDashboard.getNumber('Auto Distance',0)
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.name} with setpoint {self.setpoint} and control_type {self.control_type} at {self.start_time} s **")
        SmartDashboard.putString("alert", f"** Started {self.name} with setpoint {self.setpoint} and control_type {self.control_type} at {self.start_time} s **")
        if self.control_type == 'position':
            self.robot.drivetrain.goToSetPoint(self.setpoint)
        elif self.control_type == 'velocity':
            self.robot.drivetrain.set_velocity(self.setpoint)
        else:
            pass
        self.has_arrived = False
        self.telemetry = {'time':[], 'position':[], 'velocity':[], 'current':[], 'output':[]}
        self.counter = 0
        self.extra_time = 0.5 # amount of time to give it to slow down after we reach setpoint
        self.setTimeout(self.initial_timeout) # needs to be in initialize because we modify it in execute - dashboard instance keeps getting reused

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        self.robot.drivetrain.drive.feed()

        if self.counter % 2 == 0:
            #self.telemetry['time'].append(Timer.getFPGATimestamp() - self.robot.enabled_time)
            self.telemetry['time'].append(self.timeSinceInitialized())
            self.telemetry['position'].append(self.robot.drivetrain.get_position())
            self.telemetry['velocity'].append(self.robot.drivetrain.sparkneo_encoder_1.getVelocity())
            self.telemetry['current'].append(self.robot.drivetrain.spark_neo_left_front.getOutputCurrent())
            self.telemetry['output'].append(self.robot.drivetrain.spark_neo_left_front.getAppliedOutput())

        self.counter += 1

        # try to trick ourselves into giving ~ a half second to slow down at the end of the motion profile
        if self.control_type == 'position' and not self.has_arrived:
            if self.setpoint > 0:
                if self.setpoint - self.robot.drivetrain.get_position() <= self.tolerance:
                    self.has_arrived = True
                    self.timeout = self.timeSinceInitialized() + self.extra_time
                    print(f"** We have arrived at setpoint! (at {round(self.timeSinceInitialized(), 1)})")
            else:
                if self.setpoint - self.robot.drivetrain.get_position() >= self.tolerance:
                    self.has_arrived = True
                    self.timeout = self.timeSinceInitialized() + self.extra_time
                    print(f"** We have arrived at setpoint! (at {round(self.timeSinceInitialized(), 1)})")

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        # somehow need to wait for the error level to get to a tolerance... request from drivetrain?

        if self.control_type == 'velocity':
            return not self.button.get()
        elif self.isTimedOut():
            return True
        else:
            return False

    def end(self):
        """Called once after isFinished returns true"""
        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Ended {self.name} at {end_time} s with a duration of {round(end_time-self.start_time,1)} s **")
        SmartDashboard.putString("alert", f"** Ended {self.name} at {end_time} s with a duration of {round(end_time-self.start_time,1)} s **")
        for key in self.telemetry:
            SmartDashboard.putNumberArray("telemetry_" + str(key), self.telemetry[key])
        self.robot.drivetrain.stop()

    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.end()
        print("\n" + f"** Interrupted {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
