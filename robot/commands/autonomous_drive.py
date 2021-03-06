from wpilib.command import Command
from wpilib import Timer
from wpilib import SmartDashboard
from networktables import NetworkTables
import math
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

    def __init__(self, robot, setpoint=None, control_type='position', button = 'None', timeout=None, source=None, manual_override=False):
        """The constructor"""
        #super().__init__()
        Command.__init__(self, name='AutonomousDrive')
        # Signal that we require ExampleSubsystem
        self.requires(robot.drivetrain)
        self.setpoint = setpoint
        self.source = source
        if timeout is None:
            self.setTimeout(5)
            self.initial_timeout = 5
        else:
            self.setTimeout(timeout)
            self.initial_timeout = timeout
        self.robot = robot
        self.control_type = control_type
        self.button = button
        self.max_thrust = 0.25
        self.encoders_have_reset = False
        self.manual_override= manual_override
        self.setpoint_sign = 1.0

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)

        self.has_arrived = False
        self.telemetry = {'time':[], 'position':[], 'velocity':[], 'current':[], 'output':[]}
        self.counter = 0
        self.extra_time = 0.5 # amount of time to give it to slow down after we reach setpoint
        self.setTimeout(self.timeSinceInitialized() +  self.initial_timeout) # needs to be in initialize because we modify it in execute - dashboard instance keeps getting reused


        if self.source is None:
            self.setpoint = self.setpoint
        elif self.source == "dashboard":
            self.setpoint = SmartDashboard.getNumber('Auto Distance', 0)
        elif self.source == "camera":
            ball_table = NetworkTables.getTable("BallCam")
            if ball_table.getNumber("targets", 0) > 0:
                self.setpoint = math.sqrt(ball_table.getNumber("distance", 0) ** 2 - 37.0**2)
            else:
                self.setpoint = 0  # this should end us
                self.has_arrived = True
            print(f"Autonomous drive found {ball_table.getNumber('targets', 0)} targets on BallCam and dist is {self.setpoint}...")
        print(
            "\n" + f"** Started {self.getName()} with setpoint {self.setpoint} and control_type {self.control_type} at {self.start_time} s **")
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} with setpoint {self.setpoint} and control_type {self.control_type} at {self.start_time} s **")

        if self.manual_override:
            self.robot.drivetrain.reset()
        else:
            if self.control_type == 'position':
                self.robot.drivetrain.goToSetPoint(self.setpoint, reset=False)
            elif self.control_type == 'velocity':
                self.robot.drivetrain.set_velocity(self.setpoint)
            else:
                print(f"Invalid control type sent to automous_drive: {self.control_type}")

        self.starting_position = self.robot.drivetrain.get_position()
        self.start_counter = 0

        #pos = self.robot.drivetrain.get_position()
        #error = (self.setpoint - pos)
        #print(f"Error is : {error:4.1f}  Position is : {pos:4.1f}")

        self.setpoint_sign = 1.0  # could do this all with a copysign but better to be explicit
        if self.setpoint < 0:
            self.setpoint_sign = -1.0


    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        # having problems at competition guarnateeing that the sparkmax will execute the motion profile
        if self.manual_override:
            self.robot.drivetrain.mecanum_velocity_cartesian(thrust=1.0*self.setpoint_sign, strafe=0, z_rotation=0)
        else:
            self.robot.drivetrain.drive.feed()

        if self.start_counter < 2:
            print(f"Waiting for {self.start_counter} iterations for encoder to clear (because encoder at {self.robot.drivetrain.get_position():4.1f})...")
            self.start_counter += 1
            return

        # track telemetry so we can grab it with jupyter notebook
        if self.counter % 2 == 0:
            #self.telemetry['time'].append(Timer.getFPGATimestamp() - self.robot.enabled_time)
            self.telemetry['time'].append(self.timeSinceInitialized())
            self.telemetry['position'].append(self.robot.drivetrain.get_position() - self.starting_position )
            self.telemetry['velocity'].append(self.robot.drivetrain.sparkneo_encoder_1.getVelocity())
            self.telemetry['current'].append(self.robot.drivetrain.spark_neo_left_front.getOutputCurrent())
            self.telemetry['output'].append(self.robot.drivetrain.spark_neo_left_front.getAppliedOutput())
        self.counter += 1



        # try to trick ourselves into giving ~ a half second to slow down at the end of the motion profile
        # TODO: get more opinions on how to do this better

        if self.control_type == 'position' and not self.has_arrived:
            #pos = self.robot.drivetrain.get_position() - self.starting_position
            pos = self.robot.drivetrain.get_position()
            error = self.setpoint_sign*(self.setpoint - pos)
            print(f"Error is : {error:4.1f}  Position is : {pos:4.1f}")
            if error <= self.tolerance:
                self.has_arrived = True
                self.setTimeout( self.timeSinceInitialized() + self.extra_time)
                print(f"** We have arrived at setpoint! (at {round(self.timeSinceInitialized(), 2)})")


    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        # somehow need to wait for the error level to get to a tolerance... request from drivetrain?

        if self.control_type == 'velocity':
            return not self.button.get()
        elif self.manual_override == True:
            return self.has_arrived
        elif self.isTimedOut():
            return True
        else:
            return False

    def end(self):
        """Called once after isFinished returns true"""
        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Ended {self.getName()} at {end_time} s with a duration of {round(end_time-self.start_time,1)} s **")
        SmartDashboard.putString("alert", f"** Ended {self.getName()} at {end_time} s with a duration of {round(end_time-self.start_time,1)} s **")
        for key in self.telemetry:
            SmartDashboard.putNumberArray("telemetry_" + str(key), self.telemetry[key])
        self.robot.drivetrain.stop()

    def interrupted(self):
        self.end()
        print("\n" + f"** Interrupted {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
