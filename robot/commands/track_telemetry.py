from wpilib.command import Command
from wpilib import Timer
from wpilib import SmartDashboard

class TrackTelemetry(Command):
    """
    12/29/2019 CJH
    Track the telemetry for the main controller and send it to
    """

    def __init__(self, robot, timeout=None):
        """The constructor"""
        #super().__init__()
        Command.__init__(self, name='TrackTelemetry')
        # Signal that we require ExampleSubsystem
        #self.requires(robot.drivetrain)
        # little trick here so we can call this either from code explicitly with a setpoint or get from smartdashboard
        if timeout is None:
            self.setTimeout(5)
        else:
            self.setTimeout(timeout)
        self.robot = robot
        #strip_name = lambda x: str(x)[1 + str(x).rfind('.'):-2]
        #self.name = strip_name(self.__class__)

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **")
        self.telemetry = {'time':[], 'position':[], 'velocity':[], 'current':[]}
        self.counter = 0

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        if self.counter % 2 == 0:
            #self.telemetry['time'].append(Timer.getFPGATimestamp() - self.robot.enabled_time)
            self.telemetry['time'].append(self.timeSinceInitialized())
            self.telemetry['position'].append(self.robot.drivetrain.get_position())
            self.telemetry['velocity'].append(self.robot.drivetrain.sparkneo_encoder_1.getVelocity())
            #self.telemetry['current'].append(self.robot.drivetrain.spark_neo_l1.getAppliedOutput())
        self.counter += 1

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        # somehow need to wait for the error level to get to a tolerance... request from drivetrain?
        # I think I will change the timeout to be 0.5s after the tolerance is hit so we can graph better
        return self.isTimedOut()

    def end(self):
        """Called once after isFinished returns true"""
        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Ended {self.getName()} at {end_time} s with a duration of {round(end_time-self.start_time,1)} s **")
        for key in self.telemetry:
            SmartDashboard.putNumberArray("telemetry_"+ str(key), self.telemetry[key])

    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.end()
        print("\n" + f"** Interrupted {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
