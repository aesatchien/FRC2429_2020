from wpilib.command import Command
from wpilib import Timer
from wpilib import SmartDashboard

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
    # may need to use variables at some point ... leftover from pacgoat example
    tolerance = 0.5
    KP = -1.0 / 5.0


    def __init__(self, robot, setpoint=10, control_type='position', button = 'None'):
        """The constructor"""
        super().__init__()
        # Signal that we require ExampleSubsystem
        self.requires(robot.drivetrain)
        self.setpoint = setpoint
        self.robot = robot
        self.control_type = control_type
        self.button = button
        strip_name = lambda x: str(x)[1 + str(x).rfind('.'):-2]
        self.name = strip_name(self.__class__)


    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.name} with setpoint {self.setpoint} and control_type {self.control_type} at {self.start_time} s **")
        SmartDashboard.putString("alert", f"** Started {self.name} with setpoint {self.setpoint} and control_type {self.control_type} at {self.start_time} s **")
        if self.control_type == 'position':
            self.robot.drivetrain.goToSetPoint(self.setpoint)
        elif self.control_type == 'velocity':
            self.robot.drivetrain.set_velocity(self.setpoint)
        else:
            pass
        self.setTimeout(10)

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        self.robot.drivetrain.differential_drive.feed()

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        # somehow need to wait for the error level to get to a tolerance... request from drivetrain?
        if self.control_type == 'position':
            if self.setpoint > 0:
                return (self.setpoint - self.robot.drivetrain.get_position()) <= self.tolerance or self.isTimedOut()
            else:
                return (self.setpoint - self.robot.drivetrain.get_position()) >= self.tolerance or self.isTimedOut()
        elif self.control_type == 'velocity':
            return not self.button.get()
        else:
            return True

    def end(self):
        """Called once after isFinished returns true"""
        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Ended {self.name} at {end_time} s with a duration of {round(end_time-self.start_time,1)} s **")
        SmartDashboard.putString("alert", f"** Ended {self.name} at {end_time} s with a duration of {round(end_time-self.start_time,1)} s **")
        self.robot.drivetrain.stop()

    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.end()
        print("\n" + f"** Interrupted {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
