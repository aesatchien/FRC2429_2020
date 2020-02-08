from wpilib.command import Command
from wpilib import Timer
from wpilib import SmartDashboard


class UpdatePIDs(Command):
    """
    Update the PIDs by a pre-determined factor, or read them from the dashboard
    I started with a pre-determined factor, but then added on the ability to read from dashboard
    So it's a bit clunky.  Need to get rid of everything but the dashboard call - CJH
    """

    def __init__(self, robot, factor=1, from_dashboard=None):
        #super().__init__()
        Command.__init__(self, name='UpdatePIDs')
        self.robot = robot
        self.factor = factor
        self.from_dashboard = from_dashboard

    def initialize(self):
        """Called just before this Command runs the first time."""
        strip_name = lambda x: str(x)[1 + str(x).rfind('.'):-2]
        print(
            "\n" + f"** Started {self.getName()} with input {self.factor} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
        keys = ['kP', 'kI', 'kD', 'kIz', 'kFF']
        if self.from_dashboard == 'position':
            dict_0 = {}
            for key in keys:
                value = float(SmartDashboard.getNumber(str(key) + '_0', 0))
                dict_0.update({key: value})
            self.robot.drivetrain.change_PIDs(factor=1, dict_0=dict_0)
        if self.from_dashboard == 'velocity':
            dict_1 = {}
            for key in keys:
                value = float(SmartDashboard.getNumber(str(key) + '_1', 0))
                dict_1.update({key: value})
            self.robot.drivetrain.change_PIDs(factor=1, dict_1=dict_1)
        else:
            self.robot.drivetrain.change_PIDs(self.factor)

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        pass

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return True

    def end(self):
        """Called once after isFinished returns true"""
        self.robot.drivetrain.print_PIDs()
        # print("\n" + f"Ended {self.__class__} at {Timer.getFPGATimestamp() - self.robot.enabled_time} s")

    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.robot.drivetrain.print_PIDs()
        # print("\n" + f"Interrupted {self.__class__} at {Timer.getFPGATimestamp() - self.robot.enabled_time} s")
