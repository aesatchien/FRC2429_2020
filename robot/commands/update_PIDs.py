from wpilib.command import Command
from wpilib import Timer
from wpilib.smartdashboard import SmartDashboard

class UpdatePIDs(Command):
    """
    Update the PIDs by a pre-determined factor
    """

    def __init__(self, robot, factor = 1, from_dashboard = False):
        super().__init__()
        self.robot = robot
        self.factor = factor
        self.from_dashboard = from_dashboard

    def initialize(self):
        """Called just before this Command runs the first time."""
        print("\n" + f"Started {self.__class__} with input {self.factor} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s")
        if self.from_dashboard:
            keys = ['kP', 'kI', 'kD', 'kIz', 'kFF']
            dict_0 = {}
            dict_1 = {}
            for key in keys:
                value = float(SmartDashboard.getNumber(str(key) + '_0', 0))
                dict_0.update({key:value})
                value = float(SmartDashboard.getNumber(str(key) + '_1', 0))
                dict_1.update({key: value})
            self.robot.drivetrain.change_PIDs(factor=1, dict_0=dict_0)
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
        #print("\n" + f"Interrupted {self.__class__} at {Timer.getFPGATimestamp() - self.robot.enabled_time} s")
