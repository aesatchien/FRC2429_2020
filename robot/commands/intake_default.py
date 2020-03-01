from wpilib.command import Command
from wpilib import Timer
from wpilib import SendableBuilder
class Intake_Default(Command):
    """
    This command sets the intake
    """

    def __init__(self, robot, power=0.1, button=None, end_power=0, timeout=3):
        Command.__init__(self, name='Intake')
        self.requires(robot.intake)
        self.robot = robot
        self.timeout = timeout
        self.button = button
        self.power = power
        self.end_power = end_power
        self.max_power = 0.8


    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} with power {self.power} at {self.start_time} s **", flush=True)
        self.setTimeout(self.timeout)

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        if self.robot.oi.co_buttonB.get():
            self.robot.ball_handler.run_intake(power=-0.7)
        elif self.robot.oi.buttonA.get():
            self.robot.ball_handler.run_intake(power=0.5)
        else:
            self.robot.ball_handler.run_intake(power=0)

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        if self.button is not None:
            return not self.button.get()
        else:
            return self.isTimedOut()


    def end(self):
        """Called once after isFinished returns true"""
        print("\n" + f"** Ended {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
        # Note to self: do not reset self.power here!
        self.robot.ball_handler.run_intake(self.end_power)
    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        #print("\n" + f"** Interrupted {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
        self.end()

