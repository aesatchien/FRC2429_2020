from wpilib.command import Command
from wpilib import Timer

class ActuateGate(Command):
    """
    This command opens and closed the piston
    """

    def __init__(self, robot, direction=None, button=None):
        Command.__init__(self, name='ActuateGate')
        self.robot = robot
        self.direction = direction
        self.button = button

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        if self.direction == "open":
            self.robot.ball_handler.open_gate()
        elif self.direction == "close":
            self.robot.ball_handler.close_gate()
        else:
            print("Something happened that I didn't understand in Ball Gate")

        print("\n" + f"** Started {self.getName()} with input {self.direction} at {self.start_time} s **", flush=True)

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        pass

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return not self.button or not self.button.get()

    def end(self):
        """Called once after isFinished returns true"""
        #print("\n" + f"** Ended {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
        #self.robot.ball_handler.hopper_spark.set(0)
    def interrupted(self):
        """Called w

        hen another command which requires one or more of the same subsystems is scheduled to run."""
        #print("\n" + f"** Interrupted {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
