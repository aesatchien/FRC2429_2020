from wpilib.command import Command
from wpilib import Timer

class ActuateGate(Command):
    """
    This command opens and closed the gate to release balls
    """

    def __init__(self, robot, direction=None, button=None, timeout=None):
        Command.__init__(self, name='ActuateGate')
        self.robot = robot
        self.direction = direction
        self.button = button
        self.timeout = timeout
        self.requires(robot.ball_handler)


    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        if self.direction == "open":
            self.robot.ball_handler.open_gate()
        elif self.direction == "close":
            self.robot.ball_handler.close_gate()
        elif self.direction == 'hold':
            self.robot.ball_handler.hold_gate()
        else:
            print("Something happened that I didn't understand in Ball Gate")
        if self.timeout:
            self.setTimeout(self.timeout)
        print("\n" + f"** Started {self.getName()} with input {self.direction} at {self.start_time} s **", flush=True)

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        pass

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        #return not self.button or not self.button.get()
        if self.button:
            return not self.button.get()
        elif self.timeout:
            return self.isTimedOut()
        else:
            return True

    def end(self):
        """Called once after isFinished returns true"""
        print("\n" + f"** Ended {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
        self.robot.ball_handler.hold_gate()
    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        #print("\n" + f"** Interrupted {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
        self.end()
