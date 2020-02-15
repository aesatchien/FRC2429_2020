from wpilib.command import Command
from wpilib import Timer

class ActuateGate(Command):
    """
    This command opens and closed the piston
    """

    def __init__(self, robot, direction=None):
        Command.__init__(self, name='ActuateGate')
        self.robot = robot
        self.direction = direction
        strip_name = lambda x: str(x)[1 + str(x).rfind('.'):-2]
        self.name = strip_name(self.__class__)

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        if self.direction == "open":
            self.robot.peripherals.open_gate()
        elif self.direction == "close":
            self.robot.peripherals.close_gate()
        else:
            print("Something happened that I didn't understand in GateServo")

        print("\n" + f"** Started {self.name} with input {self.direction} at {self.start_time} s **", flush=True)
        #self.robot.pneumatics.actuate_solenoid(self.direction)

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        pass

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return True

    def end(self):
        """Called once after isFinished returns true"""
        #print("\n" + f"** Ended {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        #print("\n" + f"** Interrupted {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
