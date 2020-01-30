from wpilib.command import Command
from wpilib import Timer
from wpilib import SendableBuilder
class Intake(Command):
    """
    This command opens and cloed the piston
    """

    def __init__(self, robot, power=0.1, button=None):
        Command.__init__(self, name='intake')
        self.requires(robot.peripherals)
        self.robot = robot
        self.button = button
        strip_name = lambda x: str(x)[1 + str(x).rfind('.'):-2]
        self.name = strip_name(self.__class__)
        self.power = power

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.name} with power {self.power} at {self.start_time} s **", flush=True)

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        if self.button == self.robot.oi.axisButtonRT:
            self.power = self.robot.oi.stick.getRawAxis(3)
        elif self.button == self.robot.oi.axisButtonLT:
            self.power = -self.robot.oi.stick.getRawAxis(2)
        if self.robot.oi.competition_mode:
            if self.button == self.robot.oi.co_axisButtonRT:
                self.power = self.robot.oi.co_stick.getRawAxis(3)
            elif self.button == self.robot.oi.co_axisButtonLT:
                self.power = -self.robot.oi.co_stick.getRawAxis(2)
        self.robot.peripherals.run_intake(self.power)

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return not self.button.get()


    def end(self):
        """Called once after isFinished returns true"""
        print("\n" + f"** Ended {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
        # Note to self: do not reset self.power here!
        self.robot.peripherals.run_intake(0)
    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        #print("\n" + f"** Interrupted {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
        self.end()

