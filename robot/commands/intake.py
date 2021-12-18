from wpilib.command import Command
from wpilib import Timer
from wpilib import SendableBuilder
class Intake(Command):
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

        self.unpressed_min = 5
        self.unpressed_counter = 0


    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} with power {self.power} at {self.start_time} s **", flush=True)
        self.setTimeout(self.timeout)
        self.unpressed_counter = 0

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        if self.button == self.robot.oi.axisButtonRT:
            self.power = (0.5+ self.robot.oi.stick.getRawAxis(3)/2.0)
        elif self.button == self.robot.oi.axisButtonLT:
            self.power = -(0.5 + self.robot.oi.stick.getRawAxis(2)/2)
        if self.robot.oi.competition_mode:
            if self.button == self.robot.oi.co_axisButtonRT:
                self.power = (0.5 + self.robot.oi.co_stick.getRawAxis(3)/2.0)
            elif self.button == self.robot.oi.co_axisButtonLT:
                self.power = -(0.5 + self.robot.oi.co_stick.getRawAxis(2) /2.0)
        self.robot.intake.run_intake(self.power)

        pressed = self.button.get()
        if not pressed:
            self.unpressed_counter += 1
        else:
            self.unpressed_counter = 0

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        if self.button is not None:
            return self.unpressed_counter >= self.unpressed_min
        else:
            return self.isTimedOut()


    def end(self):
        """Called once after isFinished returns true"""
        print("\n" + f"** Ended {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
        # Note to self: do not reset self.power here!
        self.robot.intake.run_intake(self.end_power)
    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        #print("\n" + f"** Interrupted {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
        self.end()

