from wpilib.command import Command
from wpilib import Timer, SmartDashboard

class DpadDrive(Command):
    """
    This allows Logitech gamepad's dpad to drive the robot. It overrides the stick.
    Change the drive_power and twist_power variables to change how it reacts
    """

    def __init__(self, robot, state, button):
        super().__init__()
        self.requires(robot.drivetrain)
        self.robot = robot
        self.state = state
        self.button = button
        self.drive_power = 0.4
        self.strafe_power = 0.60
        self.kp_twist = 0.03
        self.direction = 1 # change this to -1 change all directions quickly
        strip_name = lambda x: str(x)[1 + str(x).rfind('.'):-2]
        self.name = strip_name(self.__class__)

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time,1)
        print("\n" + f"** Started {self.name} with input {self.state} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.name} with input {self.state} at {self.start_time} s **")
        self.heading = self.robot.navigation.get_angle()
    def execute(self):
        """
        Called repeatedly when this Command is scheduled to run
        Should not have to change this if it works - just change variables above
        """
        # easy to correct for heading drift - we know we're driving straight
        twist_correction = self.kp_twist*(self.heading-self.robot.navigation.get_angle())
        if self.state.lower() == "up":
            thrust=self.drive_power*self.direction; strafe=0; twist=twist_correction
        if self.state.lower() == "down":
            thrust=-self.drive_power*self.direction; strafe=0; twist=twist_correction
        if self.state.lower() == "right":
            thrust=0; strafe=-self.strafe_power * self.direction; twist=twist_correction
        if self.state.lower() == "left":
            thrust=0; strafe=self.strafe_power * self.direction; twist=twist_correction
        #self.robot.drivetrain.spark_with_stick(thrust=thrust, strafe=strafe, z_rotation=z_rotation)
        self.robot.drivetrain.smooth_drive(thrust=thrust, strafe=strafe, twist=twist)

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return not self.button.get()

    def end(self):
        """Called once after isFinished returns true"""
        self.robot.drivetrain.stop()
        print("\n" + f"** Ended {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.robot.drivetrain.stop()
        print("\n" + f"** Interrupted {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
