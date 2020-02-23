from wpilib.command import Command
from wpilib import Timer, SmartDashboard

class DpadDrive(Command):
    """
    This allows Logitech gamepad's dpad to drive the robot. It overrides the stick.
    Change the drive_power and twist_power variables to change how it reacts
    """

    def __init__(self, robot, state, button):
        #super().__init__()
        Command.__init__(self, name='DpadDrive')
        self.requires(robot.drivetrain)
        self.robot = robot
        self.state = state
        self.button = button
        self.drive_power = 0.3
        self.co_drive_power = 0.1
        self.strafe_power = 0.8
        self.co_strafe_power = 0.25
        self.kp_twist = 0.03
        self.direction = 1 # change this to -1 change all directions quickly

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time,1)
        print("\n" + f"** Started {self.getName()} with input {self.state} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} with input {self.state} at {self.start_time} s **")
        self.heading = self.robot.navigation.get_angle()
    def execute(self):
        """
        Called repeatedly when this Command is scheduled to run
        Should not have to change this if it works - just change variables above
        """
        # easy to correct for heading drift - we know we're driving straight
        twist_correction = self.kp_twist*(self.heading-self.robot.navigation.get_angle())
        twist_correction = 0
        if self.button == self.robot.oi.povButtonUp:
            thrust=self.drive_power*self.direction; strafe=0; twist=twist_correction
        if self.button == self.robot.oi.povButtonDown:
            thrust=-self.drive_power*self.direction; strafe=0; twist=twist_correction
        if self.button == self.robot.oi.povButtonLeft:
            thrust=0; strafe=-self.strafe_power * self.direction; twist=twist_correction
        if self.button == self.robot.oi.povButtonRight:
            thrust=0; strafe=self.strafe_power * self.direction; twist=twist_correction

        # you guys messed this up - needs to know if there is a co stick
        if self.robot.oi.competition_mode:
            if self.button == self.robot.oi.co_povButtonUp:
                thrust=self.co_drive_power*self.direction; strafe=0; twist=twist_correction
            if self.button == self.robot.oi.co_povButtonDown:
                thrust=-self.co_drive_power*self.direction; strafe=0; twist=twist_correction
            if self.button == self.robot.oi.co_povButtonLeft:
                thrust=0; strafe=-self.co_strafe_power * self.direction; twist=twist_correction
            if self.button == self.robot.oi.co_povButtonRight:
                thrust=0; strafe=self.co_strafe_power * self.direction; twist=twist_correction

        #really need to decide on how we're going to drive - smooth or pure stick or velocity
        #self.robot.drivetrain.spark_with_stick(thrust=thrust, strafe=strafe, z_rotation=twist)
        self.robot.drivetrain.mecanum_velocity_cartesian(thrust=thrust, strafe=strafe, z_rotation=twist)
        #self.robot.drivetrain.smooth_drive(thrust=thrust, strafe=strafe, twist=twist)

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return not self.button.get()

    def end(self):
        """Called once after isFinished returns true"""
        self.robot.drivetrain.stop()
        print("\n" + f"** Ended {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.robot.drivetrain.stop()
        print("\n" + f"** Interrupted {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
