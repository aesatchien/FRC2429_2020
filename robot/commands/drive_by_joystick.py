from wpilib.command import Command
from wpilib import Timer
import math

class DriveByJoystick(Command):
    """
    This allows Logitech gamepad to drive the robot. It is always running
    except when interrupted by another command.
    """

    def __init__(self, robot):
        #super().__init__()
        Command.__init__(self, name='DriveByJoystick')
        self.requires(robot.drivetrain)
        self.robot = robot
        self.twist_sensitivity = 1.0
        # correction variables
        self.is_twist_active = False
        self.heading = 0
        self.corrected_twist = 0
        self.twist_deadzone = 0.05
        self.xy_deadzone = 0.15
        self.leadtime = 0.1
        self.execution_count = 0
        self.previous_twist_error = 0
        self.gyro_error = 0
        self.kp_gyro = 0.03
        self.kd_gyro = 0.3


    def initialize(self):
        """Called just before this Command runs the first time."""
        # perhaps here we need to set the contol mode for the controllers ... not sure about this
        self.is_twist_active = False
        self.corrected_twist = 0
        self.execution_count = 0
        self.previous_twist_error = 0
        self.gyro_error = 0
        self.heading = self.robot.navigation.get_angle()
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time,1)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        is_correcting = False # change this if you want to just send the joystick straight through
        if is_correcting:
            current_twist = self.robot.oi.stick.getRawAxis(4)
            thrust = -self.robot.oi.stick.getRawAxis(1)
            strafe = self.robot.oi.stick.getRawAxis(0)
            self.gyro_error = (self.heading - self.robot.navigation.get_angle())
            if self.is_twist_active and math.fabs(current_twist) < self.twist_deadzone:
                # doing nothing with twist but still driving
                if math.sqrt(strafe**2 + thrust**2) > self.xy_deadzone:
                    # actually moving on purpose
                    self.corrected_twist = self.kp_gyro*self.gyro_error + self.kd_gyro*(self.gyro_error-self.previous_twist_error)
                else:
                    # just sitting still
                    self.corrected_twist = 0
            elif self.is_twist_active and math.fabs(current_twist) < self.twist_deadzone:
                # we were twisting last time but not now
                self.is_twist_active = False
                self.heading = self.robot.navigation.get_angle()
                self.corrected_twist = 0
            elif not self.is_twist_active and math.fabs(current_twist) > self.twist_deadzone:
                # we were not twisting last time and we are now
                self.is_twist_active = True
                self.corrected_twist = current_twist
            elif self.is_twist_active and math.fabs(current_twist) > self.twist_deadzone:
                # we were twisting and still are
                self.corrected_twist = current_twist
            else:
                # something slipped through
                #print('Unanticipated case in mechanum drive correction')
                self.corrected_twist = current_twist
            self.execution_count += 1
            self.robot.drivetrain.smooth_drive(thrust=thrust, strafe=strafe, twist = self.corrected_twist)
            self.previous_twist_error = self.heading - self.robot.navigation.get_angle()

        else:
            #self.robot.drivetrain.smooth_drive(self.robot.oi.stick.getRawAxis(1), -self.twist_sensitivity*self.robot.oi.stick.getRawAxis(4))
            #self.robot.drivetrain.spark_with_stick(thrust=-self.robot.oi.stick.getRawAxis(1), strafe=self.robot.oi.stick.getRawAxis(0), z_rotation=self.twist_sensitivity * self.robot.oi.stick.getRawAxis(4))
            self.robot.drivetrain.mecanum_velocity_cartesian(thrust=-self.robot.oi.stick.getRawAxis(1),
                                               strafe=self.robot.oi.stick.getRawAxis(0),
                                               z_rotation=self.twist_sensitivity * self.robot.oi.stick.getRawAxis(4))

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return False

    def end(self):
        """Called once after isFinished returns true"""
        self.robot.drivetrain.stop()
        print("\n" + f"** Ended {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **", flush=True)
    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.robot.drivetrain.stop()
        print("\n" + f"** Interrupted {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **", flush=True)
