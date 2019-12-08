# Attempt to convert 2019 Spartan Java to Python - 11/22/2019 CJH
import math
import wpilib
from wpilib.smartdashboard import SmartDashboard
from wpilib.command import Subsystem
from wpilib.speedcontrollergroup import SpeedControllerGroup
from wpilib.drive import DifferentialDrive
import rev
from wpilib import Timer
from commands.drive_by_joystick import DriveByJoystick


class DriveTrain(Subsystem):
    """
    The DriveTrain subsystem controls the robot's chassis and reads in
    information about its speed and position.
    """

    def __init__(self, robot):
        super().__init__()
        self.robot = robot

        # Add constants
        self.twist_sensitivity = 0.99
        self.current_thrust = 0
        self.current_twist = 0
        self.acceleration_limit = 0.1
        self.counter = 0
        # due to limitations in displaying digits in the Shuffleboard, we'll multiply these by 1000 and divide when updating the controllers
        self.PID_multiplier = 1000.
        self.PID_dict_pos = {'kP': 0.008, 'kI': 0.0, 'kD': 0.02, 'kIz': 0, 'kFF': 0.004, 'kMaxOutput': 0.99,
                             'kMinOutput': -0.99}
        self.PID_dict_vel = {'kP': 0.001, 'kI': 1.0e-6, 'kD': 0.002, 'kIz': 0, 'kFF': 0.0001, 'kMaxOutput': 0.99,
                             'kMinOutput': -0.99}
        self.current_limit = 40
        self.x = 0
        self.y = 0
        self.previous_distance = 0

        # Configure drive motors
        try:
            self.spark_neo_l1 = rev.CANSparkMax(1, rev.MotorType.kBrushless)
            self.spark_neo_l2 = rev.CANSparkMax(2, rev.MotorType.kBrushless)
            self.spark_neo_r3 = rev.CANSparkMax(3, rev.MotorType.kBrushless)
            self.spark_neo_r4 = rev.CANSparkMax(4, rev.MotorType.kBrushless)
            # Not sure if speedcontrollergroups work with the single sparkmax in python - seems to complain
            self.speedgroup_left = SpeedControllerGroup(self.spark_neo_l1)
            self.speedgroup_right = SpeedControllerGroup(self.spark_neo_r3)
            self.differential_drive = DifferentialDrive(self.speedgroup_left, self.speedgroup_right)
            self.spark_PID_controller_right = self.spark_neo_r3.getPIDController()
            self.spark_PID_controller_left = self.spark_neo_l1.getPIDController()

            # wpilib.LiveWindow.addActuator("DriveTrain", "spark_neo_l1", self.spark_neo_l1)
            # wpilib.LiveWindow.addActuator("DriveTrain", "spark_neo_r3", self.spark_neo_r3)
            # wpilib.LiveWindow.addActuator("DriveTrain", "spark_neo_l2", self.spark_neo_l2)
            # wpilib.LiveWindow.addActuator("DriveTrain", "spark_neo_r4", self.spark_neo_r4)

            # Configure the RobotDrive
            self.differential_drive.setSafetyEnabled(True)
            self.differential_drive.setExpiration(0.1)
            # self.differential_drive.setSensitivity(0.5)
            self.differential_drive.setMaxOutput(1.0)

            # Configure encoders and controllers
            self.sparkneo_encoder_1 = rev.CANSparkMax.getEncoder(self.spark_neo_l1)
            self.sparkneo_encoder_3 = rev.CANSparkMax.getEncoder(self.spark_neo_r3)
            wpilib.Timer.delay(0.02)
            # should be wheel_diameter * pi / gear_ratio - and for the old double reduction gear box
            # the gear ratio was either  5.67:1 or 4.17:1.  With the shifter (low gear) I think it was a 12.26.
            err_1 = self.sparkneo_encoder_1.setPositionConversionFactor(6.0 * 3.141 / 4.17)
            err_2 = self.sparkneo_encoder_3.setPositionConversionFactor(6.0 * 3.141 / 4.17)
            self.spark_neo_l1.setInverted(True)
            self.spark_neo_r3.setInverted(True)

            #err_1 = self.sparkneo_encoder_1.__setattr__(rev.ConfigParameter.kPositionConversionFactor, -6.0 * 3.141 / 4.17)
            #err_2 = self.sparkneo_encoder_3.__setattr__(rev.ConfigParameter.kPositionConversionFactor, -6.0 * 3.141 / 4.17)

            if err_1 != rev.CANError.kOK or err_2 != rev.CANError.kOK:
                print(f"Warning: drivetrain encoder issue with neo1 returning {err_1} and neo3 returning {err_2}")
            if robot.isReal():
                pass
            else:
                self.configure_controllers()
        except rev.CANError:
            print('Buncha CAN errors)')
        self.display_PIDs()

    def initDefaultCommand(self):
        """
        When other commands aren't using the drivetrain, allow arcade drive with the joystick.
        """
        self.setDefaultCommand(DriveByJoystick(self.robot))

    def spark_with_stick(self, x_speed, z_rotation):
        '''Simplest way to drive with a joystick'''
        self.differential_drive.arcadeDrive(x_speed, self.twist_sensitivity * z_rotation, False)

    def stop(self):
        self.differential_drive.arcadeDrive(0, 0)

    def smooth_drive(self, thrust, twist):
        '''A less jittery way to drive with a joystick
        TODO: See if this can be implemented in hardware - seems like the acceleration limit can be set there
        '''
        deadzone = 0.02
        if math.fabs(thrust) < deadzone:
            self.current_thrust = 0
        else:
            if math.fabs(thrust - self.current_thrust) < self.acceleration_limit:
                self.current_thrust = thrust
                self.current_thrust = thrust
            else:
                if thrust - self.current_thrust > 0:
                    self.current_thrust = self.current_thrust + self.acceleration_limit
                else:
                    self.current_thrust = self.current_thrust - self.acceleration_limit

        if math.fabs(twist) < deadzone:
            self.current_twist = 0
        else:
            if math.fabs(twist - self.current_twist) < self.acceleration_limit:
                self.current_twist = twist
            else:
                if twist - self.current_twist > 0:
                    self.current_twist = self.current_twist + self.acceleration_limit
                else:
                    self.current_twist = self.current_twist - self.acceleration_limit
        self.differential_drive.arcadeDrive(self.current_thrust, self.current_twist, True)

    def tank_drive(self, left, right):
        '''Not sure why we would ever need this, but it's here if we do'''
        self.differential_drive.tankDrive(left, right)

    def get_position(self):
        ''':returns: The encoder position of one of the Neos'''
        return self.sparkneo_encoder_1.getPosition()

    def set_velocity(self, velocity):
        multiplier = 1.0
        self.spark_PID_controller_left.setReference(multiplier * velocity, rev.ControlType.kVelocity, 1)
        self.spark_PID_controller_right.setReference(-multiplier * velocity, rev.ControlType.kVelocity, 1)
        # self.differential_drive.feed()

    def goToSetPoint(self, set_point):
        multiplier = 1.0
        self.reset()
        self.spark_PID_controller_left.setReference(multiplier * set_point, rev.ControlType.kPosition)
        self.spark_PID_controller_right.setReference(-multiplier * set_point, rev.ControlType.kPosition)

    def reset(self):
        err_1 = self.sparkneo_encoder_1.setPosition(0)
        err_2 = self.sparkneo_encoder_3.setPosition(0)
        if err_1 != rev.CANError.kOK or err_2 != rev.CANError.kOK:
            print(f"Warning: drivetrain reset issue with neo1 returning {err_1} and neo3 returning {err_2}")
        self.x = 0
        self.y = 0
        wpilib.Timer.delay(0.02)

    def configure_controllers(self, pid_only=False):
        '''Set the PIDs, etc for the controllers, slot 0 is position and slot 1 is velocity'''
        error_list = []
        if not pid_only:
            controllers = [self.spark_neo_l1, self.spark_neo_l2, self.spark_neo_r3, self.spark_neo_r4]
            for controller in controllers:
                Timer.delay(0.01)
                error_list.append(controller.restoreFactoryDefaults())
                Timer.delay(0.01)
                #looks like they orphaned the setIdleMode - it doesn't work.  Try ConfigParameter
                #error_list.append(controller.setIdleMode(rev.IdleMode.kBrake))
                controller.setParameter(rev.ConfigParameter.kIdleMode, rev.IdleMode.kBrake)
                Timer.delay(0.01)
                error_list.append(controller.setSmartCurrentLimit(self.current_limit))
                Timer.delay(0.01)
            err_1 = self.spark_neo_l2.follow(self.spark_neo_l1)
            err_2 = self.spark_neo_r4.follow(self.spark_neo_r3)
            if err_1 != rev.CANError.kOK or err_2 != rev.CANError.kOK:
                print(f"Warning: drivetrain follower issue with neo2 returning {err_1} and neo4 returning {err_2}")
        controllers = [self.spark_neo_l1, self.spark_neo_r3]
        for controller in controllers:
            error_list.append(controller.setParameter(rev.ConfigParameter.kP_0, self.PID_dict_pos['kP']))
            error_list.append(controller.setParameter(rev.ConfigParameter.kP_1, self.PID_dict_vel['kP']))
            error_list.append(controller.setParameter(rev.ConfigParameter.kI_0, self.PID_dict_pos['kI']))
            error_list.append(controller.setParameter(rev.ConfigParameter.kI_1, self.PID_dict_vel['kI']))
            error_list.append(controller.setParameter(rev.ConfigParameter.kD_0, self.PID_dict_pos['kD']))
            error_list.append(controller.setParameter(rev.ConfigParameter.kD_1, self.PID_dict_vel['kD']))
            error_list.append(controller.setParameter(rev.ConfigParameter.kIZone_0, self.PID_dict_pos['kIz']))
            error_list.append(controller.setParameter(rev.ConfigParameter.kIZone_1, self.PID_dict_vel['kIz']))
            error_list.append(controller.setParameter(rev.ConfigParameter.kF_0, self.PID_dict_pos['kFF']))
            error_list.append(controller.setParameter(rev.ConfigParameter.kF_1, self.PID_dict_vel['kFF']))
            error_list.append(controller.setParameter(rev.ConfigParameter.kOutputMax_0, self.PID_dict_pos['kMaxOutput']))
            error_list.append(controller.setParameter(rev.ConfigParameter.kOutputMax_1, self.PID_dict_vel['kMaxOutput']))
            error_list.append(controller.setParameter(rev.ConfigParameter.kOutputMin_0, self.PID_dict_pos['kMinOutput']))
            error_list.append(controller.setParameter(rev.ConfigParameter.kOutputMin_1, self.PID_dict_vel['kMinOutput']))

        # if 1 in error_list or 2 in error_list:
        #    print(f'Issue in configuring controllers: {error_list}')
        # else:
        print(f'Results of configuring controllers: {error_list}')

    def change_PIDs(self, factor=1, dict_0=None, dict_1=None):
        '''Pass a value to the and update the PIDs, probably use 1.5 and 0.67 to see how they change
        can also pass it a dictionary {'kP': 0.06, 'kI': 0.0, 'kD': 0, 'kIz': 0, 'kFF': 0} to set
        slot 0 (position) or slot 1 (velocity) '''
        keys = ['kP', 'kI', 'kD', 'kIz', 'kFF']
        for key in keys:
            if dict_0 == None:
                self.PID_dict_pos[key] *= factor
            else:
                self.PID_dict_pos[key] = dict_0[key] / self.PID_multiplier
                SmartDashboard.putString("alert",
                                         f"Pos: kP: {self.PID_dict_pos['kP']}  kI: {self.PID_dict_pos['kI']}  kD: {self.PID_dict_pos['kD']}  kIz: {self.PID_dict_pos['kIz']}  kFF: {self.PID_dict_pos['kFF']}")
            if dict_1 == None:
                self.PID_dict_vel[key] *= factor
            else:
                self.PID_dict_vel[key] = dict_1[key] / self.PID_multiplier
                SmartDashboard.putString("alert",
                                         f"Vel: kP: {self.PID_dict_vel['kP']}  kI: {self.PID_dict_vel['kI']}  kD: {self.PID_dict_vel['kD']}  kIz: {self.PID_dict_vel['kIz']} kFF: {self.PID_dict_vel['kFF']}")

        self.configure_controllers(pid_only=True)
        self.display_PIDs()

    def display_PIDs(self):
        keys = ['kP', 'kI', 'kD', 'kIz', 'kFF']
        for key in keys:
            SmartDashboard.putNumber(key + '_0', self.PID_multiplier * self.PID_dict_pos[key])
            SmartDashboard.putNumber(key + '_1', self.PID_multiplier * self.PID_dict_vel[key])

    def print_PIDs(self):
        print(f"Pos: kP: {self.PID_dict_pos['kP']}  kI: {self.PID_dict_pos['kI']}  kD: {self.PID_dict_pos['kD']}  kIz: {self.PID_dict_pos['kIz']}  kFF: {self.PID_dict_pos['kFF']}")
        print(f"Vel: kP: {self.PID_dict_vel['kP']}  kI: {self.PID_dict_vel['kI']}  kD: {self.PID_dict_vel['kD']}  kIz: {self.PID_dict_vel['kIz']} kFF: {self.PID_dict_vel['kFF']}")

    def log(self):
        self.counter += 1
        if self.counter % 10 == 0:
            # start keeping track of where the robot is with an x and y position
            try:
                distance = 0.5 * (self.sparkneo_encoder_1.getPosition() - self.sparkneo_encoder_3.getPosition())
            except:
                print(f"Problem: encoder position returns {self.sparkneo_encoder_1.getPosition()}")
                distance = 0
            self.x = self.x + (distance - self.previous_distance) * math.sin(
                math.radians(self.robot.navigation.get_angle()))
            self.y = self.y + (distance - self.previous_distance) * math.cos(
                math.radians(self.robot.navigation.get_angle()))
            self.previous_distance = distance
            # send values to the dash to make sure encoders are working well
            SmartDashboard.putNumber("Robot X", round(self.x, 2))
            SmartDashboard.putNumber("Robot Y", round(self.y, 2))
            SmartDashboard.putNumber("Position Enc1", round(self.sparkneo_encoder_1.getPosition(), 2))
            SmartDashboard.putNumber("Position Enc3", round(self.sparkneo_encoder_3.getPosition(), 2))
            SmartDashboard.putNumber("Velocity Enc1", round(self.sparkneo_encoder_1.getVelocity(), 2))
            SmartDashboard.putNumber("Velocity Enc3", round(self.sparkneo_encoder_3.getVelocity(), 2))
            SmartDashboard.putNumber("Power M1", round(self.spark_neo_l1.getAppliedOutput(), 2))
            SmartDashboard.putNumber("Power M2", round(self.spark_neo_l2.getAppliedOutput(), 2))
        if self.counter % 1000 == 0:
            self.display_PIDs()
            SmartDashboard.putString("alert",
                                     f"Position: ({round(self.x, 1)},{round(self.y, 1)})  Time: {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)}")
            SmartDashboard.putString("Controller1 Idle", str(self.spark_neo_l1.getIdleMode()))
            SmartDashboard.putNumber("Enc1 Conversion", self.sparkneo_encoder_1.getPositionConversionFactor())
            SmartDashboard.putNumber("Auto Distance", 10)
            SmartDashboard.putNumber("Auto Rotation", 10)
