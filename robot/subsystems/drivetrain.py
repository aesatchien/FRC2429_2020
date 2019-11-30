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
        self.PID_dict_pos = {'kP':0.008, 'kI':0.0, 'kD':0.02, 'kIz':0, 'kFF':0.004, 'kMaxOutput':0.99, 'kMinOutput':-0.99}
        self.PID_dict_vel = {'kP': 0.001, 'kI': 1.0e-6, 'kD': 0.002, 'kIz': 0, 'kFF': 0.0001, 'kMaxOutput': 0.99, 'kMinOutput': -0.99}
        self.current_limit = 40
        self.x = 0
        self.y = 0
        self.previous_distance = 0

        # Configure drive motors
        try:
            self.spark_neo_l1 = rev.CANSparkMax(1,rev.MotorType.kBrushless)
            self.spark_neo_l2 = rev.CANSparkMax(2,rev.MotorType.kBrushless)
            self.spark_neo_r3 = rev.CANSparkMax(3, rev.MotorType.kBrushless)
            self.spark_neo_r4 = rev.CANSparkMax(4, rev.MotorType.kBrushless)
            # Not sure if speedcontrollergroups work with the single sparkmax in python - seems to complain
            self.speedgroup_left = SpeedControllerGroup(self.spark_neo_l1)
            self.speedgroup_right = SpeedControllerGroup(self.spark_neo_r3)
            self.differential_drive = DifferentialDrive(self.speedgroup_left, self.speedgroup_right)
            self.spark_PID_controller_right = self.spark_neo_r3.getPIDController()
            self.spark_PID_controller_left = self.spark_neo_l1.getPIDController()

            #wpilib.LiveWindow.addActuator("DriveTrain", "spark_neo_l1", self.spark_neo_l1)
            #wpilib.LiveWindow.addActuator("DriveTrain", "spark_neo_r3", self.spark_neo_r3)
            #wpilib.LiveWindow.addActuator("DriveTrain", "spark_neo_l2", self.spark_neo_l2)
            #wpilib.LiveWindow.addActuator("DriveTrain", "spark_neo_r4", self.spark_neo_r4)

            # Configure the RobotDrive
            self.differential_drive.setSafetyEnabled(True)
            self.differential_drive.setExpiration(0.1)
            #self.differential_drive.setSensitivity(0.5)
            self.differential_drive.setMaxOutput(1.0)

            # Configure encoders and controllers
            self.sparkneo_encoder_1= rev.CANSparkMax.getEncoder(self.spark_neo_l1)
            self.sparkneo_encoder_3 = rev.CANSparkMax.getEncoder(self.spark_neo_r3)
            # Not sure if this is the right place to put in an "inverted" factor but I can't find it in the API
            # also, doesn't seem to care what numbers I put here, so that's a problem
            self.sparkneo_encoder_1.setPositionConversionFactor(-4.0 * 3.141 )
            self.sparkneo_encoder_1.setPositionConversionFactor(-4.0 * 3.141 )
            self.sparkneo_encoder_1.

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
        self.differential_drive.arcadeDrive(-x_speed, self.twist_sensitivity * z_rotation, False)

    def stop(self):
        self.differential_drive.arcadeDrive(0,0)

    def smooth_drive(self, thrust,  twist):
        '''A less jittery way to drive with a joystick'''
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
                    self.current_twist =  self.current_twist + self.acceleration_limit
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
        self.spark_PID_controller_left.setReference(multiplier*velocity, rev.ControlType.kVelocity, 1)
        self.spark_PID_controller_right.setReference(-multiplier*velocity, rev.ControlType.kVelocity, 1)
        #self.differential_drive.feed()

    def goToSetPoint(self, set_point):
        multiplier = 1.0
        self.reset()
        self.spark_PID_controller_left.setReference(multiplier*set_point, rev.ControlType.kPosition)
        self.spark_PID_controller_right.setReference(-multiplier*set_point, rev.ControlType.kPosition)

    def reset(self):
        self.sparkneo_encoder_1.setPosition(0)
        self.sparkneo_encoder_3.setPosition(0)
        self.x = 0
        self.y = 0
        wpilib.Timer.delay(0.02)

    def configure_controllers(self, pid_only = False):
        '''Set the PIDs, etc for the controllers, slot 0 is position and slot 1 is velocity'''
        if not pid_only:
            controllers = [self.spark_neo_l1,self.spark_neo_l2,self.spark_neo_r3,self.spark_neo_r4]
            for controller in controllers:
                Timer.delay(0.01)
                controller.restoreFactoryDefaults()
                #controller.setIdleMode(rev.IdleMode.kCoast)
                Timer.delay(0.01)
                controller.setIdleMode(rev.IdleMode.kBrake)
                Timer.delay(0.01)
                controller.setSmartCurrentLimit(self.current_limit)
                Timer.delay(0.01)
            self.spark_neo_l2.follow(self.spark_neo_l1)
            self.spark_neo_r4.follow(self.spark_neo_r3)
        controllers = [self.spark_neo_l1, self.spark_neo_r3]
        for controller in controllers:
            controller.setParameter(rev.ConfigParameter.kP_0,self.PID_dict_pos['kP'])
            controller.setParameter(rev.ConfigParameter.kP_1, self.PID_dict_vel['kP'])
            controller.setParameter(rev.ConfigParameter.kI_0, self.PID_dict_pos['kI'])
            controller.setParameter(rev.ConfigParameter.kI_1, self.PID_dict_vel['kI'])
            controller.setParameter(rev.ConfigParameter.kD_0, self.PID_dict_pos['kD'])
            controller.setParameter(rev.ConfigParameter.kD_1, self.PID_dict_vel['kD'])
            controller.setParameter(rev.ConfigParameter.kIZone_0, self.PID_dict_pos['kIz'])
            controller.setParameter(rev.ConfigParameter.kIZone_1, self.PID_dict_vel['kIz'])
            controller.setParameter(rev.ConfigParameter.kF_0, self.PID_dict_pos['kFF'])
            controller.setParameter(rev.ConfigParameter.kF_1, self.PID_dict_vel['kFF'])
            controller.setParameter(rev.ConfigParameter.kOutputMax_0, self.PID_dict_pos['kMaxOutput'])
            controller.setParameter(rev.ConfigParameter.kOutputMax_1, self.PID_dict_vel['kMaxOutput'])
            controller.setParameter(rev.ConfigParameter.kOutputMin_0, self.PID_dict_pos['kMinOutput'])
            controller.setParameter(rev.ConfigParameter.kOutputMin_1, self.PID_dict_vel['kMinOutput'])

    def change_PIDs(self,factor=1, dict_0 = None, dict_1 = None):
        '''Pass a value to the and update the PIDs, probably use 1.5 and 0.67 to see how they change
        can also pass it a dictionary {'kP': 0.06, 'kI': 0.0, 'kD': 0, 'kIz': 0, 'kFF': 0} to set
        slot 0 (position) or slot 1 (velocity) '''
        keys = ['kP', 'kI', 'kD', 'kIz', 'kFF']
        for key in keys:
            if dict_0 == None:
                self.PID_dict_pos[key] *= factor
            else:
                self.PID_dict_pos[key] = dict_0[key]
            if dict_1 == None:
                self.PID_dict_vel[key] *= factor
            else:
                self.PID_dict_vel[key] = dict_1[key]

        self.configure_controllers(pid_only=True)
        self.display_PIDs()

    def display_PIDs(self):
        keys = ['kP', 'kI', 'kD', 'kIz', 'kFF']
        for key in keys:
            SmartDashboard.putNumber(key + '_0', self.PID_dict_pos[key])
            SmartDashboard.putNumber(key + '_1', self.PID_dict_vel[key])

    def print_PIDs(self):
        print(f"Pos: kP: {self.PID_dict_pos['kP']}  kI: {self.PID_dict_pos['kI']}  kD: {self.PID_dict_pos['kD']}  kIz: {self.PID_dict_pos['kIz']}  kFF: {self.PID_dict_pos['kFF']}")
        print(f"Vel: kP: {self.PID_dict_vel['kP']}  kI: {self.PID_dict_vel['kI']}  kD: {self.PID_dict_vel['kD']}  kIz: {self.PID_dict_vel['kIz']} kFF: {self.PID_dict_vel['kFF']}")

    def log(self):
        self.counter += 1
        if self.counter % 10 == 0:
            # start keeping track of where the robot is with an x and y position
            try:
                distance = 0.5*(self.sparkneo_encoder_1.getPosition() - self.sparkneo_encoder_3.getPosition())
            except:
                print(f"Problem: encoder position returns {self.sparkneo_encoder_1.getPosition()}")
                distance = 0
            self.x = self.x + (distance-self.previous_distance) * math.sin(math.radians(self.robot.navigation.get_angle()))
            self.y = self.y + (distance-self.previous_distance) * math.cos(math.radians(self.robot.navigation.get_angle()))
            self.previous_distance = distance
            SmartDashboard.putNumber("Robot X", round(self.x,2))
            SmartDashboard.putNumber("Robot Y", round(self.y,2))
            SmartDashboard.putNumber("Position Enc1", round(self.sparkneo_encoder_1.getPosition(),2))
            SmartDashboard.putNumber("Position Enc3", round(self.sparkneo_encoder_3.getPosition(),2))
            SmartDashboard.putNumber("Velocity Enc1", round(self.sparkneo_encoder_1.getVelocity(),2))
            SmartDashboard.putNumber("Velocity Enc3", round(self.sparkneo_encoder_3.getVelocity(),2))
        if self.counter % 1000 == 0:
            self.display_PIDs()
