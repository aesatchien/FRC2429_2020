# Attempt to convert 2019 Spartan Java to Python - 11/22/2019 CJH
import math
import wpilib
from wpilib import SmartDashboard
from wpilib.command import Subsystem
from wpilib import SpeedControllerGroup
from wpilib.drive import DifferentialDrive
from wpilib.drive import MecanumDrive
import rev
from wpilib import Timer
from commands.drive_by_joystick import DriveByJoystick

class DriveTrain(Subsystem):
    """
    The DriveTrain subsystem controls the robot's chassis and reads in
    information about its speed and position.
    """

    def __init__(self, robot):
        super().__init__("drivetrain")
        self.robot = robot
        self.error_dict = {}
        # Add constants and helper variables
        self.twist_sensitivity = 0.5
        self.current_thrust = 0
        self.current_twist = 0
        self.current_strafe = 0
        self.acceleration_limit = 0.05
        self.counter = 0
        self.mecanum_power_limit = 0.6
        # due to limitations in displaying digits in the Shuffleboard, we'll multiply these by 1000 and divide when updating the controllers
        self.PID_multiplier = 1000.
        self.PID_dict_pos = {'kP': 0.010, 'kI': 5.0e-7, 'kD': 0.40, 'kIz': 0, 'kFF': 0.002, 'kMaxOutput': 0.99,
                             'kMinOutput': -0.99}
        self.PID_dict_vel = {'kP': 0.00015, 'kI': 8.0e-7, 'kD': 0.00, 'kIz': 0, 'kFF': 0.00022, 'kMaxOutput': 0.99,
                             'kMinOutput': -0.99}
        # something in the newest settings is broken, not sure what.
        self.PID_dict_vel = {'kP': 0.00015, 'kI': 0, 'kD': 0.00, 'kIz': 0, 'kFF': 0.00022, 'kMaxOutput': 0.1,
                             'kMinOutput': -0.1}
        # Smart Motion Coefficients - these don't seem to be writing for some reason... python is old?  just set with rev's program for now
        self.maxvel = 500  # rpm
        self.maxacc = 500
        self.current_limit = 40
        # tracking the robot across the field... easier with WCD
        self.x = 0
        self.y = 0
        self.previous_distance = 0
        self.is_limited = False
        self.deadband = 0.05

        # Configure drive motors

        if True: # or could be if self.robot.isReal():
            self.spark_neo_right_front = rev.CANSparkMax(1, rev.MotorType.kBrushless)
            self.spark_neo_right_rear = rev.CANSparkMax(2, rev.MotorType.kBrushless)
            self.spark_neo_left_front = rev.CANSparkMax(3, rev.MotorType.kBrushless)
            self.spark_neo_left_rear = rev.CANSparkMax(4, rev.MotorType.kBrushless)
            self.controllers = [self.spark_neo_left_front, self.spark_neo_left_rear,
                                self.spark_neo_right_front, self.spark_neo_right_rear]

            self.spark_PID_controller_right_front = self.spark_neo_right_front.getPIDController()
            self.spark_PID_controller_right_rear = self.spark_neo_right_rear.getPIDController()
            self.spark_PID_controller_left_front = self.spark_neo_left_front.getPIDController()
            self.spark_PID_controller_left_rear = self.spark_neo_left_rear.getPIDController()
            self.pid_controllers = [self.spark_PID_controller_left_front, self.spark_PID_controller_left_rear,
                                    self.spark_PID_controller_right_front, self.spark_PID_controller_right_rear]
            # wpilib.Timer.delay(0.02)

            # swap encoders to get sign right
            # changing them up for mechanum vs WCD
            self.sparkneo_encoder_1 = rev.CANSparkMax.getEncoder(self.spark_neo_left_front)
            self.sparkneo_encoder_2 = rev.CANSparkMax.getEncoder(self.spark_neo_left_rear)
            self.sparkneo_encoder_3 = rev.CANSparkMax.getEncoder(self.spark_neo_right_front)
            self.sparkneo_encoder_4 = rev.CANSparkMax.getEncoder(self.spark_neo_right_rear)
            self.encoders = [self.sparkneo_encoder_1, self.sparkneo_encoder_2, self.sparkneo_encoder_3, self.sparkneo_encoder_4]

            # Configure encoders and controllers
            # should be wheel_diameter * pi / gear_ratio - and for the old double reduction gear box
            # the gear ratio was either  5.67:1 or 4.17:1.  With the shifter (low gear) I think it was a 12.26.
            conversion_factor = 8.0 * 3.141 / 4.17
            for ix, encoder in enumerate(self.encoders):
                self.error_dict.update({'conv_'+ str(ix): encoder.setPositionConversionFactor(conversion_factor)})

            # wpilib.Timer.delay(0.02)
            # TODO - figure out if I want to invert the motors or the encoders
            self.spark_neo_left_front.setInverted(False)
            self.spark_neo_left_rear.setInverted(False)
            self.spark_neo_right_front.setInverted(False)
            self.spark_neo_right_rear.setInverted(False)

            self.configure_controllers()
            self.display_PIDs()

        else: # for simulation only, but the CANSpark is getting closer to behaving in sim
            # get a pretend drivetrain for the simulator
            self.spark_neo_left_front = wpilib.Talon(1)
            self.spark_neo_left_rear = wpilib.Talon(2)
            self.spark_neo_right_front = wpilib.Talon(3)
            self.spark_neo_right_rear = wpilib.Talon(4)

        # Not sure if speedcontrollergroups work with the single sparkmax in python - seems to complain
        drive_type = 'mechanum'
        print("Enabling mechanum drive!")
        if drive_type == 'wcd':
            # WCD
            err_1 = self.spark_neo_left_rear.follow(self.spark_neo_left_front)
            err_2 = self.spark_neo_right_rear.follow(self.spark_neo_right_front)
            if err_1 != rev.CANError.kOk or err_2 != rev.CANError.kOk:
                print(f"Warning: drivetrain follower issue with neo2 returning {err_1} and neo4 returning {err_2}")
            self.speedgroup_left = SpeedControllerGroup(self.spark_neo_left_front)
            self.speedgroup_right = SpeedControllerGroup(self.spark_neo_right_front)
            self.differential_drive = DifferentialDrive(self.speedgroup_left, self.speedgroup_right)
            self.drive = self.differential_drive
            self.differential_drive.setMaxOutput(1.0)
        if drive_type == 'mechanum':
            # Mechanum
            # TODO: Reset followers in software
            self.speedgroup_lfront = SpeedControllerGroup(self.spark_neo_left_front)
            self.speedgroup_lrear = SpeedControllerGroup(self.spark_neo_left_rear)
            self.speedgroup_rfront = SpeedControllerGroup(self.spark_neo_right_front)
            self.speedgroup_rrear = SpeedControllerGroup(self.spark_neo_right_rear)
            self.mechanum_drive = MecanumDrive(self.speedgroup_lfront, self.speedgroup_lrear, self.speedgroup_rfront,
                                               self.speedgroup_rrear)
            # self.mechanum_drive = MecanumDrive(self.spark_neo_left_front, self.spark_neo_left_rear, self.spark_neo_right_front, self.spark_neo_right_rear)
            self.mechanum_drive.setMaxOutput(self.mecanum_power_limit)
            self.drive = self.mechanum_drive

        self.drive.setSafetyEnabled(True)
        self.drive.setExpiration(0.1)
        # self.differential_drive.setSensitivity(0.5)
        # wpilib.LiveWindow.addActuator("DriveTrain", "spark_neo_l1", self.spark_neo_l1)
        # wpilib.LiveWindow.addActuator("DriveTrain", "spark_neo_r3", self.spark_neo_r3)
        # wpilib.LiveWindow.addActuator("DriveTrain", "spark_neo_l2", self.spark_neo_l2)
        # wpilib.LiveWindow.addActuator("DriveTrain", "spark_neo_r4", self.spark_neo_r4)

    def initDefaultCommand(self):
        """
        When other commands aren't using the drivetrain, allow arcade drive with the joystick.
        """
        self.setDefaultCommand(DriveByJoystick(self.robot))

    def spark_with_stick(self, thrust=0, strafe=0, z_rotation=0, gyroAngle=0):
        '''Simplest way to drive with a joystick'''
        # self.differential_drive.arcadeDrive(x_speed, self.twist_sensitivity * z_rotation, False)
        self.mechanum_drive.driveCartesian(xSpeed=thrust, ySpeed=strafe, zRotation=self.twist_sensitivity * z_rotation)

    def stop(self):
        # self.differential_drive.arcadeDrive(0, 0)
        self.mechanum_drive.driveCartesian(0, 0, 0)

    def smooth_drive(self, thrust, strafe, twist):
        '''A less jittery way to drive with a joystick
        TODO: See if this can be implemented in hardware - seems like the acceleration limit can be set there
        TODO: Should be ok to slow down quickly, but not speed up - check this code
        TODO: Set a smartdash to see if we are in software limit mode - like with a boolean
        '''
        deadzone = 0.05
        self.is_limited = False
        # thrust section
        if math.fabs(thrust) < deadzone:
            self.current_thrust = 0
        else:
            if math.fabs(thrust - self.current_thrust) < self.acceleration_limit:
                self.current_thrust = thrust
            else:
                if thrust - self.current_thrust > 0:
                    # accelerating forward
                    self.current_thrust = self.current_thrust + self.acceleration_limit
                    self.is_limited = True
                elif (thrust - self.current_thrust) < 0 and thrust > 0:
                    # ok to slow down quickly, although really the deadzone should catch this
                    self.current_thrust = thrust
                elif (thrust - self.current_thrust) > 0 and thrust < 0:
                    # ok to slow down quickly, although really the deadzone should catch this
                    self.current_thrust = thrust
                else:
                    # accelerating backward
                    self.current_thrust = self.current_thrust - self.acceleration_limit
                    self.is_limited = True
        # strafe section
        if math.fabs(strafe) < deadzone:
            self.current_strafe = 0
        else:
            if math.fabs(strafe - self.current_strafe) < self.acceleration_limit:
                self.current_strafe = strafe
            else:
                if strafe - self.current_strafe > 0:
                    self.current_strafe = self.current_strafe + self.acceleration_limit
                else:
                    self.current_strafe = self.current_strafe - self.acceleration_limit
        # twist section
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
        # self.differential_drive.arcadeDrive(self.current_thrust, self.current_twist, True)
        # TODO - fix this for mechanum x and y
        self.mechanum_drive.driveCartesian(xSpeed=self.current_thrust, ySpeed=self.current_strafe,
                                           zRotation=self.current_twist)

    def tank_drive(self, left, right):
        '''Not sure why we would ever need this, but it's here if we do'''
        pass
        # self.differential_drive.tankDrive(left, right)

    def get_position(self):
        ''':returns: The encoder position of one of the Neos'''
        return self.sparkneo_encoder_1.getPosition()

    def set_velocity(self, velocity):
        multipliers = [1.0, 1.0, -1.0, -1.0]
        for multiplier, controller in zip(multipliers, self.pid_controllers):
            controller.setReference(multiplier * velocity, rev.ControlType.kVelocity, 1)

    def goToSetPoint(self, set_point):
        self.reset()
        multipliers = [1.0, 1.0, -1.0, -1.0]
        for multiplier, controller in zip(multipliers, self.pid_controllers):
            # controller.setReference(multiplier * set_point, rev.ControlType.kPosition)
            controller.setReference(multiplier * set_point, rev.ControlType.kSmartMotion, pidSlot=1)

    def reset(self):
        if self.robot.isReal():
            for ix, encoder in enumerate(self.encoders):
                can_error = encoder.setPosition(0)
                self.error_dict.update({'ResetPos_' + str(ix): can_error})
                if can_error != rev.CANError.kOk:
                    print(f"Warning: drivetrain reset issue with {encoder} returning {can_error}")
        self.x = 0
        self.y = 0
        # wpilib.Timer.delay(0.02)

    def configure_controllers(self, pid_only=False):
        '''Set the PIDs, etc for the controllers, slot 0 is position and slot 1 is velocity'''
        if not pid_only:
            for i, controller in enumerate(self.controllers):
                # error_dict.append(controller.restoreFactoryDefaults())
                self.error_dict.update({'Idle_'+str(i):controller.setIdleMode(rev.IdleMode.kBrake)})
                self.error_dict.update({'CurLimit_'+str(i):controller.setSmartCurrentLimit(self.current_limit)})
                self.error_dict.update({'VoltComp_'+str(i):controller.enableVoltageCompensation(12)})


        for i, controller in enumerate(self.pid_controllers):
            self.error_dict.update({'kP0_'+str(i):controller.setP(self.PID_dict_pos['kP'], 0)})
            self.error_dict.update({'kP1_'+str(i):controller.setP(self.PID_dict_vel['kP'], 1)})
            self.error_dict.update({'kI0_'+str(i):controller.setI(self.PID_dict_pos['kI'], 0)})
            self.error_dict.update({'kI1_'+str(i):controller.setI(self.PID_dict_vel['kI'], 1)})
            self.error_dict.update({'kD0_'+str(i):controller.setD(self.PID_dict_pos['kD'], 0)})
            self.error_dict.update({'kD_1'+str(i):controller.setD(self.PID_dict_vel['kD'], 1)})
            self.error_dict.update({'kFF_0'+str(i):controller.setFF(self.PID_dict_pos['kFF'], 0)})
            self.error_dict.update({'kFF_1'+str(i):controller.setFF(self.PID_dict_vel['kFF'], 1)})
            self.error_dict.update({'kIZ_0'+str(i):controller.setIZone(self.PID_dict_pos['kIz'], 0)})
            self.error_dict.update({'kIZ_1'+str(i):controller.setIZone(self.PID_dict_vel['kIz'], 1)})
            self.error_dict.update({'MinMax0_'+str(i):controller.setOutputRange(self.PID_dict_pos['kMinOutput'], self.PID_dict_pos['kMaxOutput'], 0)})
            self.error_dict.update({'MinMax0_'+str(i):controller.setOutputRange(self.PID_dict_vel['kMinOutput'], self.PID_dict_vel['kMaxOutput'], 1)})
            self.error_dict.update({'Accel0_'+str(i):controller.setSmartMotionMaxVelocity(self.maxvel, 0)})
            self.error_dict.update({'Accel0_'+str(i):controller.setSmartMotionMaxVelocity(self.maxvel, 1)})
            self.error_dict.update({'Vel0_'+str(i):controller.setSmartMotionMaxAccel(self.maxacc, 0)})
            self.error_dict.update({'Vel1_'+str(i):controller.setSmartMotionMaxAccel(self.maxacc, 1)})


        # if 1 in error_dict or 2 in error_dict:
        #    print(f'Issue in configuring controllers: {error_dict}')
        # else:
        #print(f'Results of configuring controllers: {self.error_dict}')
        print('\n*Sparkmax setting*     *Response*')
        for key in sorted(self.error_dict.keys()):
            print(f'     {key:15} \t {self.error_dict[key]}')
        burn_flash = False
        if burn_flash:
            for i, controller in enumerate(self.controllers):
                can_error = controller.burnFlash()
                print(f'Burn flash on controller {i}: {can_error}')


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
        print(
            f"Pos: kP: {self.PID_dict_pos['kP']}  kI: {self.PID_dict_pos['kI']}  kD: {self.PID_dict_pos['kD']}  kIz: {self.PID_dict_pos['kIz']}  kFF: {self.PID_dict_pos['kFF']}")
        print(
            f"Vel: kP: {self.PID_dict_vel['kP']}  kI: {self.PID_dict_vel['kI']}  kD: {self.PID_dict_vel['kD']}  kIz: {self.PID_dict_vel['kIz']} kFF: {self.PID_dict_vel['kFF']}")

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
            SmartDashboard.putNumber("Position Enc2", round(self.sparkneo_encoder_2.getPosition(), 2))
            SmartDashboard.putNumber("Position Enc3", round(self.sparkneo_encoder_3.getPosition(), 2))
            SmartDashboard.putNumber("Position Enc4", round(self.sparkneo_encoder_4.getPosition(), 2))
            SmartDashboard.putNumber("Velocity Enc1", round(self.sparkneo_encoder_1.getVelocity(), 2))
            SmartDashboard.putNumber("Velocity Enc2", round(self.sparkneo_encoder_2.getVelocity(), 2))
            SmartDashboard.putNumber("Velocity Enc3", round(self.sparkneo_encoder_3.getVelocity(), 2))
            SmartDashboard.putNumber("Velocity Enc4", round(self.sparkneo_encoder_4.getVelocity(), 2))
            SmartDashboard.putNumber("Current M1", round(self.spark_neo_left_front.getOutputCurrent(), 2))
            SmartDashboard.putNumber("Current M3", round(self.spark_neo_right_front.getOutputCurrent(), 2))
            SmartDashboard.putBoolean('AccLimit', self.is_limited)

        if self.counter % 1000 == 0:
            self.display_PIDs()
            SmartDashboard.putString("alert",
                                     f"Position: ({round(self.x, 1)},{round(self.y, 1)})  Time: {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)}")
            #SmartDashboard.putString("Controller1 Idle", str(self.spark_neo_left_front.getIdleMode()))
            #SmartDashboard.putNumber("Enc1 Conversion", self.sparkneo_encoder_1.getPositionConversionFactor())
