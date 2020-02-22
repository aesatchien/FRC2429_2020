#!/usr/bin/env python3
# Attempt to convert 2019 Spartan Java to Python - 11/22/2019 CJH
# Then attempt to convert that to the 2020 wpilib API - 1/27/2020 CJH
import wpilib
from wpilib import Timer
from wpilib.command import Scheduler
from commandbased import CommandBasedRobot
# 2429-specific imports - need to import every subsystem you instantiate
from oi import OI
from subsystems.drivetrain import DriveTrain
from subsystems.navigation import Navigation
from subsystems.pneumatics import Pneumatics
from subsystems.peripherals import Peripherals
from subsystems.ball_handler import Ball_Handler
from subsystems.climber import Climber
from wpilib import SendableChooser
from commands.autonomous_group import AutonomousGroup
from commands.autonomous_routes import AutonomousRoutes


class Robot(CommandBasedRobot):

    """This is the main class for running the PacGoat code."""

    def robotInit(self):
        super().__init__()
        #CommandBasedRobot.__init__()
        """
        This function is run when the robot is first started up and should be
        used for any initialization code.
        """
        # make this true to ignore joystick errors
        self.debug = False
        self.enabled_time = 0
        # Initialize the subsystems
        self.drivetrain = DriveTrain(self)
        self.navigation = Navigation(self)
        #self.pneumatics = Pneumatics(self)
        self.peripherals = Peripherals(self)
        self.ball_handler = Ball_Handler(self)
        self.climber = Climber(self)
        self.position_chooser = SendableChooser()
        self.scoring_chooser = SendableChooser()
        self.backoff_chooser = SendableChooser()
        #self.distance = Distance(self)
        #wpilib.SmartDashboard.putData(self.drivetrain)
        #wpilib.SmartDashboard.putData(self.pneumatics)
        #wpilib.SmartDashboard.putData(self.peripherals)

        # This MUST be here. If the OI creates Commands (which it very likely
        # will), constructing it during the construction of CommandBase (from
        # which commands extend), subsystems are not guaranteed to be
        # yet. Thus, their requires() statements may grab null pointers. Bad
        # news. Don't move it.
        self.oi = OI(self)

        #wpilib.SmartDashboard.putData(Scheduler.getInstance())
        # instantiate the command used for the autonomous period
        self.autonomousCommand = None
        """
        self.auto_chooser.addOption("Option 1", AutonomousGroup(self))
        self.auto_chooser.addOption("Option 2", AutonomousGroup(self))
        """

        for position in AutonomousRoutes.positions:
            self.position_chooser.addOption(position, position)
        for scoring_route in AutonomousRoutes.scoring_routes:
            self.scoring_chooser.addOption(scoring_route, scoring_route)
        for backoff_route in AutonomousRoutes.backoff_routes:
            self.backoff_chooser.addOption(backoff_route, backoff_route)

        wpilib.SmartDashboard.putData('Autonomous Starting Position', self.position_chooser)
        wpilib.SmartDashboard.putData('Autonomous Scoring Route', self.scoring_chooser)
        wpilib.SmartDashboard.putData('Autonomous Backoff Route', self.backoff_chooser)

    def autonomousInit(self):
        self.reset()
        self.enabled_time = Timer.getFPGATimestamp()

        self.autonomousCommand = AutonomousRoutes(self)

        self.autonomousCommand.start()


    # self.autonomousCommand = self.autoChooser.getSelected()
    # self.autonomousCommand.start()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous"""
        Scheduler.getInstance().run()
        self.log()

    def teleopInit(self):
        """This function is called at the beginning of operator control."""
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        self.reset()
        self.enabled_time = Timer.getFPGATimestamp()
        if self.autonomousCommand is not None:
            self.autonomousCommand.cancel()
        print(f" robot.isSimulation() is {self.isSimulation()} robot.isReal() is {self.isReal()}")

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        Scheduler.getInstance().run()
        self.log()

    def testPeriodic(self):
        """This function is called periodically during test mode."""
        #wpilib.LiveWindow.run()

    def disabledInit(self):
        self.reset()
        # self.shooter.unlatch()

    def disabledPeriodic(self):
        """This function is called periodically while disabled."""
        self.log()

    def log(self):

        self.navigation.log()
        self.peripherals.log()
        if self.isReal():
            self.drivetrain.log()

    def reset(self):
        self.drivetrain.reset()

if __name__ == "__main__":
    wpilib.run(Robot)
