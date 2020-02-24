# OI compatible with robotpy 2020
import wpilib
from wpilib import SmartDashboard
from wpilib import SendableChooser
from wpilib.command import JoystickButton
# Spartan-specific commands - must import if you plan to use
from triggers.axis_button import AxisButton
from triggers.pov_button import POVButton
from commands.dpad_drive import DpadDrive
from commands.update_PIDs import UpdatePIDs
from commands.autonomous_drive import AutonomousDrive
from commands.autonomous_rotate import AutonomousRotate
from commands.pneumatic_piston import PneumaticPiston
from commands.intake import Intake
from commands.panel_spinner import PanelSpinner
from commands.actuate_gate import ActuateGate
from commands.spin_to_color import SpinToColor
from commands.autonomous_group import AutonomousGroup
from commands.autonomous_routes import AutonomousRoutes
from commands.raise_climber import RaiseClimber

class OI(object):
    """
    The operator interface of the robot.  Note we use competition_mode to determine if we will
    initialize a second joystick
    """

    def __init__(self, robot):
        super().__init__()
        self.robot = robot

        # Set single or double joystick mode
        self.competition_mode = True

        self.initialize_joystics()
        self.assign_buttons()

        """        if not robot.debug:
            self.initialize_joystics()
            self.assign_buttons()
        else:
            self.stick = wpilib.Joystick(0)"""

        # SmartDashboard stuff
        self.send_commands_to_dashboard()

    def assign_buttons(self):
        """Assign commands to buttons here"""
        # binding button to commands
        self.axisButtonRT.whenPressed(Intake(self.robot, power=0, button=self.axisButtonRT))
        self.axisButtonLT.whenPressed(Intake(self.robot, power=0, button=self.axisButtonLT))
        self.buttonRB.whenPressed(ActuateGate(self.robot, direction='open', button=self.buttonRB))
        self.buttonLB.whenPressed(ActuateGate(self.robot, direction='close', button=self.buttonLB))
        self.buttonB.whenPressed(Intake(self.robot, power=-0.5, button=self.buttonB))
        self.buttonA.whenPressed(Intake(self.robot, power=0.5, button=self.buttonA))
        self.buttonX.whenPressed(PanelSpinner(self.robot, power=0.3, button=self.buttonX))
        # still testing climber TODO: sense tilt of bar
        self.buttonY.whenPressed(RaiseClimber(self.robot, direction='hook', power=0.7, button=self.buttonY))
        self.buttonStart.whenPressed(RaiseClimber(self.robot, direction='climb', power=0.75, button=self.buttonStart))
        self.buttonBack.whenPressed(RaiseClimber(self.robot, power=0.6, direction='right', button=self.buttonBack))
        self.povButtonUp.whenPressed(DpadDrive(self.robot, 'up', self.povButtonUp))
        self.povButtonDown.whenPressed(DpadDrive(self.robot, 'down', self.povButtonDown))
        self.povButtonRight.whenPressed(DpadDrive(self.robot, 'right', self.povButtonRight))
        self.povButtonLeft.whenPressed(DpadDrive(self.robot, 'left', self.povButtonLeft))

        #self.buttonStart.whenPressed(AutonomousDrive(robot, setpoint=250, control_type='velocity', button= self.buttonStart, source=None))
        #self.buttonBack.whenPressed(AutonomousDrive(robot, setpoint=500, control_type='velocity', button=self.buttonBack, source=None))
        #self.buttonY.whenPressed(SpinToColor(robot, target_color='blue', power=0.3))

        # co-pilot joystick to commands
        if self.competition_mode:
            self.co_axisButtonRT.whenPressed(Intake(self.robot, power=0, button=self.co_axisButtonRT))
            self.co_axisButtonLT.whenPressed(Intake(self.robot, power=0, button=self.co_axisButtonLT))
            self.co_buttonB.whenPressed(Intake(self.robot, power=-0.5, button=self.co_buttonB))
            self.co_buttonA.whenPressed(Intake(self.robot, power=0.5, button=self.co_buttonA))
            self.co_buttonRB.whenPressed(ActuateGate(self.robot, direction='close', button=self.co_buttonRB))
            self.co_buttonLB.whenPressed(ActuateGate(self.robot, direction='open', button=self.co_buttonLB))
            self.co_buttonX.whenPressed(PanelSpinner(self.robot, power=0.4, button=self.co_buttonX))
            #self.co_buttonY.whenPressed(RaiseClimber(self.robot, direction='climb', power=0.75, button=self.co_buttonY))
            self.co_povButtonUp.whenPressed(RaiseClimber(self.robot, direction='hook', power=0.7, button=self.co_povButtonUp))
            self.co_povButtonDown.whenPressed(RaiseClimber(self.robot, direction='hook', power=0.2, button=self.co_povButtonDown))
            self.co_povButtonRight.whenPressed(RaiseClimber(self.robot, power=0.99, direction='right', button=self.co_povButtonRight))
            self.co_povButtonLeft.whenPressed(RaiseClimber(self.robot, power=-0.99, direction='left', button=self.co_povButtonLeft))

    def initialize_joystics(self):
        """
        Assign all buttons on the driver and co-pilot's gamepads
        Does not need to be edited once written
        :return:
        """
        self.stick = wpilib.Joystick(0)
        self.buttonA = JoystickButton(self.stick, 1)
        self.buttonB = JoystickButton(self.stick, 2)
        self.buttonX = JoystickButton(self.stick, 3)
        self.buttonY = JoystickButton(self.stick, 4)
        self.buttonLB = JoystickButton(self.stick, 5)
        self.buttonRB = JoystickButton(self.stick, 6)
        self.buttonBack = JoystickButton(self.stick, 7)
        self.buttonStart = JoystickButton(self.stick, 8)
        self.povButtonUp = POVButton(self.stick, 0)
        self.povButtonDown = POVButton(self.stick, 180)
        self.povButtonRight = POVButton(self.stick, 90)
        self.povButtonLeft = POVButton(self.stick, 270)
        self.axisButtonLT = AxisButton(self.stick, 2)
        self.axisButtonRT = AxisButton(self.stick, 3)

        # add/change bindings if we are using more than one joystick
        if self.competition_mode:
            self.co_stick = wpilib.Joystick(1)
            self.co_buttonA = JoystickButton(self.co_stick, 1)
            self.co_buttonB = JoystickButton(self.co_stick, 2)
            self.co_buttonX = JoystickButton(self.co_stick, 3)
            self.co_buttonY = JoystickButton(self.co_stick, 4)
            self.co_buttonLB = JoystickButton(self.co_stick, 5)
            self.co_buttonRB = JoystickButton(self.co_stick, 6)
            self.co_buttonBack = JoystickButton(self.co_stick, 7)
            self.co_buttonStart = JoystickButton(self.co_stick, 8)
            self.co_povButtonUp = POVButton(self.co_stick, 0)
            self.co_povButtonDown = POVButton(self.co_stick, 180)
            self.co_povButtonRight = POVButton(self.co_stick, 90)
            self.co_povButtonLeft = POVButton(self.co_stick, 270)
            self.co_axisButtonLT = AxisButton(self.co_stick, 2)
            self.co_axisButtonRT = AxisButton(self.co_stick, 3)

    def get_joystick(self):
        return self.stick

    def send_commands_to_dashboard(self):
        """Moving dashboard commands to a different function
        since they were broken when called due to pybind translation into python in 2020"""
        # These are no longer broken as of 2/3/2020 if we use the newest commands_v1 library.
        self.drive_fwd_command = AutonomousDrive(self.robot, setpoint=None, control_type='position', timeout=6, source='dashboard')
        self.rotate_command = AutonomousRotate(self.robot, setpoint=None, timeout=6, source='dashboard')
        self.position_pids_command = UpdatePIDs(self.robot, factor=1, from_dashboard='position')
        self.velocity_pids_command = UpdatePIDs(self.robot, factor=1, from_dashboard='velocity')
        self.autonomous_test_command = AutonomousGroup(self.robot)
        self.color_spinner_command = SpinToColor(self.robot, target_color=None, source='dash', power=0.25, thrust=-0.08)

        SmartDashboard.putData("Drive Forward", self.drive_fwd_command)
        SmartDashboard.putData("Rotate X", self.rotate_command)
        #SmartDashboard.putData("Update Pos PIDs", self.position_pids_command)
        #SmartDashboard.putData("Update Vel PIDs", self.velocity_pids_command)
        SmartDashboard.putData("Autonomous Test", self.autonomous_test_command)
        SmartDashboard.putData("Spin To", self.color_spinner_command)

        # SmartDashboard Buttons - test some autonomous commands here
        SmartDashboard.putNumber("Auto Distance", 30)
        SmartDashboard.putNumber("Auto Rotation", 10)

        self.color_chooser = SendableChooser()
        colors = ['blue', 'green', 'red', 'yellow']
        for ix, color in enumerate(colors):
            if ix == 0:
                self.color_chooser.setDefaultOption(color, color)
            else:
                self.color_chooser.addOption(color, color)
        wpilib.SmartDashboard.putData('Target Color', self.color_chooser)

    # set up the dashboard chooser for the autonomous options
        self.position_chooser = SendableChooser()
        self.scoring_chooser = SendableChooser()
        self.backoff_chooser = SendableChooser()
        for ix, position in enumerate(AutonomousRoutes.positions):
            if ix==0:
                self.position_chooser.setDefaultOption(position, position)
            else:
                self.position_chooser.addOption(position, position)

        for ix, scoring_route in enumerate(AutonomousRoutes.scoring_routes):
            if ix==0:
                self.scoring_chooser.setDefaultOption(scoring_route, scoring_route)
            else:
                self.scoring_chooser.addOption(scoring_route, scoring_route)

        for ix, backoff_route in enumerate(AutonomousRoutes.backoff_routes):
            if ix==0:
                self.backoff_chooser.setDefaultOption(backoff_route, backoff_route)
            else:
                self.backoff_chooser.addOption(backoff_route, backoff_route)

        wpilib.SmartDashboard.putData('Autonomous Starting Position', self.position_chooser)
        wpilib.SmartDashboard.putData('Autonomous Scoring Route', self.scoring_chooser)
        wpilib.SmartDashboard.putData('Autonomous Backoff Route', self.backoff_chooser)
