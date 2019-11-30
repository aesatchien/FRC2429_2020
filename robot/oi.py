# Attempt to convert 2019 Spartan Java to Python - 11/22/2019 CJH
import wpilib
from wpilib import SmartDashboard
from wpilib.buttons import JoystickButton
# Spartan-specific commands - must import if you plan to use
from triggers.axis_button import AxisButton
from triggers.pov_button import POVButton
from commands.dpad_drive import DpadDrive
from commands.update_PIDs import UpdatePIDs
from commands.autonomous_drive import AutonomousDrive
from commands.autonomous_rotate import AutonomousRotate

class OI(object):
    """
    The operator interface of the robot.  Note we use competition_mode to determine if we will
    initialize a second joystick
    """

    def __init__(self, robot):
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

        self.buttonA.whenPressed(UpdatePIDs(robot,1.5, from_dashboard=False))
        self.buttonB.whenPressed(UpdatePIDs(robot,0.66, from_dashboard=False))
        self.buttonX.whenPressed(AutonomousDrive(robot, setpoint=40, control_type='position'))
        self.buttonY.whenPressed(AutonomousRotate(robot, setpoint=45))
        self.buttonLB.whenPressed(AutonomousDrive(robot, setpoint=2000, control_type='velocity', button=self.buttonLB))
        self.buttonRB.whenPressed(AutonomousDrive(robot, setpoint=500, control_type='velocity', button=self.buttonRB))
        # self.buttonBack.whenPressed
        # self.buttonStart.whenPressed
        # self.axisButtonLT.whenPressed
        # self.axisButtonRT.whenPressed
        self.povButtonUp.whenPressed(DpadDrive(robot,"up",self.povButtonUp))
        self.povButtonDown.whenPressed(DpadDrive(robot, "down", self.povButtonDown))
        self.povButtonRight.whenPressed(DpadDrive(robot, "right", self.povButtonRight))
        self.povButtonLeft.whenPressed(DpadDrive(robot, "left", self.povButtonLeft))

        # add/change bindings if we are using more than one joystick
        self.competition_mode = False
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

        # SmartDashboard Buttons - test some autonomous commands here
        SmartDashboard.putData("Drive Forward", AutonomousDrive(robot, setpoint=40, control_type='position'))
        SmartDashboard.putData("Update PIDs", (UpdatePIDs(robot, factor=1, from_dashboard=True)))
        SmartDashboard.putData("Rotate X", AutonomousRotate(robot, 45))

    def getJoystick(self):
        return self.stick
