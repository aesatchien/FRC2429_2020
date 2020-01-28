#from wpilib.buttons import Button
from wpilib.command import Button
from wpilib import Joystick

class POVButton(Button):
    """
    A custom button that is triggered when the dpad is used.
    povButtonUp =   POVButton(stick, 0)
    povButtonDown =   POVButton(stick, 180)
    povButtonRight =   POVButton(stick, 90)
    povButtonLeft =   POVButton(stick, 270)
    I think it returns -1 if it is not being used.
    """

    def __init__(self, joystick, angle):
        self.joystick = joystick
        self.angle = angle
        self.threshold = 0.01

    def get(self):
        return self.joystick.getPOV(0) == self.angle
