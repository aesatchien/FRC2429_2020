from wpilib.buttons import Button
from wpilib import Joystick

class AxisButton(Button):
    """
    A custom button that is used when pretendng an axis button is digital.
    """

    def __init__(self, joystick, axis):
        self.joystick = joystick
        self.axis = axis
        self.threshold = 0.03

    def get(self):
        return self.joystick.getRawAxis(self.axis) > self.threshold

