from wpilib.command import CommandGroup
from .autonomous_drive import AutonomousDrive
from .autonomous_rotate import AutonomousRotate

class PseudoStrafe(CommandGroup):
    def __init__(self, robot, dist_right):
        CommandGroup.__init__(self, name='PseudoStrafe')

        self.robot = robot
        self.dist_right = dist_right

        self.addSequential(AutonomousRotate(robot, setpoint=90))
        self.addSequential(AutonomousDrive(robot, setpoint=dist_right))
        self.addSequential(AutonomousRotate(robot, setpoint=-90))
