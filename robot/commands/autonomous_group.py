# just leaving this from pacgoat as an example for now
from wpilib.command import CommandGroup
from commands.autonomous_drive import AutonomousDrive
from commands.autonomous_rotate import AutonomousRotate
from commands.intake import Intake

class AutonomousGroup(CommandGroup):
    """
    Testing an automatic intake of a ball
    """

    def __init__(self, robot, timeout=None):
        CommandGroup.__init__(self, name='AutonomousGroup')
        self.addSequential(AutonomousRotate(robot, setpoint=None, timeout=3, source='camera'))
        self.addParallel(Intake(robot, power=0.5, end_power=0.5))
        self.addSequential(AutonomousDrive(robot, setpoint=None, control_type='position', timeout=6, source='camera'))


