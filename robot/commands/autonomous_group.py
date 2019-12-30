# just leaving this from pacgoat as an example for now
from wpilib.command import CommandGroup

from commands.autonomous_drive import AutonomousDrive
from commands.track_telemetry import TrackTelemetry

class AutonomousGroup(CommandGroup):
    """
    Trying to figure out how to track telemetry separately (as apposed to all the time)
    """

    def __init__(self, robot, setpoint=None, control_type='position', button='None', timeout=None, from_dashboard=True):
        super().__init__()
        self.addParallel(TrackTelemetry(robot, timeout=timeout))
        self.addParallel(AutonomousDrive(robot, setpoint=setpoint, control_type=control_type, button=button, timeout=timeout, from_dashboard=from_dashboard))


