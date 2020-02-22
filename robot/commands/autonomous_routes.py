from wpilib.command import CommandGroup, WaitCommand
from .track_telemetry import TrackTelemetry
from .actuate_gate import ActuateGate
from .autonomous_drive import AutonomousDrive
from .autonomous_rotate import AutonomousRotate
from wpilib import Sendable
from wpilib import SmartDashboard

class AutonomousRoutes(CommandGroup):
    positions = 'left', 'middle', 'right'
    scoring_routes = 'scoring', 'non-scoring'
    backoff_routes = 'shield generator port side', 'shield generator trench side', 'trench'

    def __init__(self, robot, timeout=None):
        CommandGroup.__init__(self, name='AutonomousRoutes')

        self.robot = robot

        self.position = self.robot.position_chooser.getSelected()
        self.route_a = self.robot.scoring_chooser.getSelected()
        self.route_b = self.robot.backoff_chooser.getSelected()

        self.addParallel(TrackTelemetry(robot, timeout=timeout))

        if self.route_a == 'scoring':
            self.scoring_a()
        elif self.route_a == 'non-scoring':
            self.non_scoring_a()
        else:
            print('invalid route name for phase A')

        if self.route_a != 'non-scoring':
            if self.route_b == 'shield generator port side':
                self.port_side_b()
            elif self.route_b == 'shield generator trench side':
                self.trench_side_b()
            elif self.route_b == 'trench':
                self.trench_b()
            else:
                print('invalid route name for phase B')
        else:
            print('phase B was not executed because the non-scoring route was taken')

    def scoring_a(self):
        """
        PURPOSE: for when we want to score in autonomous. This should work for any location on the initiation line.

        PROCEDURE:
        Turn on ring light
        Use camera to detect reflective tape
        Calculate the distance and angle from wall

        Go within 150" of the wall using lidar sensor
        Strafe left/right until aligned with hole
        Go within 2" of wall using lidar sensor

        Open gate
        Wait ~5 seconds (or shake)
        Close gate
        """

        self.addSequential(AutonomousDrive(self.robot, setpoint=-305))

        self.addSequential(ActuateGate(self.robot, direction='open'))
        self.addSequential(WaitCommand(5))
        self.addSequential(ActuateGate(self.robot, direction='close'))

    def non_scoring_a(self):
        """
        PURPOSE: for when we are either unable to score or we don't intend to score

        PROCEDURE:
        Drive backward 30"
        """

        self.addSequential(AutonomousDrive(self.robot, setpoint=-30))

    def port_side_b(self):
        """
        PURPOSE: for when we want to get the balls on the side of the shield generator nearest to power ports

        PROCEDURE:
        Strafe right 65"
        Move forward 100"
        """

        self.addSequential(AutonomousRotate(self.robot, setpoint=90))
        self.addSequential(AutonomousDrive(self.robot, setpoint=65))
        self.addSequential(AutonomousRotate(self.robot, setpoint=-90))
        self.addSequential(AutonomousDrive(self.robot, setpoint=100))

    def trench_side_b(self):
        """
        PURPOSE: for when we want to get the balls on the side of the shield generator facing the trench

        PROCEDURE:
        Move forward 100"
        """

        self.addSequential(AutonomousDrive(self.robot, setpoint=100))

    def trench_b(self):
        """
        PURPOSE: for when we want to get the balls in the trench area

        PROCEDURE:
        Strafe left 67"
        Set intake to spin
        Drive forward 200"
        Stop intake
        """

        self.addSequential(AutonomousRotate(self.robot, setpoint=-90))
        self.addSequential(AutonomousDrive(self.robot, setpoint=67))
        self.addSequential(AutonomousRotate(self.robot, setpoint=90))
        self.robot.peripherals.run_intake(0.1)
        self.addSequential(AutonomousDrive(self.robot, setpoint=200))
        self.robot.peripherals.run_intake(0)
