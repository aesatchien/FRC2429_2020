from wpilib.command import CommandGroup
from .track_telemetry import TrackTelemetry


class AutonomousRoutes(CommandGroup):
    def __init__(self, robot, a_route, b_route, timeout=None):
        super().__init__()

        self.a_route = a_route
        self.b_route = b_route

        self.addParallel(TrackTelemetry(robot, timeout=timeout))

        if a_route == 'scoring':
            self.scoring_a()
        elif a_route == 'non-scoring':
            self.non_scoring_a()
        else:
            print('invalid route name for phase A')

        if a_route != 'non-scoring':
            if b_route == 'shield generator port side':
                self.port_side_b()
            elif b_route == 'shield generator trench side':
                self.trench_side_b()
            elif b_route == 'trench':
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

    def non_scoring_a(self):
        """
        PURPOSE: for when we are either unable to score or we don't intend to score

        PROCEDURE:
        Drive backward 30"
        """

    def port_side_b(self):
        """
        PURPOSE: for when we want to get the balls on the side of the shield generator nearest to power ports

        PROCEDURE:
        Strafe right 65"
        Move forward 100"
        """

    def trench_side_b(self):
        """
        PURPOSE: for when we want to get the balls on the side of the shield generator facing the trench

        PROCEDURE:
        Move forward 100"
        """

    def trench_b(self):
        """
        PURPOSE: for when we want to get the balls in the trench area

        PROCEDURE:
        Strafe left 67"
        Set intake to spin
        Drive forward 200"
        Stop intake
        """
