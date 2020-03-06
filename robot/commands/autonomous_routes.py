from wpilib.command import CommandGroup, WaitCommand
from .track_telemetry import TrackTelemetry
from .actuate_gate import ActuateGate
from .autonomous_drive import AutonomousDrive
from .autonomous_rotate import AutonomousRotate
from .pseudo_strafe import PseudoStrafe
from commands.autonomous_wait import AutonomousWait
from .intake import Intake
from wpilib import Timer
from wpilib import SmartDashboard

class AutonomousRoutes(CommandGroup):
    positions = 'middle', 'left',  'right'
    scoring_routes = 'direct score', 'move only', 'pick up balls', 'pick up and score'
    backoff_routes = 'none', 'shield generator port side', 'shield generator trench side', 'trench'

    # more convenient than a dict
    class dists:
        horizontal_panel_goal = 67
        horizontal_goal_ps = 65
        goal_ts = 100
        line_panel = 150
        move_only = 40
        left_collect_dist = 180
        l_target_tip_panel = 277
        l_start_dist = 9.5
        port_tip = 30
        left_score_rot = -14
        right_pre_intake = 68
        right_line_panel = 112
        right_post_intake = right_line_panel - right_pre_intake
        right_line_ps_rot = 43  # ps = player station
        right_line_ps = -123
        port_ps = 127

    def __init__(self, robot, timeout=None):
        CommandGroup.__init__(self, name='AutonomousRoutes')

        self.robot = robot
        self.scoring_distance = -120

        self.position = self.robot.oi.position_chooser.getSelected()
        self.route_a = self.robot.oi.scoring_chooser.getSelected()
        self.route_b = self.robot.oi.backoff_chooser.getSelected()

        #self.addParallel(TrackTelemetry(robot, timeout=timeout))

        if self.route_a == 'move only':
            self.move_only()
        elif self.route_a == 'pick up balls':
            self.pick_up_balls()
        elif self.route_a == 'direct score':
            self.direct_score()
        elif self.route_a == 'pick up and score':
            self.pick_up_and_score()
        else:
            print('invalid route name for phase A')

        if self.route_b != 'none':
            if self.route_b == 'shield generator port side':
                #self.port_side_b()
                pass
            elif self.route_b == 'shield generator trench side':
                #self.trench_side_b()
                pass
            elif self.route_b == 'trench':
                pass
                #self.trench_b()
            else:
                print('invalid route name for phase B')
        else:
            print('phase B was not executed because none was chosen')

    def drive(self, setpoint):
        self.addSequential(AutonomousDrive(self.robot, setpoint=setpoint))

    def turn(self, setpoint):
        self.addSequential(AutonomousRotate(self.robot, setpoint=setpoint))

    def direct_score(self):
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

        time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started direct scoring route at {time} s **")

        self.addSequential(AutonomousDrive(self.robot, setpoint=self.scoring_distance, manual_override=True))

        self.addSequential(ActuateGate(self.robot, direction='open', timeout=3))
        #self.addSequential(AutonomousWait(self.robot, timeout=3))  #  we have to have a wait command that feeds the drivetrain!
        self.addSequential(ActuateGate(self.robot, direction='close', timeout=2))

    def left_pick_up_and_score(self):
        '''
        self.pick_up_balls()

        self.addSequential(PseudoStrafe(self.robot, self.dists.horizontal_panel_goal))
        self.drive(-self.dists.line_panel))

        self.direct_score()
        '''

        self.drive(self.dists.l_start_dist)
        self.turn(-90)

        self.addSequential(Intake(self.robot, end_power=0.1))

        self.drive(self.dists.left_collect_dist)
        self.turn(self.dists.left_score_rot)
        self.drive(-self.dists.l_target_tip_panel)
        self.turn(-self.dists.left_score_rot)

        self.drive(-self.dists.port_tip)
        self.addSequential(Intake(self.robot, power=0, end_power=0))
        self.addSequential(ActuateGate(self.robot, direction='open', timeout=3))

    def right_pick_up_and_score(self):
        self.drive(self.dists.right_pre_intake)
        self.addSequential(Intake(self.robot, end_power=0.1))
        self.drive(self.dists.right_post_intake)
        self.drive(-self.dists.right_line_panel)
        self.turn(self.dists.right_line_ps_rot)
        self.drive(self.dists.right_line_ps)
        self.turn(90 - self.dists.right_line_ps_rot)
        self.drive(-self.dists.port_ps)
        self.turn(-90)

        self.drive(-self.dists.port_tip)
        self.addSequential(Intake(self.robot, power=0, end_power=0)) 
        self.addSequential(ActuateGate(self.robot, direction='open', timeout=3))

    def pick_up_and_score(self):
        if self.position == 'left':
            self.left_pick_up_and_score()
        elif self.position == 'right':
            self.right_pick_up_and_score()
        else:
            print('invalid position for "pick up and score"')

    def move_only(self):
        """
        PURPOSE: for when we are either unable to score or we don't intend to score

        PROCEDURE:
        Drive backward 30"
        """

        time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started move only route at {time} s **")

        self.addSequential(AutonomousDrive(self.robot, setpoint=self.dists.move_only, manual_override=True))

    def pick_up_balls(self):
        time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started pick up balls route at {time} s **")

        if self.position == 'left':
            dist_left = 0
        elif self.position == 'middle':
            dist_left = 26 * 12
        elif self.position == 'right':
            dist_left = 52 * 12

        '''
        if dist_left:
            self.addSequential(PseudoStrafe(self.robot, -dist_left)) 
        '''

        self.addSequential(Intake(self.robot, end_power=0.1))
        self.drive(self.dists.line_panel)
        self.addSequential(Intake(self.robot, power=0, end_power=0))

    def port_side_b(self):
        """
        PURPOSE: for when we want to get the balls on the side of the shield generator nearest to power ports

        PROCEDURE:
        Strafe right 65"
        Move forward 100"
        """

        time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started port-side-backoff route at {time} s **")

        '''
        self.turn(90))
        self.addSequential(AutonomousDrive(self.robot, setpoint=65))
        self.turn(-90)) 
        '''

        self.addSequential(PseudoStrafe(self.robot, self.dists.horizontal_goal_ps))
        self.drive(100)

    def trench_side_b(self):
        """
        PURPOSE: for when we want to get the balls on the side of the shield generator facing the trench

        PROCEDURE:
        Move forward 100"
        """

        time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started trench-side-backoff route at {time} s **")

        self.drive(self.dists.goal_ts)

    def trench_b(self):
        """
        PURPOSE: for when we want to get the balls in the trench area

        PROCEDURE:
        Strafe left 67"
        Set intake to spin
        Drive forward 200"
        Stop intake
        """

        time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started trench-backoff route at {time} s **")

        '''
        self.turn(-90))
        self.drive(67))
        self.turn(90)) 
        '''

        self.addSequential(PseudoStrafe(self.robot, -self.dists.horizontal_panel_goal))
        self.addSequential(Intake(self.robot, end_power=0.1))
        self.drive(self.dists.line_panel)
        self.addSequential(Intake(self.robot, power=0, end_power=0))

    def initialize(self):
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **")

    def end(self):
        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Ended {self.getName()} at {end_time} s with a duration of {round(end_time - self.start_time, 1)} s **")

    def interrupted(self):
        print("\n" + f"** Interrupted {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")

        self.end()