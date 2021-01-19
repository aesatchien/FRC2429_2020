from wpilib.command import Command
from wpilib import Timer, SmartDashboard


class LidarDrive(Command):
    def __init__(self, robot, distance, velocity=0.1, timeout=None):
        Command.__init__(self, name='LidarDrive')

        self.requires(robot.drivetrain)
        self.requires(robot.peripherals)

        self.distance = distance
        self.velocity = velocity
        self.timeout = timeout

        self.setTimeout(self.timeout)

    def initialize(self):
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)

        self.setTimeout(self.timeSinceInitialized() + self.timeout)

        print("\n" + f"** Started {self.getName()} with distance {self.distance}  and velocity {self.velocity} at {self.start_time} s **")

        self.robot.drivetrain.set_velocity(self.velocity)

    def isFinished(self):
        return self.robot.peripherals.lidar_distance() <= self.distance or self.isTimedOut()

    def end(self):
        self.robot.drivetrain.set_velocity(0)

        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print(
            "\n" + f"** Ended {self.getName()} at {end_time} s with a duration of {round(end_time - self.start_time, 1)} s **")
        SmartDashboard.putString("alert",
                                 f"** Ended {self.getName()} at {end_time} s with a duration of {round(end_time - self.start_time, 1)} s **")

    def interrupted(self):
        self.end()
