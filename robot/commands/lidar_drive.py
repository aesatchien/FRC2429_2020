from wpilib.command import Command
'''
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
'''