from wpilib.command import Command
from wpilib import Timer

class ActuateGate(Command):
    """
    This command opens and closed the piston
    """

    def __init__(self, robot, direction=None, button=None):
        Command.__init__(self, name='ActuateGate')
        self.robot = robot
        self.direction = direction
        self.button = button
        self.max_power = 0.3
        self.kp = self.max_power / 15. # max power when distance from target = 15
        self.ki = self.max_power / 10. # max power when integral error is 10 * 1 s
        self.kd = self.max_power / 100. # max power when rate of change is 100 * 1/s
        self.timeout = 1

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        self.gate_prev_err = 0.
        self.gate_err_sum = 0.
        if self.direction == "open":
#            self.robot.ball_handler.open_gate()
            self.gate_targ = 70.
        elif self.direction == "close":
            if not self.robot.ball_handler.gate_encoder_initialized:  # first time button is pushed
                self.robot.ball_handler.close_gate() # drive against whatever stops it
            self.gate_targ = 0.
        else:
            print("Something happened that I didn't understand in Ball Gate")

        print("\n" + f"** Started {self.getName()} with input {self.direction} at {self.start_time} s **", flush=True)

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        if not self.robot.ball_handler.gate_encoder_initialized:
            if self.direction == "close":
                button_pressed = self.button_get()
                if not button_pressed:
                    self.robot.ball_handler.gate_encoder.reset()
                    self.robot.ball_handler.gate_encoder_initialized = True
            else: return # do nothing to open if not initialized

        self.gate_pos = self.robot.ball_handler.gate_pos()
        self.gate_err = self.gate_targ - self.gate_pos

        self.gate_power = self.kp * self.gate_err + self.ki * self.gate_err_sum + self.kd * (
                    self.gate_err - self.gate_prev_err) / 0.02
        self.prev_error = self.error
        self.gate_err_sum += self.gate_err * 0.02
        if self.gate_power > 0:
            self.gate_power = min(self.max_power, self.gate_power)
        else:
            self.gate_power = max(-self.max_power, self.gate_power)
        self.robot.ball_handler.gate_power(self.gate_power)
        pass

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        if (self.direction == "open") and not self.robot.ball_handler.gate_encoder_initialized:
            return True # exit if open button is pushed but encoder is not initialized
        current_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        return current_time - self.start_time > self.timeout

    def end(self):
        """Called once after isFinished returns true"""
        #print("\n" + f"** Ended {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
        self.robot.ball_handler.hopper_spark.set(0)

    def interrupted(self):
        """Called w

        hen another command which requires one or more of the same subsystems is scheduled to run."""
        #print("\n" + f"** Interrupted {self.name} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
