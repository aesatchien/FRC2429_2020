#
# See the notes for the other physics sample
#


from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units
from pyfrc.physics import drivetrains


class PhysicsEngine(object):
    """
       Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller):
        """
            :param physics_controller: `pyfrc.physics.core.Physics` object
                                       to communicate simulation effects to
        """

        self.physics_controller = physics_controller

        # Change these parameters to fit your robot!
        bumper_width = 3.25 * units.inch

        # fmt: off
        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_CIM,  # motor configuration
            110 * units.lbs,  # robot mass
            2,  # drivetrain gear ratio
            2,  # motors per side
            22 * units.inch,  # robot wheelbase
            28 * units.inch + bumper_width * 2,  # robot width
            30 * units.inch + bumper_width * 2,  # robot length
            8 * units.inch,  # wheel diameter
        )
        # fmt: on

    def update_sim(self, hal_data, now, tm_diff):
        """
            Called when the simulation parameters for the program need to be
            updated.

            :param now: The current time as a float
            :param tm_diff: The amount of time that has passed since the last
                            time that this function was called
        """

        # Simulate the drivetrain
        #lr_motor = hal_data["pwm"][1]["value"]
        #rr_motor = hal_data["pwm"][3]["value"]

        # Not needed because front and rear should be in sync
        # lf_motor = hal_data['pwm'][3]['value']
        # rf_motor = hal_data['pwm'][4]['value']

        #x, y, angle = self.drivetrain.get_distance(lr_motor, rr_motor, tm_diff)
        #self.physics_controller.distance_drive(x, y, angle)

        # Simulate the drivetrain
        # -> Remember, in the constructor we inverted the left motors, so
        #    invert the motor values here too!
        lf_motor = hal_data["pwm"][1]["value"]
        lr_motor = hal_data["pwm"][2]["value"]
        rf_motor = -hal_data["pwm"][3]["value"]
        rr_motor = -hal_data["pwm"][4]["value"]

        vx, vy, vw = drivetrains.mecanum_drivetrain(
            lr_motor, rr_motor, lf_motor, rf_motor
        )
        #self.physics_controller.vector_drive(vx, vy, vw, tm_diff)
        speed_factor = 4.0
        strafe_factor = 1.5
        self.physics_controller.vector_drive(strafe_factor*vx, speed_factor*vy, speed_factor*vw, tm_diff)
