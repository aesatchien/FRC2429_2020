#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

import hal.simulation

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units

from wpilib import SmartDashboard
import wpilib.simulation
import wpilib.geometry as geo

class PhysicsEngine:
    """
        Simulates a motor moving something that strikes two limit switches,
        one on each end of the track. Obviously, this is not particularly
        realistic, but it's good enough to illustrate the point
    """

    def __init__(self, physics_controller: PhysicsInterface):

        self.physics_controller = physics_controller
        self.field = wpilib.simulation.Field2d()

        # Motors - ATM we have simulation motors that run off the PWMs because the CAN motors don't work in sim
        self.l_motor = hal.simulation.PWMSim(1)
        self.r_motor = hal.simulation.PWMSim(3)

        # Precompute the encoder constant. This is not necessary for us, since SparkMAX encoders return distances, not counts
        # -> encoder counts per revolution / wheel circumference
        self.kEncoder = 360 / (0.5 * 3.14159)

        # NavX (SPI interface) - no idea why the "4" is there, just using it as given
        self.navx = hal.simulation.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self.navx.getDouble("Yaw")

        # stuff left over from the example template which you can get from the vanilla physics example
        self.dio1 = hal.simulation.DIOSim(1)
        self.dio2 = hal.simulation.DIOSim(2)
        self.ain2 = hal.simulation.AnalogInSim(2)
        self.motor = hal.simulation.PWMSim(4)

        # set up two simulated encoders to see how they effect the robot
        self.r_encoder = hal.simulation.EncoderSim(0)
        self.l_encoder = hal.simulation.EncoderSim(1)

        # Gyro
        self.gyro = hal.simulation.AnalogGyroSim(1)

        self.position = 0  # for the limit switch demonstration
        self.l_distance = 0  # encoder distances
        self.r_distance = 0
        self.x = 0  # x y rot position data calculated from the pose
        self.y = 0
        self.rotation = 0

        self.count = 0  # counter for updating the dashboard at a reasonable rate

        # strings to send our simulation data out and grab in Jupyter notebook
        odometry_string = f"X: {self.x:+03.2f}  Y: {self.y:+03.2f}  Rot: {self.rotation:+03.2f}"
        SmartDashboard.putString("Odometry", odometry_string)
        odometry_string = f"{self.x:+9.2f} {self.y:+9.2f} {self.rotation:+9.2f} {self.l_distance:+9.2f} {self.r_distance:+9.1f} {self.navx_yaw.get():+9.2f}"
        self.odometry_list = 256*[odometry_string]


        # Change these parameters to fit your robot!
        bumper_width = 3.25 * units.inch

        # fmt: off
        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_CIM,           # motor configuration
            110 * units.lbs,                    # robot mass
            10.71,                              # drivetrain gear ratio
            2,                                  # motors per side
            22 * units.inch,                    # robot wheelbase
            23 * units.inch + bumper_width * 2, # robot width
            32 * units.inch + bumper_width * 2, # robot length
            6 * units.inch,                     # wheel diameter
        )
        # fmt: on

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
            Called when the simulation parameters for the program need to be
            updated.
            
            :param now: The current time as a float
            :param tm_diff: The amount of time that has passed since the last
                            time that this function was called
        """

        # Simulate the drivetrain
        l_motor = self.l_motor.getSpeed()
        r_motor = self.r_motor.getSpeed()

        # generate the transform to the robot's pose
        transform = self.drivetrain.calculate(l_motor, r_motor, tm_diff)
        # update the pose
        pose = self.physics_controller.move_robot(transform)

        # cjh keep us on the field - still need to figure out the corner case and mecanum case (may need to flip y?)
        x_limit = 15.97
        y_limit = 8.21
        if pose.translation().x < 0 or pose.translation().x > x_limit:
            curr_x = transform.translation().x
            curr_y = transform.translation().y
            new_transform = geo.Transform2d(geo.Translation2d(-curr_x, curr_y), transform.rotation())
            self.physics_controller.move_robot(new_transform)

        # apparently y is always 0 in the transform unless you are in mecanum - it goes off the rotation
        if pose.translation().y < 0 or pose.translation().y > y_limit:
            curr_x = transform.translation().x
            curr_y = transform.translation().y
            new_transform = geo.Transform2d(geo.Translation2d(-curr_x, curr_y), transform.rotation())
            #print('Out of bounds on y ...', transform, new_transform)
            self.physics_controller.move_robot(new_transform)

        # Update encoders - just as a lesson for now
        self.l_distance += l_motor * tm_diff
        self.r_distance += r_motor * tm_diff

        l_dist = int(self.l_distance * self.kEncoder)
        r_dist = int(self.r_distance * self.kEncoder)
        self.l_encoder.setCount(l_dist)
        #self.l_encoder.setRate(self.drivetrain.getLeftVelocityMetersPerSecond())
        self.r_encoder.setCount(r_dist)
        #self.r_encoder.setRate(self.drivetrain.getRightVelocityMetersPerSecond())

        # Update the gyro simulation
        # -> FRC gyros are positive clockwise, but the returned pose is positive
        #    counter-clockwise
        self.gyro.setAngle(-pose.rotation().degrees())

        # Update the navx gyro simulation
        # -> FRC gyros like NavX are positive clockwise, but
        #    the returned pose is positive counter-clockwise
        self.navx_yaw.set(-pose.rotation().degrees())

        # update the dashboard.  need to figure out how to unify this.  like maybe have a global robot pose
        self.count += 1
        if self.count % 10 == 0:
            self.x = pose.translation().x
            self.y = pose.translation().y
            self.rotation = pose.rotation().degrees()
            odometry_string = f"X: {self.x:+03.2f}  Y: {self.y:+03.2f}  Rot: {self.rotation:+03.2f}"
            SmartDashboard.putString("Odometry", odometry_string)

            # make an odometry string that gives the full history - can recreate if necessary
            odometry_string = f"{self.x:+9.2f} {self.y:+9.2f} {self.rotation:+9.2f} {self.l_distance:+9.2f} {self.r_distance:+9.1f} {self.navx_yaw.get():+9.2f}"
            self.odometry_list.pop(0)
            self.odometry_list.append(odometry_string)
            SmartDashboard.putStringArray("Odometry_List", self.odometry_list)

# ---------------- CURRENTLY UNUSED BUT USEFUL FOR LEARNING ---------------------------
        # update 'position' (use tm_diff so the rate is constant) - this is for simulating an elevator, arm etc w/ limit switches
        self.position += self.motor.getSpeed() * tm_diff * 3

        # update limit switches based on position
        if self.position <= 0:
            switch1 = True
            switch2 = False

        elif self.position > 10:
            switch1 = False
            switch2 = True

        else:
            switch1 = False
            switch2 = False

        # set values here - nice way to emulate the robot's state
        self.dio1.setValue(switch1)
        self.dio2.setValue(switch2)
        self.ain2.setVoltage(self.position)