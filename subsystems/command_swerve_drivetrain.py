# ===== SWERVE DRIVE SYSTEM =====
# This file controls the robot's main driving system, called a "swerve drive"
# A swerve drive allows the robot to move in ANY direction and rotate at the same time
# Think of it like a shopping cart where each wheel can turn independently

# Import necessary libraries - these give us the tools to control the robot
from commands2 import Command, Subsystem  # WPILib command-based programming framework
from commands2.sysid import SysIdRoutine  # System identification for tuning
import math
from phoenix6 import SignalLogger, swerve, units, utils  # CTRE Phoenix 6 libraries
from typing import Callable, overload  # Python typing utilities
from wpilib import DriverStation, Notifier, RobotController  # WPILib utilities
from wpilib.sysid import SysIdRoutineLog  # Logging for system identification
from wpimath.geometry import Rotation2d  # WPILib geometry utilities

class CommandSwerveDrivetrain(Subsystem, swerve.SwerveDrivetrain):
    """
    This is the main driving system of the robot. It controls all four wheels,
    allowing the robot to drive in any direction and rotate at the same time.
    
    The swerve drive is what makes the robot so maneuverable - each wheel can
    turn independently (like the front wheels of a car) AND drive independently.
    This gives the robot the ability to:
    - Drive forward/backward
    - Drive left/right (sideways)
    - Drive diagonally
    - Rotate in place
    - Do any combination of the above at the same time!
    
    Technical note: This class extends the Phoenix 6 SwerveDrivetrain class and implements
    Subsystem so it can easily be used in command-based projects.
    """

    # How often the simulation updates (5 milliseconds)
    _SIM_LOOP_PERIOD: units.second = 0.005

    # These define which direction is "forward" depending on which alliance we're on
    # This helps the drivers control the robot consistently regardless of starting position
    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    """When we're on the blue alliance, forward is toward the red alliance wall"""
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)
    """When we're on the red alliance, forward is toward the blue alliance wall"""
    # (This means the driver's controls stay consistent regardless of which side we start on)

    @overload
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:     Type of the drive motor
        :type drive_motor_type:      type
        :param steer_motor_type:     Type of the steer motor
        :type steer_motor_type:      type
        :param encoder_type:         Type of the azimuth encoder
        :type encoder_type:          type
        :param drivetrain_constants: Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:  swerve.SwerveDrivetrainConstants
        :param modules:              Constants for each specific module
        :type modules:               list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:            Type of the drive motor
        :type drive_motor_type:             type
        :param steer_motor_type:            Type of the steer motor
        :type steer_motor_type:             type
        :param encoder_type:                Type of the azimuth encoder
        :type encoder_type:                 type
        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:         swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        odometry_standard_deviation: tuple[float, float, float],
        vision_standard_deviation: tuple[float, float, float],
        modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:            Type of the drive motor
        :type drive_motor_type:             type
        :param steer_motor_type:            Type of the steer motor
        :type steer_motor_type:             type
        :param encoder_type:                Type of the azimuth encoder
        :type encoder_type:                 type
        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:         swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param odometry_standard_deviation: The standard deviation for odometry calculation
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians
        :type odometry_standard_deviation:  tuple[float, float, float]
        :param vision_standard_deviation:   The standard deviation for vision calculation
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians
        :type vision_standard_deviation:    tuple[float, float, float]
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        arg0=None,
        arg1=None,
        arg2=None,
        arg3=None,
    ):
        Subsystem.__init__(self)

        # config = RobotConfig.fromGUISettings()
        
        # AutoBuilder.configure(
        #     self.get_state().pose,
        #     self.reset_pose,
        #     self.get_robot_relative_speed,
        #     lambda speeds, feedforwards: self.drive_robot_relative(speeds),
        #     PPHolonomicDriveController(
        #         PIDConstants(2.0, 0.0, 0.1),
        #         PIDConstants(
        #             3.0, 0.0, 0.2
        #         ), 
        #     ),
        #     config,
        #     self.should_flip_path,
        #     self,
        # )
        

        swerve.SwerveDrivetrain.__init__(
            self,
            drive_motor_type,
            steer_motor_type,
            encoder_type,
            drivetrain_constants,
            arg0,
            arg1,
            arg2,
            arg3,
        )

        self._sim_notifier: Notifier | None = None
        self._last_sim_time: units.second = 0.0

        self._has_applied_operator_perspective = False
        """Keep track if we've ever applied the operator perspective before or not"""

        # Swerve requests to apply during SysId characterization
        self._translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self._steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self._rotation_characterization = swerve.requests.SysIdSwerveRotation()

        self._sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V to prevent brownout
                stepVoltage=4.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdTranslation_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._translation_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """SysId routine for characterizing translation. This is used to find PID gains for the drive motors."""

        self._sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._steer_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """SysId routine for characterizing steer. This is used to find PID gains for the steer motors."""

        self._sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                # This is in radians per second², but SysId only supports "volts per second"
                rampRate=math.pi / 6,
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Use default timeout (10 s)
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: (
                    # output is actually radians per second, but SysId only supports "volts"
                    self.set_control(
                        self._rotation_characterization.with_rotational_rate(output)
                    ),
                    # also log the requested output for SysId
                    SignalLogger.write_double("Rotational_Rate", output),
                ),
                lambda log: None,
                self,
            ),
        )
        """
        SysId routine for characterizing rotation.
        This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
        See the documentation of swerve.requests.SysIdSwerveRotation for info on importing the log to SysId.
        """

        self._sys_id_routine_to_apply = self._sys_id_routine_translation
        """The SysId routine to test"""

        if utils.is_simulation():
            self._start_sim_thread()

    def apply_request(
        self, request: Callable[[], swerve.requests.SwerveRequest]
    ) -> Command:
        """
        Creates a command that tells the swerve drive to move in a specific way.
        
        This is like telling your car "drive forward at 25 mph" or "turn left while moving forward".
        The request contains information about:
        - Which direction to move (forward/backward, left/right)
        - How fast to move
        - Whether to rotate and how quickly
        
        Technical details:
        :param request: Lambda returning the request to apply
        :type request: Callable[[], swerve.requests.SwerveRequest]
        :returns: Command to run
        :rtype: Command
        """
        return self.run(lambda: self.set_control(request()))

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Quasistatic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Quasistatic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Dynamic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Dynamic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.dynamic(direction)

    def periodic(self):
        # This function runs repeatedly during the match
        
        # WHAT THIS DOES: Makes sure the robot knows which direction is "forward"
        # based on which alliance (red or blue) we're on
        #
        # WHY THIS MATTERS: It ensures the driver's controls are consistent
        # regardless of which side of the field we start on. "Forward" on the
        # controller will always move the robot away from our alliance wall.
        #
        # This is like setting up a GPS in your car to match the direction you're facing
        if not self._has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color is not None:
                self.set_operator_perspective_forward(
                    self._RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == DriverStation.Alliance.kRed
                    else self._BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True

    def _start_sim_thread(self):
        def _sim_periodic():
            current_time = utils.get_current_time_seconds()
            delta_time = current_time - self._last_sim_time
            self._last_sim_time = current_time

            # use the measured time delta, get battery voltage from WPILib
            self.update_sim_state(delta_time, RobotController.getBatteryVoltage())

        self._last_sim_time = utils.get_current_time_seconds()
        self._sim_notifier = Notifier(_sim_periodic)
        self._sim_notifier.startPeriodic(self._SIM_LOOP_PERIOD)

    def should_flip_path():
        # Determines if we need to mirror our autonomous paths based on alliance color
        # If we're on the red alliance, we flip the path to match the field orientation
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def get_robot_relative_speed(self):
        # Gets how fast the robot is currently moving (from the robot's perspective)
        # This includes forward/backward speed, side-to-side speed, and rotation speed
        return self.get_state().speeds
    
    def drive_robot_relative(self, speeds):
        # Tells the robot to move at a specific speed relative to its current orientation
        # This is like saying "move forward at X speed" regardless of which way the robot is facing
        self.set_control(swerve.requests.DriveVelocity(speeds))
