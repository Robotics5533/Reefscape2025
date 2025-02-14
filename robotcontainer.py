#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder
import wpilib
from subsystems.command_coral_outake import CoralOutake
from subsystems.command_elevator import CommandElevator
from generated.tuner_constants import TunerConstants
from telemetry import Telemetry
from phoenix6 import swerve, hardware
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

from pathplannerlib.config import RobotConfig
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants

from utils.constants import MAX_ELEVATOR_HEIGHT, MIN_ELEVATOR_HEIGHT
from utils.math import inchesToRotations


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(self._max_speed)

        self._joystick = commands2.button.CommandXboxController(0)
        self._functional_controller = commands2.button.CommandXboxController(1)

        self.drivetrain = TunerConstants.create_drivetrain()
        # self.coral_outake = CoralOutake(wpilib.Spark(3))
        self.elevator = CommandElevator(hardware.TalonFX(13), hardware.TalonFX(14))

        self.autoChooser = AutoBuilder.buildAutoChooser()
        wpilib.SmartDashboard.putData("Auto Chooser", self.autoChooser)

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self._joystick.getLeftY() * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -self._joystick.getLeftX() * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self._joystick.getRightX() * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )

        (self._functional_controller.y()).whileTrue(
            self.elevator.sys_id_dynamic(SysIdRoutine.Direction.kForward)
            .until(
                lambda: self.elevator.leading_motor.get_position()
                >= inchesToRotations(MAX_ELEVATOR_HEIGHT)
            )
            .andThen(self.elevator.sys_id_dynamic(SysIdRoutine.Direction.kReverse))
            .until(
                lambda: self.elevator.leading_motor.get_position()
                <= inchesToRotations(MIN_ELEVATOR_HEIGHT)
            )
            .andThen(self.elevator.sys_id_quasistatic(SysIdRoutine.Direction.kForward))
            .until(
                lambda: self.elevator.leading_motor.get_position()
                >= inchesToRotations(MAX_ELEVATOR_HEIGHT)
            )
            .andThen(self.elevator.sys_id_quasistatic(SysIdRoutine.Direction.kReverse))
            .until(
                lambda: self.elevator.leading_motor.get_position()
                <= inchesToRotations(MIN_ELEVATOR_HEIGHT)
            )
        )

        # self._functional_controller.y().whileTrue(self.elevator.move_to("LEVEL_3"))

        # reset the field-centric heading on left bumper press
        self._joystick.leftBumper().onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        # self._functional_controller.b().onTrue(lambda: self.coral_outake.outake(50))

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        return self.autoChooser.getSelected()
