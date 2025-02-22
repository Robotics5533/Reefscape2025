#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd
from commands2 import cmd
from subsystems.climb.command import Climb
from subsystems.elevator.command import Elevator, ElevatorMode
from generated.tuner_constants import TunerConstants
from subsystems.elevator.coral.bottom_wheels import BottomWheels
from telemetry import Telemetry
from phoenix6 import swerve
from phoenix6.hardware import TalonFX
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians
from subsystems.elevator.coral.rotate import RotateCommand
from commands2.sysid import SysIdRoutine
from autonomous.forward_auto import create_forward_auto

from utils.constants import (MAX_ELEVATOR_HEIGHT, MIN_ELEVATOR_HEIGHT,
    ELEVATOR_LEADING_MOTOR_ID, ELEVATOR_FOLLOWING_MOTOR_ID,
    CLIMB_MOTOR_ID, BOTTOM_WHEELS_MOTOR_ID)
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
            TunerConstants.speed_at_12_volts * 0.25
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
        self.elevator = Elevator(TalonFX(ELEVATOR_LEADING_MOTOR_ID), TalonFX(ELEVATOR_FOLLOWING_MOTOR_ID))
        self.climb = Climb(TalonFX(CLIMB_MOTOR_ID))
        self.bottom_wheels = BottomWheels(TalonFX(BOTTOM_WHEELS_MOTOR_ID))
        
        # self.rotate_motor = TalonFX(15)  # Using CAN ID 15 for the rotate motor
        #self.rotate_command = RotateCommand(self.rotate_motor, self._functional_controller)
        # self.coral_outake = CoralOutake(wpilib.Spark(3))
        

        # self.autoChooser = AutoBuilder.buildAutoChooser()
        # wpilib.SmartDashboard.putData("Auto Chooser", self.autoChooser)
        # Configure the button bindings
        self.configureButtonBindings()
        self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        
        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: self._drive
                    .with_velocity_x(-self._joystick.getLeftY() * self._max_speed)
                    .with_velocity_y(-self._joystick.getLeftX() * self._max_speed)
                    .with_rotational_rate(-self._joystick.getRightX() * self._max_angular_rate)
            )
        )

        # Simplified button bindings for better performance
        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        
        self._joystick.b().onTrue(cmd.runOnce(lambda: setattr(self, '_max_speed', self._max_speed * 0.25)))
        self._joystick.b().onFalse(cmd.runOnce(lambda: setattr(self, '_max_speed', TunerConstants.speed_at_12_volts * 0.25)))
        

        # (self._functional_controller.y()).whileTrue(
        #     self.elevator.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        #     .until(
        #         lambda: self.elevator.leading_motor.get_position()
        #         >= inchesToRotations(MAX_ELEVATOR_HEIGHT)
        #     )
        #     .andThen(self.elevator.sys_id_dynamic(SysIdRoutine.Direction.kReverse))
        #     .until(
        #         lambda: self.elevator.leading_motor.get_position()
        #         <= inchesToRotations(MIN_ELEVATOR_HEIGHT)
        #     )
        #     .andThen(self.elevator.sys_id_quasistatic(SysIdRoutine.Direction.kForward))
        #     .until(
        #         lambda: self.elevator.leading_motor.get_position()
        #         >= inchesToRotations(MAX_ELEVATOR_HEIGHT)
        #     )
        #     .andThen(self.elevator.sys_id_quasistatic(SysIdRoutine.Direction.kReverse))
            # .until(
            #     lambda: self.elevator.leading_motor.get_position()
            #     <= inchesToRotations(MIN_ELEVATOR_HEIGHT)
            # )
        # )
        # Manual elevator control with Y/A buttons
        self._functional_controller.y().whileTrue(cmd.runEnd(
            lambda: self.elevator.move_motor(20),
            lambda: self.elevator.brake()
        ))

        self._functional_controller.a().whileTrue(cmd.runEnd(
            lambda: self.elevator.move_motor(-20), 
            lambda: self.elevator.brake()
        ))

        self._functional_controller.pov(0).whileTrue(self.elevator.move(12, ElevatorMode.POSITION))  # Top - 12 inches
        self._functional_controller.pov(90).whileTrue(self.elevator.move(14, ElevatorMode.POSITION))  # Right - 16 inches
        self._functional_controller.pov(180).whileTrue(self.elevator.move(0, ElevatorMode.POSITION))  # Bottom - 0 inches
        self._functional_controller.pov(270).whileTrue(self.elevator.move(10, ElevatorMode.POSITION))  # Left - 10 inches

        self._functional_controller.rightTrigger().whileTrue(cmd.parallel(
                self.bottom_wheels.run(-100),
                cmd.run(lambda: None)
            ))
        self._functional_controller.leftTrigger().whileTrue(cmd.parallel(
                self.bottom_wheels.run(100),
                cmd.run(lambda: None)
            ))    # Full speed forward
        # Use parallel command groups to allow climb and drive to run simultaneously
        self._functional_controller.rightBumper().whileTrue(
            self.climb.run(25)
        )  # Climb up at 25%
        
        self._functional_controller.leftBumper().whileTrue(
            self.climb.run(-25)
        )  # Climb down at 25%

        #self._functional_controller.y().whileTrue(self.elevator.move_to("LEVEL_3"))
        # reset the field-centric heading on left bumper press
        self._joystick.leftBumper().onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        return create_forward_auto(self.drivetrain)
        #return commands2.cmd.print_("No autonomous command configured")
