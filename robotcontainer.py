#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd
from commands2 import cmd
from wpilib import SmartDashboard, XboxController
from autonomous.autons.forward import create_forward_auto
from subsystems.climb.command import Climb
from subsystems.elevator.command import Elevator, ElevatorMode, ElevatorPositions
from generated.tuner_constants import TunerConstants
from subsystems.elevator.coral.wheels import Wheels
from telemetry import Telemetry
from phoenix6 import swerve
from phoenix6.hardware import TalonFX
from wpimath.units import rotationsToRadians
from subsystems.elevator.coral.rotate import RotateCommand

from utils.constants import (ELEVATOR_LEADING_MOTOR_ID, ELEVATOR_FOLLOWING_MOTOR_ID,
    CLIMB_MOTOR_ID, ROTATE_INTAKE_MOTOR_ID, TOP_WHEELS_MOTOR_ID, single_rotation_inches)
class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # Initialize drivetrain configuration
        self._configure_drivetrain()
        
        # Initialize controllers
        self._joystick = commands2.button.CommandXboxController(0)
        self._functional_controller = commands2.button.CommandXboxController(1)
        
        # Initialize subsystems
        self.drivetrain = TunerConstants.create_drivetrain()
        self.leading_motor = TalonFX(ELEVATOR_LEADING_MOTOR_ID)
        self.following_motor = TalonFX(ELEVATOR_FOLLOWING_MOTOR_ID)
        self.leading_motor.set_position(0)
        self.following_motor.set_position(0)
        self.elevator = Elevator(self.leading_motor, self.following_motor)
        self.climb = Climb(TalonFX(CLIMB_MOTOR_ID))
        self.wheels = Wheels(TalonFX(TOP_WHEELS_MOTOR_ID))
        self.rotate_command = RotateCommand(TalonFX(ROTATE_INTAKE_MOTOR_ID))

        SmartDashboard.putNumber("Elevator/Leading Motor Position", self.leading_motor.get_position().value)
        SmartDashboard.putNumber("Elevator/Following Motor Position", self.following_motor.get_position().value)
        SmartDashboard.putNumber("Elevator/Leading Motor Position(in)", self.leading_motor.get_position().value * single_rotation_inches)
        SmartDashboard.putNumber("Elevator/Following Motor Position(in)", self.following_motor.get_position().value * single_rotation_inches)
        
        # Configure all button bindings
        self.configureButtonBindings()

    def _configure_drivetrain(self) -> None:
        self._max_speed = TunerConstants.speed_at_12_volts * 0.5
        self._max_angular_rate = rotationsToRadians(0.75)
        
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(self._max_angular_rate * 0.1)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._logger = Telemetry(self._max_speed)

    def configureButtonBindings(self) -> None:
        self._configure_drivetrain_controls()
        self._configure_elevator_controls()
        self._configure_wheels_controls()
        self._configure_climb_controls()
        self._configure_rotate_controls()

    def _configure_drivetrain_controls(self) -> None:
        # Configure default drive command
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
        self._joystick.b().onFalse(cmd.runOnce(lambda: setattr(self, '_max_speed', TunerConstants.speed_at_12_volts)))
        
        # Reset field-centric heading
        self._joystick.leftBumper().onTrue(
        self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
    )
    
        # Register telemetry
        self.drivetrain.register_telemetry(lambda state: self._logger.telemeterize(state))

    def _configure_elevator_controls(self) -> None:
        # Manual elevator controls
        self._functional_controller.y().whileTrue(cmd.runEnd(
            lambda: self.elevator.move_motor(20),
            lambda: self.elevator.brake()
        ))
        
        self._functional_controller.a().whileTrue(cmd.runEnd(
            lambda: self.elevator.move_motor(-20),
            lambda: self.elevator.brake()
        ))
        # self.elevator.setDefaultCommand(
        #     cmd.run(
        #         lambda: self.elevator.move_test(
        #             int(self._functional_controller.getRawAxis(XboxController.Axis.kLeftY))
        #         ),
        #         self.elevator
        #     )
        # )
        
        # Position-based elevator controls
        self._functional_controller.pov(0).whileTrue(self.elevator.move(ElevatorPositions.Level3, ElevatorMode.POSITION)) #top
        self._functional_controller.pov(90).whileTrue(self.elevator.move(ElevatorPositions.Level4, ElevatorMode.POSITION)) #right
        self._functional_controller.pov(180).whileTrue(self.elevator.move(ElevatorPositions.Level1, ElevatorMode.POSITION)) #bottom
        self._functional_controller.pov(270).whileTrue(self.elevator.move(ElevatorPositions.Level2, ElevatorMode.POSITION)) #left

    def _configure_wheels_controls(self) -> None:
        # Intake/outtake controls
        self._functional_controller.rightTrigger().whileTrue(cmd.parallel(
            self.wheels.run(50),
            cmd.run(lambda: None)
        ))
        self._functional_controller.leftTrigger().whileTrue(cmd.parallel(
            self.wheels.run(-50),
            cmd.run(lambda: None)
        ))
    def _configure_climb_controls(self) -> None:
        # Climb controls
        self._functional_controller.rightBumper().whileTrue(self.climb.run(25))
        self._functional_controller.leftBumper().whileTrue(self.climb.run(-25))
        
    def _configure_rotate_controls(self) -> None:
        # Rotate intake control
        self.rotate_command.setDefaultCommand(
            cmd.run(
                lambda: self.rotate_command.rotate(
                    int(self._functional_controller.getRawAxis(XboxController.Axis.kRightY))
                ),
                self.rotate_command
            )
        )
    def getAutonomousCommand(self) -> commands2.Command:
        return create_forward_auto(self.drivetrain)
        #return commands2.cmd.print_("No autonomous command configured")
