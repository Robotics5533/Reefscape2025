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
import wpilib
import os
import importlib
import inspect
import sys
from subsystems.climb.command import Climb
from subsystems.elevator.command import Elevator, ElevatorMode, ElevatorPositions
from generated.tuner_constants import TunerConstants
from subsystems.elevator.coral.wheels import Wheels
from telemetry import Telemetry
from phoenix6 import swerve
from phoenix6.hardware import TalonFX
from wpimath.units import rotationsToRadians
from subsystems.elevator.coral.rotate import RotateCommand
from subsystems.vision.align import LimelightAlign

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
        # Initialize autonomous chooser
        self._auto_chooser = wpilib.SendableChooser()
        
        # Load autonomous routines dynamically
        self._auto_routines = {}
        self._load_autonomous_routines()
        
        SmartDashboard.putData("Auto Chooser", self._auto_chooser)
        
        # Initialize drivetrain configuration
        self._configure_drivetrain()
        
        # Initialize controllers
        self._joystick = commands2.button.CommandXboxController(0)
        self._functional_controller = commands2.button.CommandXboxController(1)
        
        # Initialize subsystems
        self.drivetrain = TunerConstants.create_drivetrain()
        
        # Initialize elevator motors and subsystem
        self.leading_motor = TalonFX(ELEVATOR_LEADING_MOTOR_ID)
        self.following_motor = TalonFX(ELEVATOR_FOLLOWING_MOTOR_ID)
        self.leading_motor.set_position(0)
        self.following_motor.set_position(0)
        self.elevator = Elevator(self.leading_motor, self.following_motor)
        
        # Initialize other subsystems
        self.climb = Climb(TalonFX(CLIMB_MOTOR_ID))
        self.wheels = Wheels(TalonFX(TOP_WHEELS_MOTOR_ID))
        self.rotate_command = RotateCommand(TalonFX(ROTATE_INTAKE_MOTOR_ID))
        self.limelight_align = LimelightAlign(self.drivetrain)
        
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
        
        # Speed control - slow mode when B is held
        self._joystick.b().onTrue(cmd.runOnce(lambda: setattr(self, '_max_speed', TunerConstants.speed_at_12_volts * 0.25)))
        self._joystick.b().onFalse(cmd.runOnce(lambda: setattr(self, '_max_speed', TunerConstants.speed_at_12_volts * 0.5)))
        
        # AprilTag alignment when left bumper is pressed
        self._joystick.rightBumper().whileTrue(self.limelight_align.align_to_target())

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
        
        # Position-based elevator controls - tap to position instead of hold
        self._functional_controller.pov(0).onTrue(self.elevator.move(ElevatorPositions.Level3, ElevatorMode.POSITION))   # top
        self._functional_controller.pov(90).onTrue(self.elevator.move(ElevatorPositions.Level4, ElevatorMode.POSITION))  # right
        self._functional_controller.pov(180).onTrue(self.elevator.move(ElevatorPositions.Level1, ElevatorMode.POSITION)) # bottom
        self._functional_controller.pov(270).onTrue(self.elevator.move(ElevatorPositions.Level2, ElevatorMode.POSITION)) # left

    def _configure_wheels_controls(self) -> None:
        # Intake/outtake controls - simplified by removing unnecessary parallel command
        self._functional_controller.rightTrigger().whileTrue(self.wheels.run(50))
        self._functional_controller.leftTrigger().whileTrue(self.wheels.run(-50))
        
    def _configure_climb_controls(self) -> None:
        # Climb controls
        self._functional_controller.rightBumper().whileTrue(self.climb.run(25))
        self._functional_controller.leftBumper().whileTrue(self.climb.run(-25))
        
    def _configure_rotate_controls(self) -> None:
        # Rotate intake control
        self.rotate_command.setDefaultCommand(
            cmd.run(
                lambda: self.rotate_command.rotate(
                    self._functional_controller.getRightY()  # Use the raw float value instead of casting to int
                ),
                self.rotate_command
            )
        )
        
    def _load_autonomous_routines(self) -> None:
        """Dynamically load autonomous routines from the autons folder"""
        autons_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "autonomous", "autons")
        
        # Get all Python files in the autons directory
        for file in os.listdir(autons_dir):
            if file.endswith(".py") and file != "__init__.py":
                module_name = file[:-3]  # Remove .py extension
                
                try:
                    # Import the module dynamically
                    module_path = f"autonomous.autons.{module_name}"
                    module = importlib.import_module(module_path)
                    
                    # Look for create_*_auto functions in the module
                    for name, obj in inspect.getmembers(module):
                        if name.startswith("create_") and name.endswith("_auto") and inspect.isfunction(obj):
                            # Extract auto name from function name (e.g., create_forward_auto -> forward)
                            auto_name = name[7:-5]  # Remove "create_" and "_auto"
                            
                            # Store the function reference
                            self._auto_routines[auto_name] = obj
                            
                            # Add to chooser (make the first one default)
                            display_name = " ".join(word.capitalize() for word in auto_name.split("_")) + " Auto"
                            
                            if not self._auto_chooser.getSelected():
                                self._auto_chooser.setDefaultOption(display_name, auto_name)
                            else:
                                self._auto_chooser.addOption(display_name, auto_name)
                                
                            print(f"Loaded autonomous routine: {display_name}")
                            
                except Exception as e:
                    print(f"Error loading autonomous routine from {file}: {e}")
        
        # If no routines were found, add a default option
        if not self._auto_routines:
            self._auto_chooser.setDefaultOption("No Autonomous Available", "none")
            print("No autonomous routines found in the autons directory")
    
    def getAutonomousCommand(self) -> commands2.Command:
        """Get the selected autonomous command based on the chooser selection"""
        selected_auto = self._auto_chooser.getSelected()
        
        # If we have a valid routine, execute it
        if selected_auto in self._auto_routines:
            # Get the function reference
            create_auto_func = self._auto_routines[selected_auto]
            
            # Inspect the function to determine required parameters
            sig = inspect.signature(create_auto_func)
            params = {}
            
            # Add required parameters based on function signature
            for param_name, param in sig.parameters.items():
                if param_name == "drivetrain":
                    params["drivetrain"] = self.drivetrain
                elif param_name == "elevator":
                    params["elevator"] = self.elevator
                elif param_name == "wheels":
                    params["wheels"] = self.wheels
            
            # Call the function with the appropriate parameters
            return create_auto_func(**params)
        
        # Return a default command if no valid selection
        return commands2.cmd.print_("No autonomous command configured")
