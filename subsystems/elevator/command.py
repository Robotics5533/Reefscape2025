# ===== ELEVATOR SYSTEM =====
# This file controls the robot's elevator mechanism that raises and lowers game pieces
# The elevator can move to specific heights (like an elevator in a building with preset floors)
# or be manually controlled (like pressing and holding an elevator button)

# Import necessary libraries for the elevator subsystem
from commands2 import Command, Subsystem, cmd  # WPILib command-based programming framework
from enum import Enum  # For creating enumeration types
from utils.HashMap import HashMap  # Custom HashMap implementation for position mapping
from phoenix6 import hardware, controls, signals  # CTRE Phoenix 6 motor control library
from commands2.sysid import SysIdRoutine  # System identification for tuning
from wpilib.sysid import SysIdRoutineLog  # Logging for system identification
from wpilib import RobotController, SmartDashboard  # WPILib utilities
from wpimath.controller import PIDController  # PID control for position management
from utils.constants import ELEVATOR_LEVELS, single_rotation_inches  # Robot-specific constants
from utils.math import inchesToRotations  # Conversion utility
from utils.motor_constants import percent_to_voltage, MOTOR_CONFIG, voltage_to_percent  # Motor utilities

class ElevatorPositions:
    """Constants for elevator position levels in inches"""
    # These are the preset heights the elevator can move to (measured in inches from the ground)
    # Think of these like floor buttons in a building elevator
    Level1 = 0    # Ground/bottom position
    Level2 = 6    # Low position (6 inches up)
    Level3 = 15   # Medium position (15 inches up)
    Level4 = 27.5 # High position (27.5 inches up)

class ElevatorMode(Enum):
    """Modes for elevator operation"""
    # The elevator can operate in two different ways:
    MANUAL = "manual"     # Driver directly controls speed (like pressing and holding a button)
    POSITION = "position" # Elevator automatically moves to a preset height (like an elevator going to floor 3)

class Elevator(Subsystem):
    """
    Elevator subsystem that controls vertical movement of the robot's elevator mechanism.
    
    This class manages two synchronized motors that drive the elevator, providing both
    manual control and precise position-based movement with gravity compensation.
    The elevator can move to predefined positions or be manually controlled.
    """
    def __init__(self, leading_motor: hardware.TalonFX, following_motor: hardware.TalonFX):
        """
        Initialize the Elevator subsystem with two synchronized motors.
        
        Args:
            leading_motor: Primary TalonFX motor controller for the elevator
            following_motor: Secondary TalonFX motor controller (runs in opposite direction)
        """
        super().__init__()
        # Initialize position map for preset elevator heights
        self.elevator_positions = HashMap()
        self.elevator_positions.put("LEVEL_1", ElevatorPositions.Level1)
        self.elevator_positions.put("LEVEL_2", ElevatorPositions.Level2)
        self.elevator_positions.put("LEVEL_3", ElevatorPositions.Level3)
        self.elevator_positions.put("LEVEL_4", ElevatorPositions.Level4)

        # Store motor references
        self.leading_motor = leading_motor
        self.following_motor = following_motor
        self.config = MOTOR_CONFIG["elevator"]

        # Zero both motors before any movement to establish baseline
        self.leading_motor.set_position(0)
        self.following_motor.set_position(0)

        # Configure position controller for precise movement
        self.position_controller = PIDController(
            0.5,  # P gain - For stronger position holding
            0.01,  # I gain - To eliminate steady-state error
            0     # D gain - No derivative term needed for now
        )
        self.position_controller.setIZone(0.125)  # Limit integral windup
        self.position_controller.setTolerance(0.5)  # Position tolerance in rotations
        
        # Constants for physics-based compensation
        self.kG = 0.1  # Gravity compensation factor to counteract elevator weight

    def periodic(self):
        """Update dashboard with current elevator position data"""
        leading_pos = self.leading_motor.get_position().value
        following_pos = self.following_motor.get_position().value
        
        # Update SmartDashboard with current positions
        SmartDashboard.putNumber("Elevator/Leading Motor Position", leading_pos)
        SmartDashboard.putNumber("Elevator/Following Motor Position", following_pos)
        SmartDashboard.putNumber("Elevator/Leading Motor Position(in)", leading_pos * single_rotation_inches)
        SmartDashboard.putNumber("Elevator/Following Motor Position(in)", following_pos * single_rotation_inches)

    def move_motor(self, speed_percent: float):
        """Move elevator motors at specified speed percentage
        
        Args:
            speed_percent: Speed percentage (-100 to 100)
        """
        # Limit speed to configured maximum
        speed_percent = max(-self.config["max_speed"], min(speed_percent, self.config["max_speed"]))
        voltage = percent_to_voltage(speed_percent)
        
        # Apply voltage to motors (note: following motor runs in opposite direction)
        self.leading_motor.setVoltage(voltage)
        self.following_motor.setVoltage(-voltage)

    def move(self, value: float | str, mode: ElevatorMode = ElevatorMode.MANUAL) -> Command:
        """Unified movement function that supports both manual and position-based control
        
        Args:
            value: Either speed percentage (-100 to 100) for manual mode,
                  or target position in inches/level name for position mode
            mode: Control mode (MANUAL or POSITION)
            
        Returns:
            Command to execute the requested movement
        """
        if mode == ElevatorMode.MANUAL:
            return cmd.runEnd(
                lambda: self.move_motor(float(value)),
                lambda: self.brake()
            )
        else:  # POSITION mode
            # Resolve target position from string or numeric value
            if isinstance(value, str):
                if value not in self.elevator_positions:
                    return cmd.none()
                target_position = self.elevator_positions.get(value).value
            else:
                target_position = float(value)

            # Position control execution function
            def execute():
                current_position = self.leading_motor.get_position().value
                target_rotations = inchesToRotations(target_position)
                
                # Calculate PID output
                pid_output = self.position_controller.calculate(current_position, target_rotations)
                
                # Apply physics-based compensation
                output = pid_output + self.kG
                
                # Move motor with compensated output
                self.move_motor(voltage_to_percent(output))
                
            # Check if position has been reached
            def is_finished():
                return self.position_controller.atSetpoint()
                
            # Create command sequence
            return cmd.run(execute).until(is_finished).finallyDo(lambda interrupted: self.hold_position())
            
    def brake(self) -> None:
        """Stop the motors and engage brake mode"""
        self.leading_motor.setVoltage(0)
        self.following_motor.setVoltage(0)
        self.leading_motor.setNeutralMode(signals.NeutralModeValue.BRAKE)
        self.following_motor.setNeutralMode(signals.NeutralModeValue.BRAKE)
        
    def hold_position(self) -> None:
        """Maintain current position by applying holding voltage and brake mode"""
        # Get current position
        current_position = self.leading_motor.get_position().value
        
        # Calculate holding voltage using PID and gravity compensation
        pid_output = self.position_controller.calculate(current_position, current_position)
        output = pid_output + self.kG
       
        self.move_motor(voltage_to_percent(output))

        self.leading_motor.setNeutralMode(signals.NeutralModeValue.BRAKE)
        self.following_motor.setNeutralMode(signals.NeutralModeValue.BRAKE)
