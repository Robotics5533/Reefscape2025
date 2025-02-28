# Import necessary libraries
from commands2 import Subsystem  # Base class for all subsystems in command-based programming
from phoenix6.hardware import TalonFX  # CTRE TalonFX motor controller (Falcon 500)
from phoenix6 import signals  # Signal definitions for Phoenix 6 API
from utils.motor_constants import percent_to_voltage  # Utility function to convert percentage to voltage
from wpimath.controller import PIDController  # WPILib PID controller implementation

class RotateCommand(Subsystem):
    """
    Subsystem that controls the rotation of the coral intake mechanism.
    
    This class handles the rotation of the intake mechanism with position holding,
    gravity compensation, and static friction compensation to ensure smooth and
    precise control even when the joystick is released.
    """
    def __init__(self, motor: TalonFX):
        """
        Initialize the RotateCommand subsystem.
        
        Args:
            motor: TalonFX motor controller that drives the rotation mechanism
        """
        super().__init__()
        self.motor: TalonFX = motor
        # Set motor to brake mode to hold position when not actively driven
        self.motor.setNeutralMode(signals.NeutralModeValue.BRAKE)
        
        # Constants for physics-based compensation
        self.kG = 0.4  # Gravity compensation - Counteracts the weight of the mechanism
        self.kS = 0.1  # Static friction compensation - Helps overcome initial resistance
        
        # PID controller for position control when holding or making small adjustments
        self.position_controller = PIDController(
            0.5,  # P gain - For strong position holding (proportional to position error)
            0.01,  # I gain - Eliminates steady-state error (accumulates error over time)
            0.05   # D gain - Provides damping (reduces oscillation)
        )
        self.position_controller.setTolerance(0.5)  # Position tolerance in rotations
        self.target_position = 0.0  # Initial target position in rotations
        
    def rotate(self, stick_value: float) -> None:
        """
        Rotate the mechanism based on joystick input with position holding.
        
        This method handles two modes of operation:
        1. Position holding mode: When joystick is near neutral (deadband)
        2. Manual control mode: When joystick is actively moved
        
        In both modes, gravity and static friction compensation are applied
        to ensure smooth and predictable movement.
        
        Args:
            stick_value: Joystick value (-1.0 to 1.0) from controller
        """
        # Get current position of the motor in rotations
        current_position = self.motor.get_position().value
        
        # Check if joystick is within deadband (near neutral)
        if abs(stick_value) < 0.1:
            # POSITION HOLDING MODE
            # Use PID control to maintain the last target position
            pid_output = self.position_controller.calculate(current_position, self.target_position)
            
            # Apply physics-based compensation:
            gravity_comp = self.kG  # Always apply upward force to counter gravity
            # Apply static friction compensation in the direction of intended motion
            static_comp = self.kS * (1 if pid_output > 0 else -1)  
            
            # Combine all components for final output
            output = pid_output + gravity_comp + static_comp
            
            # Apply calculated output for position holding
            self.motor.setVoltage(percent_to_voltage(output))
            return
            
        # MANUAL CONTROL MODE
        # Update target position based on stick input for smoother transitions
        # This creates a "moving target" that follows the joystick input
        self.target_position = current_position + (stick_value * 0.1)
        
        # Calculate motor speed with compensation
        motor_speed = 20 * stick_value  # Base speed proportional to joystick position
        gravity_comp = self.kG  # Always apply upward force to counter gravity
        # Apply static friction compensation in the direction of intended motion
        static_comp = self.kS * (1 if motor_speed > 0 else -1)  
        compensated_speed = motor_speed + gravity_comp + static_comp
        
        # Switch to coast mode for smoother manual control
        self.motor.setNeutralMode(signals.NeutralModeValue.COAST)
        # Apply the compensated speed to the motor
        self.motor.setVoltage(percent_to_voltage(compensated_speed))
        
    def brake(self) -> None:
        """
        Stop the motor and hold the current position.
        
        This method is called when active movement should stop and the
        mechanism should maintain its current position. It applies:
        1. PID control to hold position
        2. Gravity compensation to prevent drooping
        3. Brake mode to physically lock the motor when possible
        """
        # Get the current position to use as the new target
        current_position = self.motor.get_position().value
        self.target_position = current_position  # Update target to hold current position
        
        # Calculate holding voltage with gravity compensation
        pid_output = self.position_controller.calculate(current_position, self.target_position)
        gravity_comp = self.kG  # Maintain upward force to counter gravity
        output = pid_output + gravity_comp
        
        # Apply holding voltage to actively maintain position
        self.motor.setVoltage(percent_to_voltage(output))
        # Set brake mode to physically resist movement when no voltage is applied
        self.motor.setNeutralMode(signals.NeutralModeValue.BRAKE)