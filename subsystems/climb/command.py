from commands2 import Command, Subsystem, cmd  # WPILib command-based programming framework
from phoenix6.hardware import TalonFX  # CTRE TalonFX motor controller (Falcon 500)
from phoenix6 import controls, signals  # Phoenix 6 control and signal libraries
from utils.motor_constants import percent_to_voltage, MOTOR_CONFIG  # Utility functions for motor control


class Climb(Subsystem):
    """
    Subsystem that controls the robot's climbing mechanism.
    
    This class manages the motor that powers the climbing mechanism,
    providing commands for controlled movement with proper speed limiting
    and braking behavior.
    """
    def __init__(self, motor: TalonFX):
        """
        Initialize the Climb subsystem.
        
        Args:
            motor: TalonFX motor controller that drives the climbing mechanism
        """
        super().__init__()
        self.motor: TalonFX = motor
        self.config = MOTOR_CONFIG["climb"]  # Load configuration parameters for the climb subsystem
        
    def run(self, speed_percent: float = 20) -> Command:
        """
        Create a command to run the climbing mechanism at a specified speed.
        
        Args:
            speed_percent: Speed percentage (-100 to 100), defaults to 20%
            
        Returns:
            Command that runs the climbing mechanism at specified speed and brakes when finished
        """
        # Clamp speed to configured limits for safety and motor protection
        speed_percent = max(-self.config["max_speed"], min(speed_percent, self.config["max_speed"]))
        voltage = percent_to_voltage(speed_percent)  # Convert percentage to voltage
        
        # Create a command that runs the climbing mechanism and then brakes when the command ends
        return cmd.runEnd(
            lambda: self.motor.setVoltage(voltage),  # Function to execute while command is active
            lambda: self.brake()  # Function to execute when command ends
        )
        
    def brake(self) -> None:
        """Stop the motor and engage brake mode"""
        self.motor.setVoltage(0)  # Set voltage to zero to stop the motor
        self.motor.setNeutralMode(signals.NeutralModeValue.BRAKE)  # Set to brake mode to actively resist movement
        

    def set_speed(self, speed_percent: float) -> None:
        """Set the motor speed as a percentage (-100 to 100)"""
        # Clamp speed to configured limits for safety and motor protection
        speed_percent = max(-self.config["max_speed"], min(speed_percent, self.config["max_speed"]))
        # Convert percentage to voltage and apply to motor
        self.motor.setVoltage(percent_to_voltage(speed_percent))