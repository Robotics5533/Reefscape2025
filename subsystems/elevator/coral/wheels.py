from commands2 import Command, Subsystem, cmd  # WPILib command-based programming framework
from phoenix6.hardware import TalonFX  # CTRE TalonFX motor controller (Falcon 500)
from phoenix6 import controls, signals  # Phoenix 6 control and signal libraries
from utils.motor_constants import percent_to_voltage, MOTOR_CONFIG  # Utility functions for motor control

class Wheels(Subsystem):
    """
    Subsystem that controls the wheels of the coral intake mechanism.
    
    This class manages the top wheels that are used to intake and outtake coral game pieces,
    providing commands for controlled movement with proper braking behavior.
    """
    def __init__(self, top_wheels: TalonFX):
        """
        Initialize the Wheels subsystem.
        
        Args:
            top_wheels: TalonFX motor controller that drives the intake wheels
        """
        super().__init__()
        self.top_wheels: TalonFX = top_wheels
        self.config = MOTOR_CONFIG["wheels"]
        # Set neutral mode to brake by default for better control
        self.top_wheels.setNeutralMode(signals.NeutralModeValue.BRAKE)

    def move(self, voltage: float) -> None:
        """Set the wheels motor to a specific voltage
        
        Args:
            voltage: The voltage percentage to apply (-100 to 100)
        """
        # Convert percentage to actual voltage and apply to motor
        self.top_wheels.setVoltage(percent_to_voltage(voltage))
        
    def run(self, speed_percent: float = 20) -> Command:
        """Create a command to run the wheels at a specified speed
        
        Args:
            speed_percent: Speed percentage (-100 to 100), defaults to 20%
            
        Returns:
            Command that runs wheels at specified speed and brakes when finished
        """
        # Limit speed to configured maximum to prevent damage or excessive current draw
        speed_percent = max(-self.config["max_speed"], min(speed_percent, self.config["max_speed"]))
        
        # Create a command that runs the wheels and then brakes when the command ends
        return cmd.runEnd(
            lambda: self.move(speed_percent),  # Function to execute while command is active
            lambda: self.brake()  # Function to execute when command ends
        )
        
    def brake(self) -> None:
        """Stop the motor and engage brake mode"""
        # Set voltage to zero to stop the motor
        self.top_wheels.setVoltage(0)
        # Set to brake mode to actively resist movement when stopped
        self.top_wheels.setNeutralMode(signals.NeutralModeValue.BRAKE)
