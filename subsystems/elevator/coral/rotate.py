from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from phoenix6 import signals
from utils.motor_constants import percent_to_voltage

class RotateCommand(Subsystem):
    def __init__(self, motor: TalonFX):
        super().__init__()
        self.motor: TalonFX = motor
        self.motor.setNeutralMode(signals.NeutralModeValue.BRAKE)
        # Constants for physics-based compensation
        self.kG = 0.4  # Gravity compensation - Counteracts the weight
        self.kS = 0.1  # Static friction compensation
        
    def rotate(self, stick_value: int) -> None:
        if abs(stick_value) < 0.1:
            # Apply holding force when stopped
            gravity_comp = self.kG  # Always apply upward force to counter gravity
            self.motor.setVoltage(percent_to_voltage(gravity_comp))
            return
            
        motor_speed = 20 * stick_value
        # Add gravity compensation to the motor output
        gravity_comp = self.kG  # Always apply upward force
        static_comp = self.kS * (1 if motor_speed > 0 else -1)  # Apply in direction of motion
        compensated_speed = motor_speed + gravity_comp + static_comp
        
        self.motor.setNeutralMode(signals.NeutralModeValue.COAST)
        self.motor.setVoltage(percent_to_voltage(compensated_speed))
        
    def brake(self) -> None:
        # Apply holding voltage instead of complete stop
        gravity_comp = self.kG  # Maintain upward force to counter gravity
        self.motor.setVoltage(percent_to_voltage(gravity_comp))
        self.motor.setNeutralMode(signals.NeutralModeValue.BRAKE)
