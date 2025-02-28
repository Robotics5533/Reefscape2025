from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from phoenix6 import signals
from utils.motor_constants import percent_to_voltage
from wpimath.controller import PIDController

class RotateCommand(Subsystem):
    def __init__(self, motor: TalonFX):
        super().__init__()
        self.motor: TalonFX = motor
        self.motor.setNeutralMode(signals.NeutralModeValue.BRAKE)
        # Constants for physics-based compensation
        self.kG = 0.4  # Gravity compensation - Counteracts the weight
        self.kS = 0.1  # Static friction compensation
        
        # PID controller for position control
        self.position_controller = PIDController(
            0.5,  # P gain - For strong position holding
            0.01,  # I gain - Eliminates steady-state error
            0.05   # D gain - Provides damping
        )
        self.position_controller.setTolerance(0.5)  # Position tolerance
        self.target_position = 0.0
        
    def rotate(self, stick_value: int) -> None:
        current_position = self.motor.get_position().value
        
        if abs(stick_value) < 0.1:
            # Use PID control for position holding
            pid_output = self.position_controller.calculate(current_position, self.target_position)
            gravity_comp = self.kG  # Always apply upward force to counter gravity
            static_comp = self.kS * (1 if pid_output > 0 else -1)  # Apply in direction of motion
            output = pid_output + gravity_comp + static_comp
            self.motor.setVoltage(percent_to_voltage(0))
            return
            
        # Update target position based on stick input
        self.target_position = current_position + (stick_value * 0.1)  # Scale stick input for smoother control
        motor_speed = 20 * stick_value
        
        # Add gravity and friction compensation
        gravity_comp = self.kG  # Always apply upward force
        static_comp = self.kS * (1 if motor_speed > 0 else -1)  # Apply in direction of motion
        compensated_speed = motor_speed + gravity_comp + static_comp
        
        self.motor.setNeutralMode(signals.NeutralModeValue.COAST)
        self.motor.setVoltage(percent_to_voltage(motor_speed))
        
    def brake(self) -> None:
        # Use PID control with gravity compensation for precise holding
        current_position = self.motor.get_position().value
        self.target_position = current_position  # Hold current position
        pid_output = self.position_controller.calculate(current_position, self.target_position)
        gravity_comp = self.kG  # Maintain upward force to counter gravity
        output = pid_output + gravity_comp
        self.motor.setVoltage(percent_to_voltage(0))
        self.motor.setNeutralMode(signals.NeutralModeValue.BRAKE)