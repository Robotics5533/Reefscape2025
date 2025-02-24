from commands2 import Command, Subsystem, cmd
from wpilib import XboxController
from phoenix6.hardware import TalonFX
from phoenix6 import controls, signals
from utils.motor_constants import percent_to_voltage

class RotateCommand(Subsystem):
    def __init__(self, motor: TalonFX):
        super().__init__()
        self.motor: TalonFX = motor
        self.motor.setNeutralMode(signals.NeutralModeValue.BRAKE)
        
    def rotate(self, stick_value: int) -> None:
        normalized_value = stick_value / 100.0
        if abs(normalized_value) < 0.1:
            self.brake()
            return
            
        motor_speed = 20 * normalized_value
        self.motor.setVoltage(percent_to_voltage(motor_speed))
        self.motor.setNeutralMode(signals.NeutralModeValue.COAST)
        
    def brake(self) -> None:
        self.motor.setVoltage(0)
        self.motor.setNeutralMode(signals.NeutralModeValue.BRAKE)