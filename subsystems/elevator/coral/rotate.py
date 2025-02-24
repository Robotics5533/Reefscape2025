from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from phoenix6 import signals
from utils.motor_constants import percent_to_voltage

class RotateCommand(Subsystem):
    def __init__(self, motor: TalonFX):
        super().__init__()
        self.motor: TalonFX = motor
        self.motor.setNeutralMode(signals.NeutralModeValue.BRAKE)
        
    def rotate(self, stick_value: int) -> None:
        if abs(stick_value) < 0.1:
            self.brake()
            return
            
        motor_speed = 20 * stick_value
        self.motor.setNeutralMode(signals.NeutralModeValue.COAST)
        self.motor.setVoltage(percent_to_voltage(motor_speed))
        
    def brake(self) -> None:
        self.motor.setVoltage(0)
        self.motor.setNeutralMode(signals.NeutralModeValue.BRAKE)