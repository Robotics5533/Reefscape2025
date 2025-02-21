from commands2 import Command, Subsystem, cmd
from phoenix6.hardware import TalonFX
from phoenix6 import controls, signals
from utils.motor_constants import percent_to_voltage, MOTOR_CONFIG

class BottomWheels(Subsystem):
    def __init__(self, motor: TalonFX):
        super().__init__()
        self.motor: TalonFX = motor
        self.config = MOTOR_CONFIG["wheels"]
        
    def run(self, speed_percent: float = 20) -> Command:
        speed_percent = max(-self.config["max_speed"], min(speed_percent, self.config["max_speed"]))
        voltage = percent_to_voltage(speed_percent)
        
        return cmd.runEnd(
            lambda: self.motor.setVoltage(voltage),
            lambda: self.brake()
        )
        
    def brake(self) -> None:
        """Stop the motor and engage brake mode"""
        self.motor.setVoltage(0)
        self.motor.setNeutralMode(signals.NeutralModeValue.BRAKE)

    def set_speed(self, speed_percent: float) -> None:
        """Set the motor speed as a percentage (-100 to 100)"""
        speed_percent = max(-self.config["max_speed"], min(speed_percent, self.config["max_speed"]))
        self.motor.setVoltage(percent_to_voltage(speed_percent))