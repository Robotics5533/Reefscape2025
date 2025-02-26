from commands2 import Command, Subsystem, cmd
from phoenix6.hardware import TalonFX
from phoenix6 import controls, signals
from utils.motor_constants import percent_to_voltage, MOTOR_CONFIG

class Wheels(Subsystem):
    def __init__(self, top_wheels: TalonFX):
        super().__init__()
        self.top_wheels: TalonFX = top_wheels
        self.config = MOTOR_CONFIG["wheels"]

    def move(self, voltage: float):
        self.top_wheels.setVoltage(percent_to_voltage(voltage))
        
    def run(self, speed_percent: float = 20) -> Command:
        speed_percent = max(-self.config["max_speed"], min(speed_percent, self.config["max_speed"]))
        voltage = speed_percent
        
        return cmd.runEnd(
            lambda: self.move(voltage),
            lambda: self.brake()
        )
        
    def brake(self) -> None:
        """Stop the motor and engage brake mode"""
        self.top_wheels.setVoltage(0)
        self.top_wheels.setNeutralMode(signals.NeutralModeValue.BRAKE)
