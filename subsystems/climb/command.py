from commands2 import Command, Subsystem, cmd
from phoenix6.hardware import TalonFX
from phoenix6 import controls

class Climb(Subsystem):
    def __init__(self, motor: TalonFX):
        super().__init__()
        self.motor: TalonFX = motor
        
    def run(self, speed: float = 2) -> Command:
        return cmd.runEnd(
            lambda: self.motor.setVoltage(speed),
            lambda: self.motor.setVoltage(0)
        )
    def set_speed(self, speed: float) -> None:

        self.speed = speed