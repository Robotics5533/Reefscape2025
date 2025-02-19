from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from phoenix6 import controls

class TopWheelsCommand(Subsystem):
    def __init__(self, motor: TalonFX, speed: float = 30.0):
        super().__init__()
        self.motor: TalonFX = motor
        self.speed = speed
        
        
    def initialize(self) -> None:
        
        pass
        
    def execute(self) -> None:
        
        self.motor.setVoltage(self.speed)
        
    def end(self, interrupted: bool) -> None:
        
        self.motor.setVoltage(0)
        
    def isFinished(self) -> bool:
        
        return False
    
    def set_speed(self, speed: float) -> None:
        self.speed = speed