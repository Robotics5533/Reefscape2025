from commands2 import Subsystem
from wpilib import XboxController
from phoenix6.hardware import TalonFX
from phoenix6 import controls

class RotateCommand(Subsystem):
    def __init__(self, motor: TalonFX, controller: XboxController, axis_id: int = XboxController.Axis.kRightY):
        self.motor: TalonFX = motor
        self.controller = controller
        self.axis_id = axis_id
        
        self.motor.set_control(controls.StaticBrake())
        
    def initialize(self) -> None:
        
        pass
        
    def execute(self) -> None:
        
        
        stick_value = self.controller.getRawAxis(self.axis_id)
        
        if abs(stick_value) < 0.1:
            stick_value = 0
        else:
            stick_value = 30 if stick_value > 0 else -30
            
        self.motor.setVoltage(stick_value)
        
    def end(self, interrupted: bool) -> None:
        
        
        

        self.motor.setVoltage(0)
        self.motor.set_control(controls.StaticBrake())
        
    def isFinished(self) -> bool:
        
        
        return False