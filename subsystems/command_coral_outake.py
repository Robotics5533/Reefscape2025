from commands2 import Subsystem
import wpilib

class CoralOutake(Subsystem):
    def __init__(self, coral_outake_motor: wpilib.Spark):
        super().__init__()
        self.coral_outake_motor = coral_outake_motor

    def set_outake_power(self, power: float):
        self.coral_outake_motor.set(power / 100)
