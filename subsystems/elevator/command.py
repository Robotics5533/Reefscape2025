from commands2 import Command, Subsystem, cmd
from enum import Enum
from utils.HashMap import HashMap
from phoenix6 import hardware, controls
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog
from wpilib import RobotController
from wpimath.controller import PIDController
import math
from phoenix6 import signals

from utils.constants import ELEVATOR_LEVELS
from utils.math import inchesToRotations


class ElevatorPositions(Enum):
    Level1 = 16
    Level2 = 31.875
    Level3 = 47.625
    Level4 = 72

class CommandElevator(Subsystem):
    def __init__(
        self, leading_motor: hardware.TalonFX, following_motor: hardware.TalonFX
    ):
        self.elevator_positions = HashMap()
        self.elevator_positions.put("LEVEL_1", ElevatorPositions.Level1)
        self.elevator_positions.put("LEVEL_2", ElevatorPositions.Level2)
        self.elevator_positions.put("LEVEL_3", ElevatorPositions.Level3)
        self.elevator_positions.put("LEVEL_4", ElevatorPositions.Level4)

        self.leading_motor = leading_motor
        self.following_motor = following_motor

        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(stepVoltage=3), 
            SysIdRoutine.Mechanism(self.move, self.log, self),
        )

    def move_motor(self, voltage: float):
        self.leading_motor.setVoltage(voltage)
        self.following_motor.setVoltage(-voltage)

    def move(self, voltage: float) -> Command:
        return cmd.runEnd(
            lambda: self.move_motor(voltage),
            lambda: self.brake()
        )
    
    def brake(self):
        self.leading_motor.setVoltage(0)
        self.leading_motor.setNeutralMode(signals.NeutralModeValue.BRAKE)
        self.following_motor.setVoltage(0)
        self.following_motor.setNeutralMode(signals.NeutralModeValue.BRAKE)

    # def move_to(self, level: str) -> Command:
    #     """
    #     Moves the elevator to the specified level using a PID controller.
    #     The level should be one of the values in ELEVATOR_LEVELS (e.g., "LEVEL_1").
    #     """
    #     # Validate level input.
    #     if level not in ELEVATOR_LEVELS:
    #         raise ValueError(
    #             f"Invalid level: {level}. Valid levels: {', '.join(ELEVATOR_LEVELS)}"
    #         )
    #     target_inches = self.elevator_positions.get(level).value

    #     target_rotations = inchesToRotations(target_inches)

    #     pid = PIDController(0.02, 0.0, 0.0)
    #     pid.setSetpoint(target_rotations)

    #     def measurement():
    #         return self.leading_motor.get_position().value

        # return cmd.runEnd(
        #     lambda: self.move(pid.calculate(measurement())),
        #     lambda: self.brake()
        # )

    def log(self, sys_id_routine: SysIdRoutineLog) -> None:
        sys_id_routine.motor("leading_motor").voltage(
            self.leading_motor.get_motor_voltage().value
            * RobotController.getBatteryVoltage()
        ).position(self.leading_motor.get_position().value).velocity(
            self.leading_motor.get_velocity().value
        )

        sys_id_routine.motor("following_motor").voltage(
            self.following_motor.get_motor_voltage().value
            * RobotController.getBatteryVoltage()
        ).position(self.following_motor.get_position().value).velocity(
            self.following_motor.get_velocity().value
        )

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction):
        return self.sys_id_routine.dynamic(direction)
