from commands2 import Command, Subsystem, cmd
from enum import Enum
from utils.HashMap import HashMap
from phoenix6 import hardware, controls, signals
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog
from wpilib import RobotController
from wpimath.controller import PIDController
from utils.constants import ELEVATOR_LEVELS, single_rotation_inches
from utils.math import inchesToRotations
from wpilib import SmartDashboard
from utils.motor_constants import percent_to_voltage, MOTOR_CONFIG, voltage_to_percent

class ElevatorPositions(Enum):
    Level1 = 16
    Level2 = 31.875
    Level3 = 47.625
    Level4 = 72

class ElevatorMode(Enum):
    MANUAL = "manual" 
    POSITION = "position" 

class Elevator(Subsystem):
    def __init__(self, leading_motor: hardware.TalonFX, following_motor: hardware.TalonFX):
        super().__init__()
        self.elevator_positions = HashMap()
        self.elevator_positions.put("LEVEL_1", ElevatorPositions.Level1)
        self.elevator_positions.put("LEVEL_2", ElevatorPositions.Level2)
        self.elevator_positions.put("LEVEL_3", ElevatorPositions.Level3)
        self.elevator_positions.put("LEVEL_4", ElevatorPositions.Level4)

        self.leading_motor = leading_motor
        self.following_motor = following_motor
        self.config = MOTOR_CONFIG["elevator"]

        # Zero both motors before any movement
        self.leading_motor.set_position(0)
        self.following_motor.set_position(0)

        self.position_controller = PIDController(
            0.5,  # P gain - Increased for stronger position holding, 0.4375
            0.01,  # I gain - Added to eliminate steady-state error
            0  # D gain - Increased for better damping
        )
        self.position_controller.setIZone(0.125)
        self.position_controller.setTolerance(0.5)
        
        self.kG = 0.1  # Increased gravity compensation

        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(stepVoltage=3),
            SysIdRoutine.Mechanism(lambda x: self.move(x, ElevatorMode.MANUAL), self.log, self),
        )

        self.periodic()

    def periodic(self):
        leading_pos = self.leading_motor.get_position().value
        following_pos = self.following_motor.get_position().value
        SmartDashboard.putNumber("Elevator/Leading Motor Position", leading_pos)
        SmartDashboard.putNumber("Elevator/Following Motor Position", following_pos)
        SmartDashboard.putNumber("Elevator/Leading Motor Position(in)", leading_pos * single_rotation_inches)
        SmartDashboard.putNumber("Elevator/Following Motor Position(in)", following_pos * single_rotation_inches)

    def move_motor(self, speed_percent: float):
        speed_percent = max(-self.config["max_speed"], min(speed_percent, self.config["max_speed"]))
        voltage = percent_to_voltage(speed_percent)
        self.leading_motor.setVoltage(voltage)
        self.following_motor.setVoltage(-voltage)
        self.periodic()

    def move(self, value: float | str, mode: ElevatorMode = ElevatorMode.MANUAL) -> Command:
        """Unified movement function that supports both manual and position-based control
        
        Args:
            value: Either speed percentage (-100 to 100) for manual mode,
                  or target position in inches/level name for position mode
            mode: ElevatorMode.MANUAL for direct control or ElevatorMode.POSITION for PID control
        """
        if mode == ElevatorMode.MANUAL:
            return cmd.runEnd(
                lambda: self.move_motor(float(value)),
                lambda: self.brake()
            )
        else:
            if isinstance(value, str):
                if value not in self.elevator_positions:
                    return cmd.none()
                target_position = self.elevator_positions.get(value).value
            else:
                target_position = float(value)

            def execute():
                current_position = self.leading_motor.get_position().value
                pid_output = self.position_controller.calculate(
                    current_position, 
                    inchesToRotations(target_position)
                )
                
                output = pid_output # + self.kG
                
                self.move_motor(voltage_to_percent(output))
                
            def is_finished():
                self.periodic()
                return self.position_controller.atSetpoint()
                
            return cmd.run(execute).until(is_finished).finallyDo(lambda interrupted: self.brake())

    def brake(self):
        self.leading_motor.setVoltage(0)
        self.leading_motor.setNeutralMode(signals.NeutralModeValue.BRAKE)
        self.following_motor.setVoltage(0)
        self.following_motor.setNeutralMode(signals.NeutralModeValue.BRAKE)

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
