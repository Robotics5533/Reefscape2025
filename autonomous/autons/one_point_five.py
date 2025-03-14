from commands2 import Command
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.elevator.command import Elevator, ElevatorMode, ElevatorPositions
from subsystems.elevator.coral.wheels import Wheels
from autonomous.paths.base_path import PathState
from autonomous.paths.path_builder import create_path, Direction
from autonomous.commands.for_command import create_for_command
from utils.math import inchesToRotations
from wpimath.geometry import Rotation2d


def create_one_point_five_auto(drivetrain: CommandSwerveDrivetrain, elevator: Elevator = None, wheels: Wheels = None, continuous_align: bool = False) -> Command:
    state = PathState()
    
    if elevator is None:
        return drivetrain.runOnce(lambda: drivetrain.reset_rotation(
    Rotation2d.fromDegrees(180) + drivetrain.get_operator_forward_direction()
)) 
    
    elevator.set_tolerance(inchesToRotations(0.5))
    
    return (
        drivetrain.runOnce(lambda: drivetrain.reset_rotation(
    Rotation2d.fromDegrees(180) + drivetrain.get_operator_forward_direction()
)) 
        .andThen(create_path(drivetrain, state, "to_reef", lambda builder: builder.move_x(1.0, 3.0, Direction.BACKWARD)))
        # .andThen(elevator.move(ElevatorPositions.Autonl4, ElevatorMode.POSITION))
        # .andThen(create_for_command(lambda: wheels.move(50), 1.2, lambda: wheels.brake()))
        # .andThen(elevator.move(ElevatorPositions.Level1, ElevatorMode.POSITION))
        # .andThen(create_for_command(lambda: elevator.set_tolerance(0.5), 0.1))
        .andThen(create_path(drivetrain, state, "to_station", lambda builder: builder.move_y(1.0, 4.0, Direction.RIGHT)))
        .andThen(create_path(drivetrain, state, "station_forward", lambda builder: builder.move_x(1.0, 4.0, Direction.BACKWARD)))
    )