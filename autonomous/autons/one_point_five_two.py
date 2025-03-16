from commands2 import Command, WaitCommand
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.elevator.command import Elevator, ElevatorMode, ElevatorPositions
from subsystems.elevator.coral.wheels import Wheels
from autonomous.paths.base_path import PathState
from autonomous.paths.path_builder import create_path, Direction
from autonomous.commands.for_command import create_for_command
from utils.math import inchesToRotations
from phoenix6 import swerve
from wpimath.geometry import Rotation2d


def create_one_point_five_two_auto(drivetrain: CommandSwerveDrivetrain, elevator: Elevator = None, wheels: Wheels = None, continuous_align: bool = False) -> Command:
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
        .andThen(create_path(drivetrain, state, "to_reef", 
            lambda builder: builder.move(
                velocity=(1.0, 0.0, 0.0),
                distance=(4.8, 0.0, 0.0),
                direction=(Direction.BACKWARD, Direction.RIGHT, Direction.CLOCKWISE)
            )))
        .andThen(WaitCommand(0.1))
        .andThen(create_path(drivetrain, state, "to_station", 
            lambda builder: builder.move(
                velocity=(0.0, 0.0, 1.0),
                distance=(0.0, 0.0, 55.0),
                direction=(Direction.FORWARD, Direction.RIGHT, Direction.CLOCKWISE)
            )))
        .andThen(WaitCommand(0.1))
        .andThen(create_path(drivetrain, state, "diagonal_move", 
            lambda builder: builder.move(
                velocity=(1.0, 1.0, 0.0),
                distance=(1, 1, 0.0),
                direction=(Direction.BACKWARD, Direction.LEFT, Direction.CLOCKWISE)
            )))
                .andThen(WaitCommand(0.1))
        .andThen(elevator.move(ElevatorPositions.Level2, ElevatorMode.POSITION))
        .andThen(WaitCommand(0.1))
        .andThen(create_for_command(lambda: wheels.move(50), 1.2, lambda: wheels.brake()))
        .andThen(WaitCommand(0.1))
        .andThen(create_path(drivetrain, state, "diagonal_move_station", 
            lambda builder: builder.move(
                velocity=(1.0, 1.0, 1.0),
                distance=(3, 2, -10),
                direction=(Direction.BACKWARD, Direction.RIGHT, Direction.COUNTERCLOCKWISE)
            )))
                .andThen(WaitCommand(0.75))
.andThen(create_path(drivetrain, state, "diagonal_move_reef", 
            lambda builder: builder.move(
                velocity=(1.0, 1.0, 1.0),
                distance=(2, 3, 10),
                direction=(Direction.FORWARD, Direction.LEFT, Direction.CLOCKWISE)
            )))
        .andThen(elevator.move(ElevatorPositions.Level1, ElevatorMode.POSITION))

    )

