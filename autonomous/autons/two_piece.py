from commands2 import Command
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.elevator.command import Elevator, ElevatorPositions
from autonomous.paths.base_path import PathState
from autonomous.paths.to_reef import create_path_to_reef
from autonomous.paths.to_human_player import create_path_to_human_player
from autonomous.commands import place_coral
from autonomous.commands.lower_elevator import create_lower_elevator
from autonomous.commands.wait import create_wait_command

def create_two_piece_auto(drivetrain: CommandSwerveDrivetrain, elevator: Elevator) -> Command:
    state = PathState()
    
    return (
        drivetrain.runOnce(lambda: drivetrain.seed_field_centric())
        .andThen(create_path_to_reef(drivetrain, state))
        .andThen(place_coral(elevator, ElevatorPositions.Level2))
        .andThen(create_wait_command(2))
        .andThen(create_lower_elevator(elevator))
        .andThen(create_path_to_human_player(drivetrain, state))
        # Add grab coral command here when implemented
        .andThen(create_path_to_reef(drivetrain, state))
        .andThen(place_coral(elevator, ElevatorPositions.Level3))
        .andThen(create_wait_command(2))
        .andThen(create_lower_elevator(elevator))
    )