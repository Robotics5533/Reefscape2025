from commands2 import Command
from commands2.cmd import parallel
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.elevator.command import Elevator, ElevatorPositions
from subsystems.elevator.coral.wheels import Wheels
from autonomous.paths.base_path import PathState
from autonomous.paths.to_reef import create_path_to_reef
from autonomous.paths.to_human_player import create_path_to_human_player
from autonomous.commands import place_coral
from autonomous.commands.align_to_target import AlignToTarget

def create_two_piece_auto(drivetrain: CommandSwerveDrivetrain, elevator: Elevator, wheels: Wheels, continuous_align: bool = False) -> Command:
    state = PathState()
    
    # Create alignment command
    align_command = AlignToTarget(drivetrain).get_command()
    
    # Create path commands
    path_to_reef1 = create_path_to_reef(drivetrain, state)
    path_to_human = create_path_to_human_player(drivetrain, state)
    path_to_reef2 = create_path_to_reef(drivetrain, state)
    
    # If continuous alignment is enabled, run paths and alignment in parallel
    drive_command1 = parallel(path_to_reef1, align_command) if continuous_align else path_to_reef1.andThen(align_command)
    drive_command2 = parallel(path_to_human, align_command) if continuous_align else path_to_human.andThen(align_command)
    drive_command3 = parallel(path_to_reef2, align_command) if continuous_align else path_to_reef2.andThen(align_command)
    
    return (
        drivetrain.runOnce(lambda: drivetrain.seed_field_centric())
        .andThen(drive_command1)
        .andThen(place_coral(elevator, wheels, ElevatorPositions.Level2))
        .andThen(drive_command2)
        # Add grab coral command here when implemented
        .andThen(drive_command3)
        .andThen(place_coral(elevator, wheels, ElevatorPositions.Level3))
    )