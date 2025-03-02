from commands2 import Command
from commands2.cmd import parallel
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.elevator.command import Elevator, ElevatorPositions
from subsystems.elevator.coral.wheels import Wheels
from autonomous.paths.base_path import PathState
from autonomous.paths.to_reef import create_path_to_reef
from autonomous.commands import place_coral
from autonomous.commands.align_to_target import AlignToTarget

def create_forward_auto(drivetrain: CommandSwerveDrivetrain, elevator: Elevator = None, wheels: Wheels = None, continuous_align: bool = False) -> Command:
    state = PathState()
    
    # Create alignment command
    align_command = AlignToTarget(drivetrain).get_command()
    
    # Create the path command
    path_command = create_path_to_reef(drivetrain, state)
    
    # If continuous alignment is enabled, run path and alignment in parallel
    drive_command = (
        parallel(path_command, align_command) if continuous_align
        else path_command.andThen(align_command)
    )
    
    if elevator is None:
        return (
            drivetrain.runOnce(lambda: drivetrain.seed_field_centric())
            .andThen(drive_command)
        )
    
    return (
        drivetrain.runOnce(lambda: drivetrain.seed_field_centric())
        .andThen(drive_command)
        .andThen(place_coral(elevator, wheels, ElevatorPositions.Level2))
    )