from commands2 import Command
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from autonomous.paths.base_path import PathState
from autonomous.paths.to_reef import create_path_to_reef

def create_forward_auto(drivetrain: CommandSwerveDrivetrain) -> Command:
    state = PathState()
    
    return (
        drivetrain.runOnce(lambda: drivetrain.seed_field_centric())
        .andThen(create_path_to_reef(drivetrain, state))
    )