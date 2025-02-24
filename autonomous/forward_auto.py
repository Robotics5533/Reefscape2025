from commands2 import Command, cmd
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
from wpimath.units import feetToMeters
from dataclasses import dataclass

@dataclass
class AutoState:
    seed_pose = None

def create_forward_auto(drivetrain: CommandSwerveDrivetrain) -> Command:
    state = AutoState()
    
    def update_seed():
        state.seed_pose = drivetrain.get_state().pose
    
    return (
        drivetrain.runOnce(lambda: drivetrain.seed_field_centric())
        .andThen(drivetrain.runOnce(update_seed))
        .andThen(
            drivetrain.apply_request(
                lambda: (
                    swerve.requests.FieldCentric()
                    .with_velocity_x(-0.5)
                    .with_velocity_y(0)
                    .with_rotational_rate(0)
                )
            )
            .until(lambda: state.seed_pose is not None and abs(drivetrain.get_state().pose.x - state.seed_pose.x) >= feetToMeters(7))
        )
    )