from commands2 import Command, cmd
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
from wpimath.units import feetToMeters

def create_forward_auto(drivetrain: CommandSwerveDrivetrain) -> Command:
    # Store initial x position in a mutable container to allow modification from lambda
    seed_pose = drivetrain.get_state().pose
    
    def update_seed():
        global seed_pose
        seed_pose = drivetrain.get_state().pose
    
    return (
        drivetrain.runOnce(update_seed)
        .andThen(
            drivetrain.apply_request(
                lambda: (
                    swerve.requests.FieldCentric()
                    .with_velocity_x(0.5)
                    .with_velocity_y(0)
                    .with_rotational_rate(0)
                )
            )
            .until(lambda: abs(drivetrain.get_state().pose.x - seed_pose.x) >= feetToMeters(7))
        )
    )