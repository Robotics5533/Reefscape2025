from wpimath.units import feetToMeters
from phoenix6 import swerve
from commands2 import Command
from autonomous.paths.base_path import BasePath, PathState
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain

class ToHumanPlayerPath(BasePath):
    def __init__(self, drivetrain: CommandSwerveDrivetrain, state: PathState):
        super().__init__(drivetrain, state)
    
    def create_path(self) -> Command:
        self.update_seed("to_human_player")
        
        return (
            self.drivetrain.apply_request(
                lambda: (
                    swerve.requests.FieldCentric()
                    .with_velocity_x(0.5)
                    .with_velocity_y(0)
                    .with_rotational_rate(0)
                )
            )
            .until(lambda: abs(self.drivetrain.get_state().pose.x - self.state.seed_pose.x) >= feetToMeters(5))
        )

def create_path_to_human_player(drivetrain: CommandSwerveDrivetrain, state: PathState) -> Command:
    path = ToHumanPlayerPath(drivetrain, state)
    return path.create_path()