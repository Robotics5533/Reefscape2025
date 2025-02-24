from dataclasses import dataclass
from typing import Optional
from wpimath.geometry import Pose2d
from commands2 import Command
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain

@dataclass
class PathState:
    seed_pose: Optional[Pose2d] = None
    current_path: str = ""

class BasePath:
    def __init__(self, drivetrain: CommandSwerveDrivetrain, state: PathState):
        self.drivetrain = drivetrain
        self.state = state
        
    def update_seed(self, path_name: str):
        self.state.seed_pose = self.drivetrain.get_state().pose
        self.state.current_path = path_name
        
    def create_path(self) -> Command:
        """Override this method in derived path classes"""
        raise NotImplementedError