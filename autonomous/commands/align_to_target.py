from autonomous.commands.base_command import BaseCommand
from subsystems.vision.align import LimelightAlign
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain

class AlignToTarget(BaseCommand):
    def __init__(self, drivetrain: CommandSwerveDrivetrain):
        super().__init__()
        self.limelight_align = LimelightAlign(drivetrain)
        
    def get_command(self):
        """Returns the command to align the robot with the AprilTag."""
        return self.limelight_align.align_to_target()
