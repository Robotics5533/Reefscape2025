from commands2 import Command, Subsystem
from wpimath.controller import PIDController
from ntcore import NetworkTableInstance
from phoenix6 import swerve
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from wpimath.geometry import Pose2d

class LimelightAlign(Subsystem):
    def __init__(self, drivetrain: CommandSwerveDrivetrain):
        super().__init__()
        self.drivetrain = drivetrain
        
        # Initialize NetworkTables for Limelight
        self.limelight = NetworkTableInstance.getDefault().getTable("limelight")
        
        # PID controllers for X and Y alignment
        # Tune these values based on robot behavior
        self.x_controller = PIDController(0.1, 0.0, 0.02)
        self.y_controller = PIDController(0.1, 0.0, 0.02)
        
        # Set tolerance for position error
        self.x_controller.setTolerance(0.5)  # 0.5 degrees of tolerance
        self.y_controller.setTolerance(0.5)
        
    def align_to_target(self) -> Command:
        """Creates a command to align the robot with the AprilTag."""
        
        def initialize():
            # Set Limelight LED and pipeline
            self.limelight.putNumber("ledMode", 3)  # Force LED on
            self.limelight.putNumber("pipeline", 0)  # Use default pipeline
            
            # Reset PID controllers
            self.x_controller.reset()
            self.y_controller.reset()
        
        def execute():
            # Get target data from Limelight
            has_target = self.limelight.getNumber("tv", 0.0)
            
            if has_target:
                # Get target offsets (in degrees)
                x_offset = self.limelight.getNumber("tx", 0.0)
                y_offset = self.limelight.getNumber("ty", 0.0)
                
                # Calculate correction values using PID
                x_speed = self.x_controller.calculate(x_offset, 0)
                y_speed = self.y_controller.calculate(y_offset, 0)
                
                # Limit correction values
                x_speed = max(min(x_speed, 0.5), -0.5)
                y_speed = max(min(y_speed, 0.5), -0.5)
                
                # Apply movement using swerve drive
                self.drivetrain.set_control(
                    swerve.requests.FieldCentric()
                    .with_velocity_x(y_speed)  # Forward/backward based on y offset
                    .with_velocity_y(-x_speed)  # Left/right based on x offset
                    .with_rotational_rate(0)  # Maintain current heading
                )
            else:
                # No target found, stop moving
                self.drivetrain.set_control(swerve.requests.SwerveDriveBrake())
        
        def isFinished() -> bool:
            # Check if we have a target and are within tolerance
            has_target = self.limelight.getNumber("tv", 0.0)
            if not has_target:
                return False
                
            return (self.x_controller.atSetpoint() and 
                    self.y_controller.atSetpoint())
        
        def end(interrupted: bool):
            # Stop moving and turn off Limelight LED
            self.drivetrain.set_control(swerve.requests.SwerveDriveBrake())
            self.limelight.putNumber("ledMode", 1)  # Force LED off
        
        return Command(
            initialize=initialize,
            execute=execute,
            isFinished=isFinished,
            end=end,
            requirements=[self]
        )

