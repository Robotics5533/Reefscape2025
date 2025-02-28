# ===== VISION ALIGNMENT SYSTEM =====
# This file controls the robot's camera-based targeting system
# It helps the robot automatically line up with targets on the field (like AprilTags)
# Think of it like a car's parking assist that helps you perfectly align with a parking space

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
        # (Limelight is a special camera system that can detect targets)
        self.limelight = NetworkTableInstance.getDefault().getTable("limelight")
        
        # PID controllers for X and Y alignment
        # These help the robot make smooth, precise movements to line up with targets
        # (Like how cruise control gradually adjusts your car's speed to match what you set)
        self.x_controller = PIDController(0.1, 0.0, 0.02)
        self.y_controller = PIDController(0.1, 0.0, 0.02)
        
        # Set tolerance for position error
        # (How close is "close enough" - in this case, within 0.5 degrees)
        self.x_controller.setTolerance(0.5)  # 0.5 degrees of tolerance
        self.y_controller.setTolerance(0.5)
        
    def align_to_target(self) -> Command:
        """Creates a command to align the robot with the AprilTag."""
        # This function creates a sequence of steps that will automatically
        # move the robot to line up perfectly with a target (like an AprilTag)
        
        def initialize():
            # STEP 1: SETUP - Get the camera ready
            # Turn on the Limelight's LED light (like a flashlight) to help see targets
            self.limelight.putNumber("ledMode", 3)  # Force LED on
            # Set the camera to use its default vision processing settings
            self.limelight.putNumber("pipeline", 0)  # Use default pipeline
            
            # Reset the alignment controllers so we start fresh
            self.x_controller.reset()
            self.y_controller.reset()
        
        def execute():
            # STEP 2: LOOK - Check if the camera can see a target
            has_target = self.limelight.getNumber("tv", 0.0)
            
            if has_target:
                # STEP 3: MEASURE - If we see a target, figure out how far off-center it is
                # x_offset tells us how far left/right the target is from center of view
                # y_offset tells us how far up/down the target is from center of view
                x_offset = self.limelight.getNumber("tx", 0.0)
                y_offset = self.limelight.getNumber("ty", 0.0)
                
                # STEP 4: CALCULATE - Figure out how fast to move to line up with target
                # (Like how you'd turn the steering wheel more for a sharp turn, less for a gentle one)
                x_speed = self.x_controller.calculate(x_offset, 0)
                y_speed = self.y_controller.calculate(y_offset, 0)
                
                # Make sure we don't move too fast (safety limit)
                x_speed = max(min(x_speed, 0.5), -0.5)
                y_speed = max(min(y_speed, 0.5), -0.5)
                
                # STEP 5: MOVE - Drive the robot to align with the target
                self.drivetrain.set_control(
                    swerve.requests.FieldCentric()
                    .with_velocity_x(y_speed)  # Forward/backward based on y offset
                    .with_velocity_y(-x_speed)  # Left/right based on x offset
                    .with_rotational_rate(0)  # Maintain current heading
                )
            else:
                # If we don't see a target, stop moving and wait
                self.drivetrain.set_control(swerve.requests.SwerveDriveBrake())
        
        def isFinished() -> bool:
            # STEP 6: CHECK - Are we done? Have we lined up with the target?
            
            # First, make sure we can still see the target
            has_target = self.limelight.getNumber("tv", 0.0)
            if not has_target:
                return False  # Can't be done if we don't see a target!
            
            # Check if we're lined up in both directions (left/right and forward/backward)
            # This is like checking if you've parked perfectly in a parking space
            return (self.x_controller.atSetpoint() and 
                    self.y_controller.atSetpoint())
        
        def end(interrupted: bool):
            # STEP 7: FINISH - Clean up when we're done or interrupted
            
            # Stop the robot from moving
            self.drivetrain.set_control(swerve.requests.SwerveDriveBrake())
            
            # Turn off the camera's LED light to save power
            self.limelight.putNumber("ledMode", 1)  # Force LED off
        
        return Command(
            initialize=initialize,
            execute=execute,
            isFinished=isFinished,
            end=end,
            requirements=[self]
        )

