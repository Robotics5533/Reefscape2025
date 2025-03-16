from commands2 import Command, Subsystem, cmd
from enum import Enum
from utils.HashMap import HashMap
from phoenix6 import hardware, controls, signals
from wpimath.controller import PIDController
from wpimath.trajectory import TrapezoidProfile
from utils.constants import ELEVATOR_LEVELS, single_rotation_inches
from utils.math import inchesToRotations, is_between
from wpilib import SmartDashboard, Timer
from utils.motor_constants import percent_to_voltage, MOTOR_CONFIG, voltage_to_percent

class ElevatorPositions():
    Level1 = 0
    Level2 = 6
    Level3 = 14.625
    Level4 = 27.75 # previously 27.8125
    Autonl4 = 27.8125 # previously 28

class ElevatorMode(Enum):
    MANUAL = "manual" 
    POSITION = "position" 

class Elevator(Subsystem):
    def __init__(self, leading_motor: hardware.TalonFX, following_motor: hardware.TalonFX):
        super().__init__()
        self.elevator_positions = HashMap()
        self.elevator_positions.put("LEVEL_1", ElevatorPositions.Level1)
        self.elevator_positions.put("LEVEL_2", ElevatorPositions.Level2)
        self.elevator_positions.put("LEVEL_3", ElevatorPositions.Level3)
        self.elevator_positions.put("LEVEL_4", ElevatorPositions.Level4)

        self.leading_motor = leading_motor
        self.following_motor = following_motor
        self.config = MOTOR_CONFIG["elevator"]

        # Zero both motors before any movement
        self.leading_motor.set_position(0)
        self.following_motor.set_position(0)
        self.leading_motor.setNeutralMode(signals.NeutralModeValue.BRAKE)
        self.following_motor.setNeutralMode(signals.NeutralModeValue.BRAKE)

        # PID controller for position control
        self.position_controller = PIDController(
            0.5,  # P gain - Increased for stronger position holding
            0.01,  # I gain - Added to eliminate steady-state error
            0  # D gain - Increased for better damping
        )
        self.position_controller.setIZone(0.125)
        self.position_controller.setTolerance(0.5)
        
        # Constants for physics-based compensation
        self.kG = 1.5  # Gravity compensation - Counteracts the weight of the elevator
        self.kS = 0.5  # Static friction compensation - Overcomes motor and mechanism friction
        
        # Trapezoidal profile constraints
        # Maximum velocity in rotations per second
        self.max_velocity = 2.0  # Adjust based on your elevator's capabilities
        # Maximum acceleration in rotations per second squared
        self.max_acceleration = 4.0  # Adjust based on your elevator's capabilities
        
        # Create the trapezoidal profile constraints
        self.constraints = TrapezoidProfile.Constraints(self.max_velocity, self.max_acceleration)
        
        # Initialize profile state variables
        self.goal = TrapezoidProfile.State()
        self.current_state = TrapezoidProfile.State()
        self.profile = None
        self.profile_start_time = 0
        
        # Dashboard outputs
        SmartDashboard.putBoolean("Elevator/Finished", False)
        SmartDashboard.putBoolean("Elevator/Braked", False)
        SmartDashboard.putNumber("Elevator/kG", self.kG)
        SmartDashboard.putNumber("Elevator/kS", self.kS)
        SmartDashboard.putNumber("Elevator/MaxVelocity", self.max_velocity)
        SmartDashboard.putNumber("Elevator/MaxAcceleration", self.max_acceleration)
        self.periodic()

    def periodic(self):
        leading_pos = self.leading_motor.get_position().value
        following_pos = self.following_motor.get_position().value
        SmartDashboard.putNumber("Elevator/Leading Motor Position", leading_pos)
        SmartDashboard.putNumber("Elevator/Following Motor Position", following_pos)
        SmartDashboard.putNumber("Elevator/Leading Motor Position(in)", leading_pos * single_rotation_inches)
        SmartDashboard.putNumber("Elevator/Following Motor Position(in)", following_pos * single_rotation_inches)
        
        # If we have an active profile, update dashboard with profile state
        if self.profile is not None:
            SmartDashboard.putNumber("Elevator/Profile Goal Position", self.goal.position)
            SmartDashboard.putNumber("Elevator/Profile Current Position", self.current_state.position)
            SmartDashboard.putNumber("Elevator/Profile Current Velocity", self.current_state.velocity)

    def move_motor(self, speed_percent: float):
        speed_percent = max(-self.config["max_speed"], min(speed_percent, self.config["max_speed"]))

        voltage = percent_to_voltage(speed_percent)

        self.leading_motor.setVoltage(voltage)
        self.following_motor.setVoltage(-voltage)

        self.periodic()
    
    def set_tolerance(self, value: float):
        self.position_controller.setTolerance(value)
    
    def move(self, value: float | str, mode: ElevatorMode = ElevatorMode.MANUAL) -> Command:
        """
        Unified movement function that supports both manual and position-based control
        
        Args:
            value: Either speed percentage (-100 to 100) for manual mode,
                  or target position in inches/level name for position mode
            mode: ElevatorMode.MANUAL for direct control or ElevatorMode.POSITION for PID control
        """

        if mode == ElevatorMode.MANUAL:
            return cmd.runEnd(
                lambda: self.move_motor(float(value)),
                lambda: self.brake_manual()
            )
        else:
            if isinstance(value, str):
                if value not in self.elevator_positions:
                    return cmd.none()
                target_position = self.elevator_positions.get(value).value
            else:
                target_position = float(value)

            def initialize():
                # Convert target position from inches to rotations
                target_rotations = inchesToRotations(target_position)
                
                # Get current position and velocity
                current_position = self.leading_motor.get_position().value
                current_velocity = self.leading_motor.get_velocity().value
                
                # Set up the initial and goal states
                self.current_state = TrapezoidProfile.State(current_position, current_velocity)
                self.goal = TrapezoidProfile.State(target_rotations, 0)
                
                # Create a new profile
                self.profile = TrapezoidProfile(self.constraints)
                
                # Record the start time
                self.profile_start_time = Timer.getFPGATimestamp()
                
                # Reset the PID controller
                self.position_controller.reset()
                
                SmartDashboard.putBoolean("Elevator/Finished", False)

            def execute():
                # Calculate elapsed time since profile started
                elapsed_time = Timer.getFPGATimestamp() - self.profile_start_time
                
                # Calculate the profile state at the current time
                self.current_state = self.profile.calculate(elapsed_time, self.current_state, self.goal)
                
                # Get actual current position
                actual_position = self.leading_motor.get_position().value
                
                # Use PID to correct any position error between profile and actual position
                pid_output = self.position_controller.calculate(
                    actual_position, 
                    self.current_state.position
                )
                
                # Add feedforward for velocity and physics compensation
                # Velocity feedforward helps track the profile velocity
                velocity_ff = self.current_state.velocity * 0.1  # Adjust this gain as needed
                
                # Apply physics-based compensation
                gravity_comp = self.kG  # Always apply upward force to counter gravity
                static_comp = self.kS * (1 if pid_output > 0 else -1)  # Apply in direction of desired motion
                
                # Combine all control outputs
                output = pid_output + velocity_ff + gravity_comp + static_comp
                
                # Apply the control output to the motors
                self.move_motor(voltage_to_percent(output))
                
                # Update dashboard
                self.periodic()
                
            def is_finished():
                # Check if we've reached the goal position within tolerance
                actual_position = self.leading_motor.get_position().value
                at_goal = abs(actual_position - self.goal.position) < self.position_controller.getTolerance()
                
                # Also check if the profile is complete (we're at the end time)
                profile_complete = abs(self.current_state.position - self.goal.position) < 0.01 and \
                                 abs(self.current_state.velocity) < 0.01
                
                if at_goal and profile_complete:
                    SmartDashboard.putBoolean("Elevator/Finished", True)
                    return True
                return False
            
            def end(interrupted):
                # Clean up when the command ends
                self.profile = None
                self.brake()
                
            return cmd.Command.runinitialize(initialize, execute, end, is_finished)

    def brake(self):
        SmartDashboard.putBoolean("Elevator/Braked", True)
        self.leading_motor.setNeutralMode(signals.NeutralModeValue.BRAKE)
        self.following_motor.setNeutralMode(signals.NeutralModeValue.BRAKE)
        self.leading_motor.setVoltage(0)
        self.following_motor.setVoltage(0)
    
    def brake_manual(self):
        SmartDashboard.putBoolean("Elevator/Braked", True)
        # Set voltage to zero to allow the elevator to fall in manual mode
        self.leading_motor.setVoltage(0)
        self.following_motor.setVoltage(0)