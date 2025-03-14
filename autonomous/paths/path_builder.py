from typing import List, Callable, Optional, Tuple
from dataclasses import dataclass
from enum import Enum, auto
from wpimath.units import feetToMeters
from phoenix6 import swerve
from commands2 import Command
from autonomous.paths.base_path import BasePath, PathState
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from wpilib import SmartDashboard

class Direction(Enum):
    FORWARD = auto()
    BACKWARD = auto()
    LEFT = auto()
    RIGHT = auto()
    CLOCKWISE = auto()
    COUNTERCLOCKWISE = auto()

@dataclass
class Movement:
    velocity: float = 0
    distance: float = 0
    direction: Direction = Direction.FORWARD

class PathBuilder:
    def __init__(self, drivetrain: CommandSwerveDrivetrain, state: PathState):
        self.drivetrain = drivetrain
        self.state = state
        self.movements: List[Movement] = []



    def move_x(self, velocity: float, distance_feet: float, direction: Direction = Direction.FORWARD) -> 'PathBuilder':
        if direction not in [Direction.FORWARD, Direction.BACKWARD]:
            raise ValueError("X movement must be FORWARD or BACKWARD")
        actual_velocity = velocity if direction == Direction.FORWARD else -velocity
        self.movements.append(Movement(velocity=actual_velocity, distance=distance_feet, direction=direction))
        return self
    
    def move_y(self, velocity: float, distance_feet: float, direction: Direction = Direction.RIGHT) -> 'PathBuilder':
        if direction not in [Direction.LEFT, Direction.RIGHT]:
            raise ValueError("Y movement must be LEFT or RIGHT")
        actual_velocity = velocity if direction == Direction.RIGHT else -velocity
        self.movements.append(Movement(velocity=actual_velocity, distance=distance_feet, direction=direction))
        return self
    
    def rotate(self, rate: float, degrees: float, direction: Direction = Direction.CLOCKWISE) -> 'PathBuilder':
        if direction not in [Direction.CLOCKWISE, Direction.COUNTERCLOCKWISE]:
            raise ValueError("Rotation must be CLOCKWISE or COUNTERCLOCKWISE")
        actual_rate = rate if direction == Direction.CLOCKWISE else -rate
        self.movements.append(Movement(velocity=actual_rate, distance=degrees, direction=direction))
        return self
    
    def _create_movement_command(self, movement: Movement) -> Command:
        request = swerve.requests.FieldCentric()
        
        if movement.direction in [Direction.FORWARD, Direction.BACKWARD]:
            request = request.with_velocity_x(movement.velocity).with_velocity_y(0).with_rotational_rate(0)
            initial_x = self.drivetrain.get_state().pose.x
            condition = lambda: abs(self.drivetrain.get_state().pose.x - initial_x) >= feetToMeters(movement.distance)
        
        elif movement.direction in [Direction.LEFT, Direction.RIGHT]:
            request = request.with_velocity_x(0).with_velocity_y(movement.velocity).with_rotational_rate(0)
            initial_y = self.drivetrain.get_state().pose.y
            condition = lambda: abs(self.drivetrain.get_state().pose.y - initial_y) >= feetToMeters(movement.distance)
        
        else:  # CLOCKWISE or COUNTERCLOCKWISE
            request = request.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(movement.velocity)
            initial_heading = float(self.drivetrain.get_state().raw_heading.degrees())
            condition = lambda: abs(float(self.drivetrain.get_state().raw_heading.degrees()) - initial_heading) >= abs(movement.distance)
          
            
        return self.drivetrain.apply_request(lambda: request).until(condition)
    
    def build(self, path_name: str) -> Command:
        if not self.movements:
            return self.drivetrain.runOnce(lambda: None)
        
        self.state.current_path = path_name
        
        command = self._create_movement_command(self.movements[0])
        
        for movement in self.movements[1:]:
            command = command.andThen(self._create_movement_command(movement))
        
        return (
            command
            .andThen(self.drivetrain.runOnce(lambda: self.drivetrain.set_control(swerve.requests.SwerveDriveBrake())))
            .finallyDo(lambda interrupted: self.drivetrain.set_control(swerve.requests.SwerveDriveBrake()) if interrupted else None)
        )

def create_path(drivetrain: CommandSwerveDrivetrain, state: PathState, path_name: str,
                build_func: Callable[[PathBuilder], PathBuilder]) -> Command:
    builder = PathBuilder(drivetrain, state)
    build_func(builder)
    return builder.build(path_name)