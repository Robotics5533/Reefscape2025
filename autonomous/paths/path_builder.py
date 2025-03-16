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
    velocity: tuple[float, float, float] = (0, 0, 0)  # (x, y, z)
    distance: tuple[float, float, float] = (0, 0, 0)  # (x, y, z) in feet
    direction: tuple[Direction, Direction, Direction] = (Direction.FORWARD, Direction.RIGHT, Direction.CLOCKWISE)  # (x, y, z)
    timeout: float = None

class PathBuilder:
    def __init__(self, drivetrain: CommandSwerveDrivetrain, state: PathState):
        self.drivetrain = drivetrain
        self.state = state
        self.movements: List[Movement] = []


    # def move_x(self, velocity: float, distance_feet: float, direction: Direction = Direction.FORWARD, timeout: float = None) -> 'PathBuilder':
    #     if direction not in [Direction.FORWARD, Direction.BACKWARD]:
    #         raise ValueError("X movement must be FORWARD or BACKWARD")
    #     actual_velocity = velocity if direction == Direction.FORWARD else -velocity
    #     self.movements.append(Movement(velocity=actual_velocity, distance=distance_feet, direction=direction, timeout=timeout))
    #     return self
    
    # def move_y(self, velocity: float, distance_feet: float, direction: Direction = Direction.RIGHT, timeout: float = None) -> 'PathBuilder':
    #     if direction not in [Direction.LEFT, Direction.RIGHT]:
    #         raise ValueError("Y movement must be LEFT or RIGHT")
    #     actual_velocity = velocity if direction == Direction.RIGHT else -velocity
    #     self.movements.append(Movement(velocity=actual_velocity, distance=distance_feet, direction=direction, timeout=timeout))
    #     return self
    
    # def rotate(self, rate: float, degrees: float, direction: Direction = Direction.CLOCKWISE, timeout: float = None) -> 'PathBuilder':
    #     if direction not in [Direction.CLOCKWISE, Direction.COUNTERCLOCKWISE]:
    #         raise ValueError("Rotation must be CLOCKWISE or COUNTERCLOCKWISE")
    #     actual_rate = rate if direction == Direction.CLOCKWISE else -rate
    #     self.movements.append(Movement(velocity=actual_rate, distance=degrees, direction=direction, timeout=timeout))
    #     return self
    
    def move(self, velocity: tuple[float, float, float], distance: tuple[float, float, float],
              direction: tuple[Direction, Direction, Direction] = (Direction.FORWARD, Direction.RIGHT, Direction.CLOCKWISE),
              timeout: float = None) -> 'PathBuilder':
        if direction[0] not in [Direction.FORWARD, Direction.BACKWARD]:
            raise ValueError("X movement must be FORWARD or BACKWARD")
        if direction[1] not in [Direction.LEFT, Direction.RIGHT]:
            raise ValueError("Y movement must be LEFT or RIGHT")
        if direction[2] not in [Direction.CLOCKWISE, Direction.COUNTERCLOCKWISE]:
            raise ValueError("Z movement must be CLOCKWISE or COUNTERCLOCKWISE")
            
        actual_velocity = (
            velocity[0] if direction[0] == Direction.FORWARD else -velocity[0],
            velocity[1] if direction[1] == Direction.RIGHT else -velocity[1],
            velocity[2] if direction[2] == Direction.CLOCKWISE else -velocity[2]
        )
        
        self.movements.append(Movement(
            velocity=actual_velocity,
            distance=distance,
            direction=direction,
            timeout=timeout
        ))
        return self
    
    def _create_movement_command(self, movement: Movement) -> Command:
        request = swerve.requests.FieldCentric()
        
        request = (request
            .with_velocity_x(movement.velocity[0])
            .with_velocity_y(movement.velocity[1])
            .with_rotational_rate(movement.velocity[2]))
        
        initial_x = self.drivetrain.get_state().pose.x
        initial_y = self.drivetrain.get_state().pose.y
        initial_heading = float(self.drivetrain.get_state().raw_heading.degrees())
        
        def check_condition():
            current_x = self.drivetrain.get_state().pose.x
            current_y = self.drivetrain.get_state().pose.y
            current_heading = float(self.drivetrain.get_state().raw_heading.degrees())
            
            x_done = abs(current_x - initial_x) >= feetToMeters(movement.distance[0])
            y_done = abs(current_y - initial_y) >= feetToMeters(movement.distance[1])
            z_done = abs(current_heading - initial_heading) >= abs(movement.distance[2])
            
            return x_done and y_done and z_done
        
        command = self.drivetrain.apply_request(lambda: request).until(check_condition)
        
        
        if movement.timeout is not None:
            command = command.withTimeout(movement.timeout)
            
        return command
    
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
        )

def create_path(drivetrain: CommandSwerveDrivetrain, state: PathState, path_name: str,
                build_func: Callable[[PathBuilder], PathBuilder]) -> Command:
    builder = PathBuilder(drivetrain, state)
    
    build_func(builder)
    return builder.build(path_name)