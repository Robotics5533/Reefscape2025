from commands2 import Command
from subsystems.elevator.command import Elevator, ElevatorMode, ElevatorPositions

def place_coral(elevator: Elevator, target_position: ElevatorPositions | float) -> Command:
    """Creates a command to place coral at a specified height
    
    Args:
        elevator: The elevator subsystem
        target_position: Either an ElevatorPositions enum value or a specific height in inches
    """
    if isinstance(target_position, ElevatorPositions):
        return elevator.move(target_position, ElevatorMode.POSITION)
    else:
        return elevator.move(float(target_position), ElevatorMode.POSITION)