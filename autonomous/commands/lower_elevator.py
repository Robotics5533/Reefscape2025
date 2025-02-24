from commands2 import Command
from subsystems.elevator.command import Elevator, ElevatorMode, ElevatorPositions

def create_lower_elevator(elevator: Elevator) -> Command:
    """Creates a command to lower the elevator to Level1 (base position)
    
    Args:
        elevator: The elevator subsystem
    """
    return elevator.move(ElevatorPositions.Level1, ElevatorMode.POSITION)