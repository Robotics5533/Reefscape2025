from commands2 import Command
from subsystems.elevator.command import Elevator, ElevatorMode, ElevatorPositions
from subsystems.elevator.coral.wheels import Wheels
from autonomous.commands.wait import create_wait_command

def place_coral(elevator: Elevator, wheels: Wheels, target_position: ElevatorPositions | float) -> Command:
    """Creates a command sequence to place coral at a specified height.
    This command:
    1. Moves the elevator to the target position
    2. Runs the wheels in reverse to release the coral
    3. Waits for the coral to be released
    4. Lowers the elevator back to ground position
    
    Args:
        elevator: The elevator subsystem
        wheels: The coral wheels subsystem
        target_position: Either an ElevatorPositions enum value or a specific height in inches
    """
    # Move elevator to target position
    if isinstance(target_position, ElevatorPositions):
        elevator_command = elevator.move(target_position, ElevatorMode.POSITION)
    else:
        elevator_command = elevator.move(float(target_position), ElevatorMode.POSITION)
    
    # Create a command sequence that:
    # 1. Moves elevator to position
    # 2. Waits for elevator to stabilize
    # 3. Runs wheels in reverse to release coral (-50% speed)
    # 4. Waits for 0.5 second while running wheels
    # 5. Lowers elevator back to ground position (Level1)
    return (
        elevator_command
        .andThen(create_wait_command(0.1))  # Wait for elevator to stabilize
        .andThen(wheels.run(-50))  # Run wheels in reverse to release coral
        .andThen(create_wait_command(0.5))  # Run wheels for 0.5 second
        .andThen(elevator.move(ElevatorPositions.Level1, ElevatorMode.POSITION))  # Lower elevator back to ground position
    )