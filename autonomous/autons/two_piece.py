from commands2 import Command, WaitCommand, cmd
from commands2.cmd import parallel
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.elevator.command import Elevator, ElevatorMode, ElevatorPositions
from subsystems.elevator.coral.wheels import Wheels
from autonomous.paths.base_path import PathState
from autonomous.paths.path_builder import create_path, Direction
from autonomous.commands import place_coral
from autonomous.commands.for_command import create_for_command
from utils.math import inchesToRotations
from wpimath.geometry import Rotation2d
from phoenix6 import swerve

def create_two_piece_auto(drivetrain: CommandSwerveDrivetrain, elevator: Elevator = None, wheels: Wheels = None) -> Command:
    state = PathState()
    
    # Create the path commands using the new path builder
    path_to_reef = create_path(drivetrain, state, "to_reef", 
        lambda builder: builder.move_x(1.5, 6.2, Direction.BACKWARD))
    
    rotate_reef = create_path(drivetrain, state, "rotate_reef", 
        lambda builder: builder.rotate(1.5, 55, Direction.CLOCKWISE))
    
    path_after_rotate = create_path(drivetrain, state, "to_reef_from_rotate", 
        lambda builder: builder.move_x(1.5, 1.5, Direction.BACKWARD))
    
    # Combine path commands sequentially
    drive_command = path_to_reef.andThen(rotate_reef).andThen(path_after_rotate)
    
    if elevator is None:
        return (
            drivetrain.runOnce(lambda: drivetrain.reset_rotation(
    Rotation2d.fromDegrees(180) + drivetrain.get_operator_forward_direction()
))
            .andThen(drive_command)
        )
    elevator.set_tolerance(inchesToRotations(0.5))

    return (
        drivetrain.runOnce(lambda: drivetrain.reset_rotation(
    Rotation2d.fromDegrees(180) + drivetrain.get_operator_forward_direction()
))
        .andThen(drive_command)
        # Uncomment these lines if you want to use the elevator and wheels
        # .andThen(elevator.move(ElevatorPositions.Autonl4, ElevatorMode.POSITION))
        # .andThen(create_for_command(lambda: wheels.move(50), 1.2, lambda: wheels.brake()))
        # .andThen(elevator.move(ElevatorPositions.Level1, ElevatorMode.POSITION))
        # .andThen(create_for_command(lambda: elevator.set_tolerance(0.5), 0.1))
    )