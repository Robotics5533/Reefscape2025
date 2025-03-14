from commands2 import Command, WaitCommand, cmd
from commands2.cmd import parallel
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.elevator.command import Elevator, ElevatorMode, ElevatorPositions
from subsystems.elevator.coral.wheels import Wheels
from autonomous.paths.base_path import PathState
from autonomous.paths.path_builder import create_path, Direction
from autonomous.commands import place_coral
from autonomous.commands.for_command import create_for_command
from autonomous.commands.align_to_target import AlignToTarget
from utils.math import inchesToRotations
from wpimath.geometry import Rotation2d

def create_forward_and_algae_auto_disabled(drivetrain: CommandSwerveDrivetrain, elevator: Elevator = None, wheels: Wheels = None, continuous_align: bool = False) -> Command:
    state = PathState()
    
    # Create alignment command
    # align_command = AlignToTarget(drivetrain).get_command()
    
    # Create the path commands using the new path builder
    path_command = create_path(drivetrain, state, "to_reef", 
        lambda builder: builder.move_x(0.7, 4.8, Direction.BACKWARD))
    
    
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
        .andThen(path_command)
        .andThen(elevator.move(ElevatorPositions.Autonl4, ElevatorMode.POSITION))
        .andThen(create_for_command(lambda: wheels.move(50), 1.2, lambda: wheels.brake()))
        .andThen(elevator.move(ElevatorPositions.Level2 + .75, ElevatorMode.POSITION))
        .andThen(create_path(drivetrain, state, "to_algae", 
        lambda builder: builder.move_y(0.7, 0.7375, Direction.RIGHT)))
        .andThen(create_for_command(lambda: wheels.move(50), 1.2, lambda: wheels.brake()))
        .andThen(elevator.move(ElevatorPositions.Level1, ElevatorMode.POSITION))
        .andThen(create_for_command(lambda: elevator.set_tolerance(0.5), 0.1))
    )