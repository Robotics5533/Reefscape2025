# Reefscape2025 Robot Code

## Project Overview
This repository contains the code for the Reefscape2025 FRC (FIRST Robotics Competition) robot. The robot is designed to compete in the 2025 game, which involves manipulating coral-like game pieces and navigating a reef-themed playing field.

## Robot Capabilities
- **Swerve Drive**: Precise omnidirectional movement using a swerve drivetrain
- **Elevator System**: Multi-level positioning system for game piece placement
- **Climbing Mechanism**: Allows the robot to climb structures on the field
- **Coral Intake**: Specialized mechanism for collecting and manipulating game pieces
- **Vision Processing**: Camera-based alignment with field targets

## Code Structure

### Main Files
- `robot.py`: The main robot program that initializes all systems and manages the robot lifecycle
- `robotcontainer.py`: Configures all subsystems and binds controller inputs to commands

### Subsystems
The code is organized into subsystems, each controlling a specific part of the robot:

#### Drivetrain
- `subsystems/command_swerve_drivetrain.py`: Controls the swerve drive system, allowing the robot to move in any direction and rotate independently

#### Elevator
- `subsystems/elevator/command.py`: Manages the elevator that positions game pieces at different heights
- `subsystems/elevator/coral/`: Contains mechanisms for manipulating the coral game pieces

#### Climbing Mechanism
- `subsystems/climb/command.py`: Controls the climbing system that allows the robot to hang from field elements

#### Vision System
- `subsystems/vision/align.py`: Processes camera data to align with targets on the field

### Autonomous
The autonomous code allows the robot to operate without driver input for the first 15 seconds of a match:

- `autonomous/autons/`: Contains different autonomous routines
- `autonomous/paths/`: Defines movement paths for autonomous navigation
- `autonomous/commands/`: Individual actions the robot can perform during autonomous

### Utilities
- `utils/`: Helper functions and constants used throughout the codebase
- `generated/`: Auto-generated configuration files for the swerve drive

## How It Works

### Control System
The robot uses a command-based programming framework from WPILib, where:
1. **Subsystems** represent physical parts of the robot
2. **Commands** are actions that use one or more subsystems
3. **Buttons** on controllers trigger commands

### Driver Controls
The robot uses two Xbox controllers:
- **Driver Controller**: Controls robot movement (left stick for translation, right stick for rotation)
- **Functional Controller**: Operates mechanisms like the elevator, intake, and climbing system

### Autonomous Operation
During the autonomous period, the robot:
1. Determines its starting position on the field
2. Follows pre-programmed paths using odometry
3. Performs actions like placing game pieces at specific locations
4. Can use vision systems to fine-tune its positioning

## Setup and Deployment

### Requirements
- Python 3.10 or newer
- RobotPy and associated libraries
- Phoenix 6 libraries for CTRE motor controllers

### Deploying to the Robot
1. Connect to the robot's network
2. Run `python -m robotpy deploy`

### Simulation
The code includes simulation support for testing without physical hardware:
- Run `python -m robotpy sim`

## Contributing
To contribute to this codebase:
1. Understand the subsystem you want to modify
2. Test changes in simulation before deploying to the robot
3. Document your changes thoroughly

## Troubleshooting
- Check motor IDs in the constants files if mechanisms aren't responding
- Verify controller mappings in `robotcontainer.py` if controls aren't working
- Use the Phoenix Tuner X application to diagnose issues with CTRE devices