# gronk
gronk is an [FRC Robot program](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/creating-robot-program.html) written in Java designed to control Prosper Robotics 2024 offseason robot *Redux* - an SDS MK4i L2 Kraken Motor swerve drive. The robot was designed for the 2024 FRC Season's game: [Cresdendo](https://www.firstinspires.org/robotics/frc/game-and-season).

## Table of Contents
- [Description](#description)
 - [Packages](#packages)
- [Installation](#installation)
- [Useage](#usage)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Description
The robot is based off of FRC [Team 2056](https://www.thebluealliance.com/team/2056)'s robot for the 2024 FRC season. It is a swerve drive with an arm in the middle, and a shooter as its end-affector. There is a full-width intake spanning the back of the robot, that passes the note to the shooter when the arm is in its resting position.

This code is currently structured using subsystems, where each subsystem is used to control a major component of the robot (E.g. Drivetrain, Arm). Each subsystem, or group of subsystems, has a set of desired states it may be in. These state's dictate what those subsystems will do.

> [!NOTE]
> Currently the code is a work-in-progress. I intend to tie the arm and shooter together into a superstructure class and that class will dictate the state's for those subsystems.

### AdvantageKit
This project utilizes [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit), a logging framework developed by FRC [Team 6328](https://www.thebluealliance.com/team/6328). Thus, each subsystem is structured to have a hardware and simulation interface. For more information about structuring an AdvantageKit project, see [here](https://docs.advantagekit.org/recording-inputs/io-interfaces).

### Packages
- [`frc.robot.subsystems`](src/main/java/frc/robot/subsystems/)
  
  Contains control logic for each subsystem of the robot. Some subsystems may be nested under a state-machine for ease of management.

- [`frc.robot.util`](src/main/java/frc/robot/util)

  Contains utility classes imported from various projects. Note that utility classes are here because they are used throughout this project. If a class is specific to one subsystem, it should be included in that subsystem's directory.

## Installation

## Usage

## License

## Acknowledgements
