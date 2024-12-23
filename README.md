# gronk
gronk is an [FRC Robot program](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/creating-robot-program.html) written in Java designed to control Prosper Robotics 2024 offseason robot *Redux* - an SDS MK4i L2 Kraken Motor swerve drive. The robot was designed for the 2024 FRC Season's game: [Cresdendo](https://www.firstinspires.org/robotics/frc/game-and-season). **This is a personal project and is intended for simulation use cases only, and is not meant to be ran on any physical hardware in its current state**.

## Table of Contents
- [Description](#description)
  - [Todo](#todo)
  - [AdvantageKit](#advantagekit)
  - [Packages](#packages)
- [Installation](#installation)
  - [Requirements](#requirements)
- [Useage](#usage)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Description
The robot design is based off of FRC [Team 2056](https://www.thebluealliance.com/team/2056)'s robot for the 2024 FRC season. It is a swerve drive with an arm in the middle, and a shooter as its end-affector. There is a full-width intake spanning the back of the robot, that passes a note to the shooter when the arm is in its resting position.

This code is currently structured using subsystems, where each subsystem is used to control a major component of the robot (E.g. Drivetrain, Arm). Each subsystem, or group of subsystems, has a set of desired states it may be in. These state's dictate what those subsystems will do.

> [!NOTE]
> Currently the code is a work-in-progress. I intend to tie the arm and shooter together into a superstructure class and that class will dictate the state's for those subsystems.

### TODO
A personal todo of things to complete

- [ ] Refactor arm code
  - [ ] Instead of a set of "states" use goals
  - [ ] Change how constants and configurations are setup (store the Kraken config object?? Though then it wouldn't be cross motor compatable soz)
- [ ] Add intake code
- [ ] Add flywheel + indexer code
- [ ] Add superstructure code
  - [ ] Proper inter-op between subsystems and the supertructure with the drive

### AdvantageKit
This project utilizes [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit), a logging framework developed by FRC [Team 6328](https://www.thebluealliance.com/team/6328). Thus, each subsystem is structured to have a hardware and simulation interface. For more information about structuring an AdvantageKit project, see [here](https://docs.advantagekit.org/recording-inputs/io-interfaces).

### Packages
- [`frc.robot.subsystems`](src/main/java/frc/robot/subsystems/)

  Contains control logic for each subsystem of the robot. Some subsystems may be nested under a state-machine for ease of management.

- [`frc.robot.util`](src/main/java/frc/robot/util)

  Contains utility classes imported from various projects. Note that utility classes are here because they are used throughout this project. If a class is specific to one subsystem, it should be included in that subsystem's directory.

## Installation
Ensure your system meets the project's [requirements](#requirements).

1. Clone the repository with `git clone git@github.com:barbute/gronk.git`
2. Build the repository with `./gradlew build` or by typing into the WPILib VSCode Command Palette `>WPILib: Build robot code`

### Requirements

> [!NOTE]
> This project currently uses WPILib 2024.3.2, but will likely be migrating to the 2025 beta soon (if I continue to maintain this project)

- [JDK 17 +](https://www.oracle.com/java/technologies/javase/jdk17-archive-downloads.html)
- [WPI VSCode](https://github.com/wpilibsuite/allwpilib/releases/tag/v2024.3.2)
  - Follow WPILib's [guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
  - Must be version 2024.3.2

## Usage
Robots obviously differ from each other. If you are looking to use part of this code for your own robot, make sure that you understand what each function is doing *in context*.

If you are looking to use an entire subsystems, and keep the logic with the hardware implementation, you may simply edit the constants file for that subsystem, located in its respective directory. The constants file serves as a "map" for that subsystem, where each value represents an aspect of the physical robot that needs to be used in code.

When PID tuning, the gains for the controllers as well as their custom setpoints and published to NetworkTables via the [LoggedTunableNumber](src/main/java/frc/robot/util/debugging/LoggedTunableNumber.java) class. This allows for tuning controllers without needing to re-deploy the code each time you update a gain. If you wish to actually save a gain that you've tuned, copy that value into the respective variable for that subsystem's constants file.

## License
This project is licesnsed under the GNU General Public License v3.0. See the [LICENSE file](/AdvantageKit-License.md) for more information.

## Acknowledgements
- Thanks to [AnshulBompada](https://github.com/AnshulBompada) for reviewing some aspects of the code
- Thanks to [Jelatinone](https://github.com/Jelatinone) for inspiration on the [Installation](#installation) section of this README file
- Thanks to FRC [Team 6328](https://github.com/Mechanical-Advantage) for using their [AdvantageKit template](https://github.com/Mechanical-Advantage/AdvantageKit/releases/tag/v3.2.1) code, as well as inspiration for controlling the Arm and the [Alert](src/main/java/frc/robot/util/debugging/Alert.java) and [LoggedTunableNumber](src/main/java/frc/robot/util/debugging/LoggedTunableNumber.java) utility classes
  - Also thanks for inspiring the control scheme of the drive subsystem, where "controllers" are used to modify the behavior of the drive
