// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmGoal;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOKrakenFOC;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOKrakenFOC;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final Drive drive;
  private final Arm arm;

  private final CommandXboxController controller = new CommandXboxController(0);

  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOKrakenFOC(
                    DriveConstants.MODULE_CONFIGURATIONS[0], DriveConstants.KRAKEN_CONFIGURATION),
                new ModuleIOKrakenFOC(
                    DriveConstants.MODULE_CONFIGURATIONS[1], DriveConstants.KRAKEN_CONFIGURATION),
                new ModuleIOKrakenFOC(
                    DriveConstants.MODULE_CONFIGURATIONS[2], DriveConstants.KRAKEN_CONFIGURATION),
                new ModuleIOKrakenFOC(
                    DriveConstants.MODULE_CONFIGURATIONS[3], DriveConstants.KRAKEN_CONFIGURATION));
        arm = new Arm(new ArmIOKrakenFOC(ArmConstants.ARM_CONFIGURATION));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(DriveConstants.MODULE_CONFIGURATIONS[0]),
                new ModuleIOSim(DriveConstants.MODULE_CONFIGURATIONS[1]),
                new ModuleIOSim(DriveConstants.MODULE_CONFIGURATIONS[2]),
                new ModuleIOSim(DriveConstants.MODULE_CONFIGURATIONS[3]));
        arm = new Arm(new ArmIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        arm = new Arm(new ArmIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption("Drive SysID", drive.runIndetificationRoutineCommand());

    // Configure the button bindings
    configureButtonBindings();
  }

  /** Bind actions that the robot can run to buttons on the controllers */
  private void configureButtonBindings() {
    drive.acceptTeleoperatedInput(
        () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX());
    drive.setDefaultCommand(
        Commands.run(() -> drive.setDriveState(DriveState.TELEOPERATED), drive));
    // Reset gyro heading (Odometry)
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    // Arm to custom setpoint (settable via NT)
    controller
        .a()
        .whileTrue(Commands.run(() -> arm.setArmGoal(ArmGoal.CUSTOM), arm))
        .whileFalse(Commands.run(() -> arm.stop(), arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Schedule drive to accept autonomous speeds from Path Planner, once auto is over drive will
    // return to default command
    Commands.runOnce(() -> drive.setDriveState(DriveState.AUTONOMOUS), drive).schedule();

    return autoChooser.get();
  }
}
