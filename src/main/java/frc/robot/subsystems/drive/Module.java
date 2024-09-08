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

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIGURATION;
import static frc.robot.subsystems.drive.DriveConstants.MODULE_CONSTANTS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
  private static final String ROOT_LOG_KEY = "Drive/Module/";

  private static final double WHEEL_RADIUS_METER = DRIVE_CONFIGURATION.WHEEL_RADIUS_METER();

  private final ModuleIO IO;
  private final ModuleIOInputsAutoLogged INPUTS = new ModuleIOInputsAutoLogged();
  private final int INDEX;

  private final SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(
          MODULE_CONSTANTS.DRIVE_S(), MODULE_CONSTANTS.DRIVE_V(), MODULE_CONSTANTS.DRIVE_A());

  private static final LoggedTunableNumber DRIVE_P =
      new LoggedTunableNumber(ROOT_LOG_KEY + "DriveP", MODULE_CONSTANTS.DRIVE_P());
  private static final LoggedTunableNumber DRIVE_I =
      new LoggedTunableNumber(ROOT_LOG_KEY + "DriveI", MODULE_CONSTANTS.DRIVE_I());
  private static final LoggedTunableNumber DRIVE_D =
      new LoggedTunableNumber(ROOT_LOG_KEY + "DriveD", MODULE_CONSTANTS.DRIVE_D());
  private static final LoggedTunableNumber AZIMUTH_P =
      new LoggedTunableNumber(ROOT_LOG_KEY + "AzimuthP", MODULE_CONSTANTS.AZIMUTH_P());
  private static final LoggedTunableNumber AZIMUTH_I =
      new LoggedTunableNumber(ROOT_LOG_KEY + "AzimuthI", MODULE_CONSTANTS.AZIMUTH_I());
  private static final LoggedTunableNumber AZIMUTH_D =
      new LoggedTunableNumber(ROOT_LOG_KEY + "AzimuthD", MODULE_CONSTANTS.AZIMUTH_D());

  public Module(ModuleIO io, int index) {
    this.IO = io;
    this.INDEX = index;
  }

  public void periodic() {
    IO.updateInputs(INPUTS);
    Logger.processInputs("Drive/Module" + INDEX, INPUTS);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> IO.setDriveFeedbackGains(DRIVE_P.get(), DRIVE_I.get(), DRIVE_D.get()),
        DRIVE_P,
        DRIVE_I,
        DRIVE_D);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> IO.setAzimuthFeedbackGains(AZIMUTH_P.get(), AZIMUTH_I.get(), AZIMUTH_D.get()),
        AZIMUTH_P,
        AZIMUTH_I,
        AZIMUTH_D);
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    IO.runDriveVelocitySetpoint(
        optimizedState.speedMetersPerSecond / WHEEL_RADIUS_METER,
        driveFeedforward.calculate(optimizedState.speedMetersPerSecond / WHEEL_RADIUS_METER));
    IO.runAzimuthPositionSetpoint(optimizedState.angle);

    return optimizedState;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    IO.runCharacterization(volts);
    IO.runAzimuthPositionSetpoint(new Rotation2d());
  }

  /** Disables all outputs to motors. */
  public void stop() {
    IO.stop();
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    IO.setDriveBrakeMode(enabled);
    IO.setAzimuthBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return INPUTS.azimuthAbsolutePosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return INPUTS.drivePositionRad * WHEEL_RADIUS_METER;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return INPUTS.driveVelocityRadPerSec * WHEEL_RADIUS_METER;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return INPUTS.driveVelocityRadPerSec;
  }
}
