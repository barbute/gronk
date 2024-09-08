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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private final DCMotorSim DRIVE_MOTOR =
      new DCMotorSim(
          DCMotor.getKrakenX60Foc(1),
          DriveConstants.MODULE_CONFIGURATIONS[0].DRIVE_MOTOR_GEAR_RATIO(),
          0.025);
  private final DCMotorSim AZIMUTH_MOTOR =
      new DCMotorSim(
          DCMotor.getKrakenX60Foc(1),
          DriveConstants.MODULE_CONFIGURATIONS[0].AZIMUTH_MOTOR_GEAR_RATIO(),
          0.004);

  private final PIDController DRIVE_FEEDBACK =
      new PIDController(
          DriveConstants.MODULE_CONSTANTS.DRIVE_P(),
          DriveConstants.MODULE_CONSTANTS.DRIVE_I(),
          DriveConstants.MODULE_CONSTANTS.DRIVE_D(),
          LOOP_PERIOD_SECS);
  private final PIDController AZIMUTH_FEEDBACK =
      new PIDController(
          DriveConstants.MODULE_CONSTANTS.AZIMUTH_P(),
          DriveConstants.MODULE_CONSTANTS.AZIMUTH_I(),
          DriveConstants.MODULE_CONSTANTS.AZIMUTH_D(),
          LOOP_PERIOD_SECS);

  // Used to simulate coasting after disabled
  private SlewRateLimiter DRIVE_LIMITER = new SlewRateLimiter(2.5);

  // Simulate absolute encoder being at a random value
  private final Rotation2d AZIMUTH_INITIAL_ABSOLUTE_POSITION =
      new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double driveAppliedVolts = 0.0;
  private double azimuthAppliedVolts = 0.0;

  private boolean driveCoast = false;

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }

    if (driveCoast && DriverStation.isDisabled()) {
      setDriveVoltage(DRIVE_LIMITER.calculate(driveAppliedVolts));
    } else {
      DRIVE_LIMITER.reset(driveAppliedVolts);
    }

    DRIVE_MOTOR.update(LOOP_PERIOD_SECS);
    AZIMUTH_MOTOR.update(LOOP_PERIOD_SECS);

    inputs.drivePositionRad = DRIVE_MOTOR.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = DRIVE_MOTOR.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(DRIVE_MOTOR.getCurrentDrawAmps())};

    inputs.azimuthAbsolutePosition =
        new Rotation2d(AZIMUTH_MOTOR.getAngularPositionRad())
            .plus(AZIMUTH_INITIAL_ABSOLUTE_POSITION);
    inputs.azimuthPosition = new Rotation2d(AZIMUTH_MOTOR.getAngularPositionRad());
    inputs.azimuthVelocityRadPerSec = AZIMUTH_MOTOR.getAngularVelocityRadPerSec();
    inputs.azimuthAppliedVolts = azimuthAppliedVolts;
    inputs.azimuthCurrentAmps = new double[] {Math.abs(AZIMUTH_MOTOR.getCurrentDrawAmps())};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    DRIVE_MOTOR.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setAzimuthVoltage(double volts) {
    azimuthAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    AZIMUTH_MOTOR.setInputVoltage(azimuthAppliedVolts);
  }

  @Override
  public void runCharacterization(double input) {
    setDriveVoltage(input);
  }

  @Override
  public void runDriveVelocitySetpoint(double velocityRadPerSec, double feedforward) {
    setDriveVoltage(
        DRIVE_FEEDBACK.calculate(DRIVE_MOTOR.getAngularVelocityRadPerSec(), velocityRadPerSec)
            + feedforward);
  }

  @Override
  public void runAzimuthPositionSetpoint(Rotation2d setpoint) {
    setAzimuthVoltage(
        AZIMUTH_FEEDBACK.calculate(
            AZIMUTH_MOTOR.getAngularPositionRad() + AZIMUTH_INITIAL_ABSOLUTE_POSITION.getRadians(),
            setpoint.getRadians()));
  }

  @Override
  public void setDriveFeedbackGains(double p, double i, double d) {
    DRIVE_FEEDBACK.setPID(p, i, d);
  }

  @Override
  public void setAzimuthFeedbackGains(double p, double i, double d) {
    AZIMUTH_FEEDBACK.setPID(p, i, d);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveCoast = !enable;
  }

  @Override
  public void stop() {
    setDriveVoltage(0.0);
    setAzimuthVoltage(0.0);
  }
}
