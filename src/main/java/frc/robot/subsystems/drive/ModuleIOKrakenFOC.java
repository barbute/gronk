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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOKrakenFOC implements ModuleIO {
  private final TalonFX driveMotor;
  private final TalonFX azimuthMotor;
  private final CANcoder cancoder;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> azimuthAbsolutePosition;
  private final StatusSignal<Double> azimuthPosition;
  private final StatusSignal<Double> azimuthVelocity;
  private final StatusSignal<Double> azimuthAppliedVolts;
  private final StatusSignal<Double> azimuthCurrent;

  private final double DRIVE_GEAR_RATIO = 6.746031746031747;
  private final double AZIMUTH_GEAR_RATIO = 21.428571428571427;

  private final boolean isAzimuthMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOKrakenFOC(int index) {
    switch (index) {
      case 0:
        driveMotor = new TalonFX(11);
        azimuthMotor = new TalonFX(21);
        cancoder = new CANcoder(31);
        absoluteEncoderOffset = new Rotation2d(-2.1); // MUST BE CALIBRATED
        break;
      case 1:
        driveMotor = new TalonFX(12);
        azimuthMotor = new TalonFX(22);
        cancoder = new CANcoder(32);
        absoluteEncoderOffset = new Rotation2d(2.008); // MUST BE CALIBRATED
        break;
      case 2:
        driveMotor = new TalonFX(13);
        azimuthMotor = new TalonFX(23);
        cancoder = new CANcoder(33);
        absoluteEncoderOffset = new Rotation2d(-1.01); // MUST BE CALIBRATED
        break;
      case 3:
        driveMotor = new TalonFX(14);
        azimuthMotor = new TalonFX(24);
        cancoder = new CANcoder(34);
        absoluteEncoderOffset = new Rotation2d(-2.52); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = 65.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveMotor.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    azimuthMotor.getConfigurator().apply(turnConfig);
    setAzimuthBrakeMode(false);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    driveAppliedVolts = driveMotor.getMotorVoltage();
    driveCurrent = driveMotor.getSupplyCurrent();

    azimuthAbsolutePosition = cancoder.getAbsolutePosition();
    azimuthPosition = azimuthMotor.getPosition();
    azimuthVelocity = azimuthMotor.getVelocity();
    azimuthAppliedVolts = azimuthMotor.getMotorVoltage();
    azimuthCurrent = azimuthMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, drivePosition, azimuthPosition); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        azimuthAbsolutePosition,
        azimuthVelocity,
        azimuthAppliedVolts,
        azimuthCurrent);
    driveMotor.optimizeBusUtilization();
    azimuthMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        azimuthAbsolutePosition,
        azimuthPosition,
        azimuthVelocity,
        azimuthAppliedVolts,
        azimuthCurrent);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.azimuthAbsolutePosition =
        Rotation2d.fromRotations(azimuthAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.azimuthPosition =
        Rotation2d.fromRotations(azimuthPosition.getValueAsDouble() / AZIMUTH_GEAR_RATIO);
    inputs.azimuthVelocityRadPerSec =
        Units.rotationsToRadians(azimuthVelocity.getValueAsDouble()) / AZIMUTH_GEAR_RATIO;
    inputs.azimuthAppliedVolts = azimuthAppliedVolts.getValueAsDouble();
    inputs.azimuthCurrentAmps = new double[] {azimuthCurrent.getValueAsDouble()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setAzimuthVoltage(double volts) {
    azimuthMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveMotor.getConfigurator().apply(config);
  }

  @Override
  public void setAzimuthBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isAzimuthMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    azimuthMotor.getConfigurator().apply(config);
    azimuthMotor.setNeutralMode(config.NeutralMode);
  }
}
