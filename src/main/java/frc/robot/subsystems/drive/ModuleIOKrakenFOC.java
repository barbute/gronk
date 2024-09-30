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

import static frc.robot.subsystems.drive.DriveConstants.MODULE_GAINS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants.KrakenConfiguration;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfiguration;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

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
  private final TalonFX DRIVE_MOTOR;
  private final TalonFX AZIMUTH_MOTOR;
  private final CANcoder CANCODER;

  private final StatusSignal<Double> DRIVE_POSITION;
  private final StatusSignal<Double> DRIVE_VELOCITY;
  private final StatusSignal<Double> DRIVE_APPLIED_VOLTS;
  private final StatusSignal<Double> DRIVE_CURRENT;

  private final StatusSignal<Double> AZIMUTH_ABSOLUTE_POSITION;
  private final StatusSignal<Double> AZIMIUTH_POSITION;
  private final StatusSignal<Double> AZIMUTH_VELOCITY;
  private final StatusSignal<Double> AZIMUTH_APPLIED_VOLTS;
  private final StatusSignal<Double> AZIMUTH_CURRENT;

  private final double DRIVE_GEAR_RATIO;
  private final double AZIMUTH_GEAR_RATIO;

  private final TalonFXConfiguration DRIVE_MOTOR_CONFIG = new TalonFXConfiguration();
  private final TalonFXConfiguration AZIMUTH_MOTOR_CONFIG = new TalonFXConfiguration();
  private final CANcoderConfiguration CANCODER_CONFIG = new CANcoderConfiguration();

  private final boolean INVERT_AZIMUTH;
  private final Rotation2d ABSOLUTE_ENCODER_OFFSET;

  // Allows for synchronized execution of setting the brake or coast mode for motors
  private static final Executor BRAKE_MODE_EXECUTOR = Executors.newFixedThreadPool(8);

  private final VoltageOut VOLTAGE_CONTROL = new VoltageOut(0.0).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC CURRENT_CONTROL = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VelocityTorqueCurrentFOC VELOCITY_TORQUE_CURRENT_FOC =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC POSITION_CONTROL =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final NeutralOut NEUTRAL_CONTROL = new NeutralOut().withUpdateFreqHz(0.0);

  public ModuleIOKrakenFOC(
      ModuleConfiguration configuration, KrakenConfiguration krakenConfiguration) {
    INVERT_AZIMUTH = configuration.INVERT_AZIMUTH_MOTOR();

    DRIVE_GEAR_RATIO = configuration.DRIVE_MOTOR_GEAR_RATIO();
    AZIMUTH_GEAR_RATIO = configuration.AZIMUTH_MOTOR_GEAR_RATIO();

    DRIVE_MOTOR = new TalonFX(configuration.DRIVE_MOTOR_ID(), DriveConstants.CANBUS);
    AZIMUTH_MOTOR = new TalonFX(configuration.AZIMUTH_MOTOR_ID(), DriveConstants.CANBUS);
    CANCODER = new CANcoder(configuration.ABSOLUTE_ENCODER_ID(), DriveConstants.CANBUS);
    ABSOLUTE_ENCODER_OFFSET = configuration.ABSOLUTE_ENCODER_OFFSET();

    DRIVE_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit =
        krakenConfiguration.DRIVE_STATOR_CURRENT_LIMIT_AMP();
    DRIVE_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable =
        krakenConfiguration.DRIVE_ENABLE_STATOR_CURRENT_LIMIT();
    DRIVE_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit =
        krakenConfiguration.DRIVE_SUPPLY_CURRENT_LIMIT_AMP();
    DRIVE_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable =
        krakenConfiguration.DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT();
    DRIVE_MOTOR_CONFIG.TorqueCurrent.PeakForwardTorqueCurrent =
        krakenConfiguration.DRIVE_PEAK_FORWARD_TORQUE_CURRENT_LIMIT_AMP();
    DRIVE_MOTOR_CONFIG.TorqueCurrent.PeakReverseTorqueCurrent =
        krakenConfiguration.DRIVE_PEAK_REVERSE_TORQUE_CURRENT_LIMIT_AMP();
    DRIVE_MOTOR_CONFIG.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    DRIVE_MOTOR_CONFIG.MotorOutput.NeutralMode = krakenConfiguration.DRIVE_NEUTRAL_MODE();

    AZIMUTH_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit =
        krakenConfiguration.AZIMUTH_STATOR_CURRENT_LIMIT_AMP();
    AZIMUTH_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable =
        krakenConfiguration.AZIMUTH_ENABLE_STATOR_CURRENT_LIMIT();
    AZIMUTH_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit =
        krakenConfiguration.AZIMUTH_SUPPLY_CURRENT_LIMIT_AMP();
    AZIMUTH_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable =
        krakenConfiguration.AZIMUTH_ENABLE_SUPPLY_CURRENT_LIMIT();
    AZIMUTH_MOTOR_CONFIG.TorqueCurrent.PeakForwardTorqueCurrent =
        krakenConfiguration.AZIMUTH_PEAK_FORWARD_TORQUE_CURRENT_LIMIT_AMP();
    AZIMUTH_MOTOR_CONFIG.TorqueCurrent.PeakReverseTorqueCurrent =
        krakenConfiguration.AZIMUTH_PEAK_REVERSE_TORQUE_CURRENT_LIMIT_AMP();
    AZIMUTH_MOTOR_CONFIG.MotorOutput.Inverted =
        (INVERT_AZIMUTH)
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    AZIMUTH_MOTOR_CONFIG.MotorOutput.NeutralMode = krakenConfiguration.AZIMUTH_NEUTRAL_MODE();

    // Conversion affect getPosition(), setPosition(), and getVelocity()
    DRIVE_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
    AZIMUTH_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = AZIMUTH_GEAR_RATIO;
    AZIMUTH_MOTOR_CONFIG.ClosedLoopGeneral.ContinuousWrap = true; // PID Wrapping

    // Attempt to apply configurations four times, if fails womp womp
    for (int i = 0; i < 4; i++) {
      boolean isConfigurationErrorPresent =
          DRIVE_MOTOR.getConfigurator().apply(DRIVE_MOTOR_CONFIG, 0.1) == StatusCode.OK;
      isConfigurationErrorPresent =
          AZIMUTH_MOTOR.getConfigurator().apply(AZIMUTH_MOTOR_CONFIG, 0.1) == StatusCode.OK;
      if (!isConfigurationErrorPresent) {
        break;
      }
    }

    CANCODER_CONFIG.MagnetSensor.MagnetOffset = ABSOLUTE_ENCODER_OFFSET.getRotations();

    CANCODER.getConfigurator().apply(CANCODER_CONFIG);

    setDriveFeedbackGains(MODULE_GAINS.DRIVE_P(), MODULE_GAINS.DRIVE_I(), MODULE_GAINS.DRIVE_D());
    setAzimuthFeedbackGains(
        MODULE_GAINS.AZIMUTH_P(), MODULE_GAINS.AZIMUTH_I(), MODULE_GAINS.AZIMUTH_D());

    DRIVE_POSITION = DRIVE_MOTOR.getPosition();
    DRIVE_VELOCITY = DRIVE_MOTOR.getVelocity();
    DRIVE_APPLIED_VOLTS = DRIVE_MOTOR.getMotorVoltage();
    DRIVE_CURRENT = DRIVE_MOTOR.getSupplyCurrent();

    AZIMUTH_ABSOLUTE_POSITION = CANCODER.getAbsolutePosition();
    AZIMIUTH_POSITION = AZIMUTH_MOTOR.getPosition();
    AZIMUTH_VELOCITY = AZIMUTH_MOTOR.getVelocity();
    AZIMUTH_APPLIED_VOLTS = AZIMUTH_MOTOR.getMotorVoltage();
    AZIMUTH_CURRENT = AZIMUTH_MOTOR.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, DRIVE_POSITION, AZIMIUTH_POSITION); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        DRIVE_VELOCITY,
        DRIVE_APPLIED_VOLTS,
        DRIVE_CURRENT,
        AZIMUTH_ABSOLUTE_POSITION,
        AZIMUTH_VELOCITY,
        AZIMUTH_APPLIED_VOLTS,
        AZIMUTH_CURRENT);

    // TODO Check if this is right lmao
    // Units are in rotations
    AZIMUTH_MOTOR.setPosition(CANCODER.getPosition().getValueAsDouble(), 1.0);

    // Any status signal not explicitly given an update frequency (what we just did above) will be
    // set to 0.0Hz in order to reduce bus util - TLDR: Only send CAN signals we actually use
    DRIVE_MOTOR.optimizeBusUtilization(0.0, 1.0);
    AZIMUTH_MOTOR.optimizeBusUtilization(0.0, 1.0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.hasCurrentControl = true; // TODO What does this do
    inputs.driveMotorConnected =
        BaseStatusSignal.refreshAll(
                DRIVE_POSITION, DRIVE_VELOCITY, DRIVE_APPLIED_VOLTS, DRIVE_CURRENT)
            .isOK();
    inputs.azimuthMotorConntected =
        BaseStatusSignal.refreshAll(
                AZIMIUTH_POSITION, AZIMUTH_VELOCITY, AZIMUTH_APPLIED_VOLTS, AZIMUTH_CURRENT)
            .isOK();
    inputs.absoluteEncoderConnected = BaseStatusSignal.refreshAll(AZIMUTH_ABSOLUTE_POSITION).isOK();

    inputs.drivePositionRad = Units.rotationsToRadians(DRIVE_POSITION.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(DRIVE_VELOCITY.getValueAsDouble());
    inputs.driveAppliedVolts = DRIVE_APPLIED_VOLTS.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {DRIVE_CURRENT.getValueAsDouble()};

    inputs.azimuthAbsolutePosition =
        Rotation2d.fromRotations(AZIMUTH_ABSOLUTE_POSITION.getValueAsDouble());
    inputs.azimuthPosition = Rotation2d.fromRotations(AZIMIUTH_POSITION.getValueAsDouble());
    inputs.azimuthVelocityRadPerSec = Units.rotationsToRadians(AZIMUTH_VELOCITY.getValueAsDouble());
    inputs.azimuthAppliedVolts = AZIMUTH_APPLIED_VOLTS.getValueAsDouble();
    inputs.azimuthCurrentAmps = new double[] {AZIMUTH_CURRENT.getValueAsDouble()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    DRIVE_MOTOR.setControl(VOLTAGE_CONTROL.withOutput(volts));
  }

  @Override
  public void setAzimuthVoltage(double volts) {
    AZIMUTH_MOTOR.setControl(VOLTAGE_CONTROL.withOutput(volts));
  }

  @Override
  public void runCharacterization(double input) {
    DRIVE_MOTOR.setControl(CURRENT_CONTROL.withOutput(input));
  }

  @Override
  public void runDriveVelocitySetpoint(double velocityRadPerSec, double feedforward) {
    DRIVE_MOTOR.setControl(
        VELOCITY_TORQUE_CURRENT_FOC
            .withVelocity(Units.radiansToRotations(velocityRadPerSec))
            .withFeedForward(feedforward));
  }

  @Override
  public void runAzimuthPositionSetpoint(Rotation2d setpoint) {
    AZIMUTH_MOTOR.setControl(POSITION_CONTROL.withPosition(setpoint.getRotations()));
  }

  @Override
  public void setDriveFeedbackGains(double p, double i, double d) {
    DRIVE_MOTOR_CONFIG.Slot0.kP = p;
    DRIVE_MOTOR_CONFIG.Slot0.kI = i;
    DRIVE_MOTOR_CONFIG.Slot0.kD = d;

    DRIVE_MOTOR.getConfigurator().apply(DRIVE_MOTOR_CONFIG, 0.01);
  }

  @Override
  public void setAzimuthFeedbackGains(double p, double i, double d) {
    AZIMUTH_MOTOR_CONFIG.Slot0.kP = p;
    AZIMUTH_MOTOR_CONFIG.Slot0.kI = i;
    AZIMUTH_MOTOR_CONFIG.Slot0.kD = d;

    AZIMUTH_MOTOR.getConfigurator().apply(AZIMUTH_MOTOR_CONFIG, 0.01);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    BRAKE_MODE_EXECUTOR.execute(
        () -> {
          synchronized (DRIVE_MOTOR_CONFIG) {
            DRIVE_MOTOR_CONFIG.MotorOutput.NeutralMode =
                (enable) ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            DRIVE_MOTOR.getConfigurator().apply(DRIVE_MOTOR_CONFIG, 0.25);
          }
        });
  }

  @Override
  public void setAzimuthBrakeMode(boolean enable) {
    BRAKE_MODE_EXECUTOR.execute(
        () -> {
          synchronized (AZIMUTH_MOTOR_CONFIG) {
            AZIMUTH_MOTOR_CONFIG.MotorOutput.NeutralMode =
                (enable) ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            AZIMUTH_MOTOR.getConfigurator().apply(AZIMUTH_MOTOR_CONFIG, 0.25);
          }
        });
  }

  @Override
  public void stop() {
    DRIVE_MOTOR.setControl(NEUTRAL_CONTROL);
    AZIMUTH_MOTOR.setControl(NEUTRAL_CONTROL);
  }
}
