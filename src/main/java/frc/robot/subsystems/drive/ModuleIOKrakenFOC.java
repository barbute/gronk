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

  private final double DRIVE_GEAR_RATIO = 6.746031746031747;
  private final double AZIMUTH_GEAR_RATIO = 21.428571428571427;

  private final TalonFXConfiguration DRIVE_MOTOR_CONFIG = new TalonFXConfiguration();
  private final TalonFXConfiguration AZIMUTH_MOTOR_CONFIG = new TalonFXConfiguration();
  private final CANcoderConfiguration CANCODER_CONFIG = new CANcoderConfiguration();

  private final boolean INVERT_AZIMUTH = true;
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

  public ModuleIOKrakenFOC(int index) {
    switch (index) {
      case 0:
        DRIVE_MOTOR = new TalonFX(11);
        AZIMUTH_MOTOR = new TalonFX(21);
        CANCODER = new CANcoder(31);
        ABSOLUTE_ENCODER_OFFSET = new Rotation2d(-2.1); // MUST BE CALIBRATED
        break;
      case 1:
        DRIVE_MOTOR = new TalonFX(12);
        AZIMUTH_MOTOR = new TalonFX(22);
        CANCODER = new CANcoder(32);
        ABSOLUTE_ENCODER_OFFSET = new Rotation2d(2.008); // MUST BE CALIBRATED
        break;
      case 2:
        DRIVE_MOTOR = new TalonFX(13);
        AZIMUTH_MOTOR = new TalonFX(23);
        CANCODER = new CANcoder(33);
        ABSOLUTE_ENCODER_OFFSET = new Rotation2d(-1.01); // MUST BE CALIBRATED
        break;
      case 3:
        DRIVE_MOTOR = new TalonFX(14);
        AZIMUTH_MOTOR = new TalonFX(24);
        CANCODER = new CANcoder(34);
        ABSOLUTE_ENCODER_OFFSET = new Rotation2d(-2.52); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    DRIVE_MOTOR_CONFIG.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    DRIVE_MOTOR_CONFIG.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    DRIVE_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 65.0;
    DRIVE_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    DRIVE_MOTOR_CONFIG.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    DRIVE_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    AZIMUTH_MOTOR_CONFIG.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    AZIMUTH_MOTOR_CONFIG.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
    AZIMUTH_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 40.0;
    AZIMUTH_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    AZIMUTH_MOTOR_CONFIG.MotorOutput.Inverted =
        (INVERT_AZIMUTH)
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    AZIMUTH_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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
