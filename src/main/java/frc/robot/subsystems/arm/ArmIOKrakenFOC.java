// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.arm.ArmConstants.KrakenConfiguration;
import java.util.List;

public class ArmIOKrakenFOC implements ArmIO {
  // Follow motor follows lead motor exactly, so we only need to write code for the leader
  private final TalonFX LEAD_MOTOR;
  private final TalonFX FOLLOW_MOTOR;
  private final DutyCycleEncoder ABSOLUTE_ENCODER;

  private final StatusSignal<Double> INTERNAL_POSITION_ROTATION;
  private final StatusSignal<Double> VELOCITY_ROTATION_PER_SEC;
  private final List<StatusSignal<Double>> APPLIED_VOLTAGE;
  private final List<StatusSignal<Double>> SUPPLY_CURRENT_AMP;
  private final List<StatusSignal<Double>> TEMPERATURE_CELSIUS;
  private final List<StatusSignal<Double>> TORQUE_CURRENT_AMP;

  private final VoltageOut VOLTAGE_CONTROL =
      new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC TORQUE_CURRENT_CONTROL =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC POSSITION_TORQUE_CURRENT_CONTROL =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  private final TalonFXConfiguration LEAD_MOTOR_CONFIG = new TalonFXConfiguration();

  public ArmIOKrakenFOC(KrakenConfiguration krakenConfiguration) {
    LEAD_MOTOR = new TalonFX(ArmConstants.LEAD_MOTOR_ID);
    FOLLOW_MOTOR = new TalonFX(ArmConstants.FOLLOW_MOTOR_ID);
    FOLLOW_MOTOR.setControl(new Follower(ArmConstants.LEAD_MOTOR_ID, true));
    ABSOLUTE_ENCODER = new DutyCycleEncoder(ArmConstants.ABSOLUTE_ENCODER_PORT);

    LEAD_MOTOR_CONFIG.Slot0.kP = ArmConstants.ARM_GAINS.P();
    LEAD_MOTOR_CONFIG.Slot0.kI = ArmConstants.ARM_GAINS.I();
    LEAD_MOTOR_CONFIG.Slot0.kD = ArmConstants.ARM_GAINS.D();

    LEAD_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit =
        krakenConfiguration.LEADER_STATOR_CURRENT_LIMIT_AMP();
    LEAD_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable =
        krakenConfiguration.LEADER_ENABLE_STATOR_CURRENT_LIMIT();
    LEAD_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit =
        krakenConfiguration.LEADER_SUPPLY_CURRENT_LIMIT_AMP();
    LEAD_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable =
        krakenConfiguration.LEADER_ENABLE_SUPPLY_CURRENT_LIMIT();
    LEAD_MOTOR_CONFIG.TorqueCurrent.PeakForwardTorqueCurrent =
        krakenConfiguration.LEADER_PEAK_FORWARD_TORQUE_CURRENT_LIMIT_AMP();
    LEAD_MOTOR_CONFIG.TorqueCurrent.PeakReverseTorqueCurrent =
        krakenConfiguration.LEADER_PEAK_REVERSE_TORQUE_CURRENT_LIMIT_AMP();
    LEAD_MOTOR_CONFIG.MotorOutput.Inverted =
        krakenConfiguration.LEADER_INVERT()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    LEAD_MOTOR_CONFIG.MotorOutput.NeutralMode = krakenConfiguration.LEADER_NEUTRAL_MODE();

    // Use internal sensor after reset; Because the internal sensor makes use of the Kraken's output
    // shaft to count rotations (and the motor is geared) we must account for this when using
    // internal PID control
    LEAD_MOTOR_CONFIG.Feedback.RotorToSensorRatio = ArmConstants.GEAR_RATIO;
    LEAD_MOTOR_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    LEAD_MOTOR.getConfigurator().apply(LEAD_MOTOR_CONFIG, 1.0);

    INTERNAL_POSITION_ROTATION = LEAD_MOTOR.getPosition();
    VELOCITY_ROTATION_PER_SEC = LEAD_MOTOR.getVelocity();
    APPLIED_VOLTAGE = List.of(LEAD_MOTOR.getMotorVoltage(), FOLLOW_MOTOR.getMotorVoltage());
    SUPPLY_CURRENT_AMP = List.of(LEAD_MOTOR.getSupplyCurrent(), FOLLOW_MOTOR.getSupplyCurrent());
    TORQUE_CURRENT_AMP = List.of(LEAD_MOTOR.getTorqueCurrent(), FOLLOW_MOTOR.getTorqueCurrent());
    TEMPERATURE_CELSIUS = List.of(LEAD_MOTOR.getDeviceTemp(), FOLLOW_MOTOR.getDeviceTemp());

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        INTERNAL_POSITION_ROTATION,
        VELOCITY_ROTATION_PER_SEC,
        APPLIED_VOLTAGE.get(0),
        APPLIED_VOLTAGE.get(1),
        SUPPLY_CURRENT_AMP.get(0),
        SUPPLY_CURRENT_AMP.get(1),
        TORQUE_CURRENT_AMP.get(0),
        TORQUE_CURRENT_AMP.get(1),
        TEMPERATURE_CELSIUS.get(0),
        TEMPERATURE_CELSIUS.get(1));

    LEAD_MOTOR.optimizeBusUtilization(0.0, 1.0);
    FOLLOW_MOTOR.optimizeBusUtilization(0.0, 1.0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.leaderMotorConnected =
        BaseStatusSignal.refreshAll(
                INTERNAL_POSITION_ROTATION,
                VELOCITY_ROTATION_PER_SEC,
                APPLIED_VOLTAGE.get(0),
                SUPPLY_CURRENT_AMP.get(0),
                TORQUE_CURRENT_AMP.get(0),
                TEMPERATURE_CELSIUS.get(0))
            .isOK();
    inputs.followerMotorConnected =
        BaseStatusSignal.refreshAll(
                APPLIED_VOLTAGE.get(1),
                SUPPLY_CURRENT_AMP.get(1),
                TORQUE_CURRENT_AMP.get(1),
                TEMPERATURE_CELSIUS.get(1))
            .isOK();
    inputs.absoluteEncoderConnected = ABSOLUTE_ENCODER.isConnected();

    inputs.position =
        Rotation2d.fromRotations(
            INTERNAL_POSITION_ROTATION.getValueAsDouble()); // Offset accounted for in configs
    inputs.absoluteEncoderPosition =
        Rotation2d.fromRotations(
            ABSOLUTE_ENCODER.getAbsolutePosition()); // Mounted directly to pivot
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(VELOCITY_ROTATION_PER_SEC.getValueAsDouble());
    inputs.appliedVolts =
        APPLIED_VOLTAGE.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.supplyCurrentAmp =
        SUPPLY_CURRENT_AMP.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.torqueCurrentAmp =
        TORQUE_CURRENT_AMP.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.temperatureCelsius =
        TEMPERATURE_CELSIUS.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
  }

  @Override
  public void setArmVoltage(double voltage) {
    LEAD_MOTOR.setControl(VOLTAGE_CONTROL.withOutput(voltage));
  }

  @Override
  public void setArmPositionSetpoint(Rotation2d positionSetpoint, double feedforward) {
    LEAD_MOTOR.setControl(
        POSSITION_TORQUE_CURRENT_CONTROL
            .withPosition(positionSetpoint.getRotations())
            .withFeedForward(feedforward));
  }

  @Override
  public void setArmCurrent(double currentAmp) {
    LEAD_MOTOR.setControl(TORQUE_CURRENT_CONTROL.withOutput(currentAmp));
  }

  @Override
  public void setFeedbackGains(double p, double i, double d) {
    LEAD_MOTOR_CONFIG.Slot0.kP = p;
    LEAD_MOTOR_CONFIG.Slot0.kI = i;
    LEAD_MOTOR_CONFIG.Slot0.kD = d;

    LEAD_MOTOR.getConfigurator().apply(LEAD_MOTOR_CONFIG);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    LEAD_MOTOR.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    FOLLOW_MOTOR.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
