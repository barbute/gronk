// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private final double LOOP_PERIOD_SEC = 0.02;

  private final SingleJointedArmSim ARM =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(2),
          ArmConstants.GEAR_RATIO,
          ArmConstants.SIMULATION_ARM_CONFIGURATION.J_KG_METER_SQUARED(),
          ArmConstants.SIMULATION_ARM_CONFIGURATION.ARM_LENGTH_METERS(),
          ArmConstants.SIMULATION_ARM_CONFIGURATION.MIN_ARM_ANGLE().getRadians(),
          ArmConstants.SIMULATION_ARM_CONFIGURATION.MAX_ARM_ANGLE().getRadians(),
          ArmConstants.SIMULATION_ARM_CONFIGURATION.SIMULATE_GRAVITY(),
          ArmConstants.SIMULATION_ARM_CONFIGURATION.START_ARM_ANGLE().getRadians());

  private final PIDController FEEDBACK =
      new PIDController(
          ArmConstants.ARM_GAINS.P(), ArmConstants.ARM_GAINS.I(), ArmConstants.ARM_GAINS.D());

  private double positionOffsetRad = ArmConstants.SIMULATION_ARM_POSITION_OFFSET.getRadians();
  private double appliedVoltage = 0.0;

  private boolean feedbackNeedsReset = false;
  private boolean closedLoopControl = false;

  public ArmIOSim() {
    ARM.setState(0.0, 0.0);
    setPosition(0.0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    ARM.update(LOOP_PERIOD_SEC);

    inputs.position = Rotation2d.fromRadians(ARM.getAngleRads() + positionOffsetRad);
    inputs.velocityRadPerSec = ARM.getVelocityRadPerSec();
    inputs.appliedVolts = new double[] {appliedVoltage};
    inputs.supplyCurrentAmp = new double[] {ARM.getCurrentDrawAmps()};
    inputs.torqueCurrentAmp = new double[] {ARM.getCurrentDrawAmps()};
    inputs.temperatureCelsius = new double[] {0.0};
  }

  @Override
  public void setArmVoltage(double voltage) {
    closedLoopControl = false;
    appliedVoltage = MathUtil.clamp(-12.0, 12.0, voltage);

    ARM.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setArmPositionSetpoint(Rotation2d setpoint, double feedforward) {
    if (!closedLoopControl) {
      feedbackNeedsReset = true;
      closedLoopControl = true;
    }
    if (feedbackNeedsReset) {
      FEEDBACK.reset();
      feedbackNeedsReset = false;
    }
  }

  private void setPosition(double positionRad) {
    positionOffsetRad = positionRad - ARM.getAngleRads();
  }
}
