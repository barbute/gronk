// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class ArmIOKrakenFOC implements ArmIO {
  private final TalonFX LEAD_MOTOR;
  private final TalonFX FOLLOW_MOTOR;
  private final CANcoder ABSOLUTE_ENCODER;

  private final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration();

  public ArmIOKrakenFOC() {
    LEAD_MOTOR = new TalonFX(ArmConstants.LEAD_MOTOR_ID);
    FOLLOW_MOTOR = new TalonFX(ArmConstants.FOLLOW_MOTOR_ID);
    FOLLOW_MOTOR.setControl(new Follower(ArmConstants.LEAD_MOTOR_ID, true));

    ABSOLUTE_ENCODER = new CANcoder(ArmConstants.ABSOLUTE_ENCODER_ID);
  }
}
