// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

/** Arm constants class */
public class ArmConstants {
  public record ArmGains(double P, double I, double D, double S, double V, double A, double G) {}

  public static final int LEAD_MOTOR_ID = 41;
  public static final int FOLLOW_MOTOR_ID = 42;
  public static final int ABSOLUTE_ENCODER_ID = 43; // TODO Get this

  public static final double GEAR_RATIO = 96.9 / 1.0;
  public static final Rotation2d ABSOLUTE_ENCODER_OFFSET = new Rotation2d();

  public static final ArmGains ARM_GAINS =
      switch (Constants.CURRENT_MODE) {
        case REAL -> new ArmGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case SIM -> new ArmGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        default -> new ArmGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };

  public static final Rotation2d POSITION_TOLERANCE = Rotation2d.fromDegrees(3.0);
}
