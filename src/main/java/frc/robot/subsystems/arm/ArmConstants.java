// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

/** Arm constants class */
public class ArmConstants {
  public record ArmGains(double P, double I, double D, double S, double V, double A, double G) {}

  public record KrakenConfiguration(
      boolean LEADER_INVERT,
      boolean LEADER_ENABLE_STATOR_CURRENT_LIMIT,
      boolean LEADER_ENABLE_SUPPLY_CURRENT_LIMIT,
      double LEADER_STATOR_CURRENT_LIMIT_AMP,
      double LEADER_SUPPLY_CURRENT_LIMIT_AMP,
      double LEADER_PEAK_FORWARD_TORQUE_CURRENT_LIMIT_AMP,
      double LEADER_PEAK_REVERSE_TORQUE_CURRENT_LIMIT_AMP,
      NeutralModeValue LEADER_NEUTRAL_MODE) {}

  public record SimulationConfiguration(
      boolean SIMULATE_GRAVITY, // Sim grav since it's just a regular arm
      double J_KG_METER_SQUARED,
      double ARM_LENGTH_METERS,
      Rotation2d MIN_ARM_ANGLE,
      Rotation2d MAX_ARM_ANGLE,
      Rotation2d START_ARM_ANGLE) {}

  public static final int LEAD_MOTOR_ID = 41;
  public static final int FOLLOW_MOTOR_ID = 42;
  public static final int ABSOLUTE_ENCODER_PORT = 2; // TODO Get this

  /*
   * rotations = (meters/(2*pi*wheelRadiusMeters))*gearRatio
   * meters = (rotations/gearRatio)*(2*pi*wheelRadiusMeters)
   */
  public static final double GEAR_RATIO = 96.9 / 1.0;
  public static final Rotation2d ABSOLUTE_ENCODER_OFFSET = new Rotation2d(); // TODO Get this
  public static final Rotation2d SIMULATION_ARM_POSITION_OFFSET = new Rotation2d();

  public static final ArmGains ARM_GAINS =
      switch (Constants.CURRENT_MODE) {
        case REAL -> new ArmGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case SIM -> new ArmGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        default -> new ArmGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };

  public static final KrakenConfiguration ARM_CONFIGURATION =
      new KrakenConfiguration(true, true, true, 80.0, 30.0, 80.0, -80.0, NeutralModeValue.Brake);

  public static final SimulationConfiguration SIMULATION_ARM_CONFIGURATION =
      new SimulationConfiguration(
          true,
          0.0,
          0.0,
          Rotation2d.fromDegrees(0.0),
          Rotation2d.fromDegrees(0.0),
          Rotation2d.fromDegrees(0.0));

  public static final Rotation2d POSITION_TOLERANCE = Rotation2d.fromDegrees(1.0);
}
