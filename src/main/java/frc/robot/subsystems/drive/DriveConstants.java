// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** Swerve constants class */
public class DriveConstants {
  public record DriveConfiguration(
      double WHEEL_RADIUS_METER,
      double TRACK_WIDTH_X_METER,
      double TRACK_WIDTH_Y_METER,
      double MAX_LINEAR_VELOCITY_METER_PER_SEC,
      double MAX_LINEAR_ACCELERATION_METER_PER_SEC_SQUARED,
      double MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
      double MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQUARED) {}

  public record ModuleConfiguration(
      int DRIVE_MOTOR_ID,
      int AZIMUTH_MOTOR_ID,
      int ABSOLUTE_ENCODER_ID,
      Rotation2d ABSOLUTE_ENCODER_OFFSET,
      boolean INVERT_AZIMUTH_MOTOR,
      double DRIVE_MOTOR_GEAR_RATIO,
      double AZIMUTH_MOTOR_GEAR_RATIO) {}

  public record KrakenConfiguration(
      boolean DRIVE_ENABLE_STATOR_CURRENT_LIMIT,
      boolean AZIMUTH_ENABLE_STATOR_CURRENT_LIMIT,
      boolean DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT,
      boolean AZIMUTH_ENABLE_SUPPLY_CURRENT_LIMIT,
      double DRIVE_STATOR_CURRENT_LIMIT_AMP,
      double AZIMUTH_STATOR_CURRENT_LIMIT_AMP,
      double DRIVE_SUPPLY_CURRENT_LIMIT_AMP,
      double AZIMUTH_SUPPLY_CURRENT_LIMIT_AMP,
      double DRIVE_PEAK_FORWARD_TORQUE_CURRENT_LIMIT_AMP,
      double DRIVE_PEAK_REVERSE_TORQUE_CURRENT_LIMIT_AMP,
      double AZIMUTH_PEAK_FORWARD_TORQUE_CURRENT_LIMIT_AMP,
      double AZIMUTH_PEAK_REVERSE_TORQUE_CURRENT_LIMIT_AMP,
      NeutralModeValue DRIVE_NEUTRAL_MODE,
      NeutralModeValue AZIMUTH_NEUTRAL_MODE) {}

  public record ModuleGains(
      double DRIVE_S,
      double DRIVE_V,
      double DRIVE_A,
      double DRIVE_P,
      double DRIVE_I,
      double DRIVE_D,
      double AZIMUTH_P,
      double AZIMUTH_I,
      double AZIMUTH_D) {}

  public static final String CANBUS = "drivebase";
  public static final int GYRO_ID = 10;

  public static final DriveConfiguration DRIVE_CONFIGURATION =
      switch (Constants.CURRENT_MODE) {
        case REAL, SIM -> new DriveConfiguration(
            Units.inchesToMeters(2.0),
            Units.inchesToMeters(21.75),
            Units.inchesToMeters(21.75),
            Units.feetToMeters(14.5),
            Units.feetToMeters(50.0),
            12.0,
            6.0);
        default -> new DriveConfiguration(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };

  public static final ModuleConfiguration[] MODULE_CONFIGURATIONS =
      switch (Constants.CURRENT_MODE) {
        case REAL -> new ModuleConfiguration[] {
          new ModuleConfiguration(11, 21, 31, Rotation2d.fromRotations(0.0), true, 6.746, 21.429),
          new ModuleConfiguration(12, 22, 32, Rotation2d.fromRotations(0.0), true, 6.746, 21.429),
          new ModuleConfiguration(13, 23, 33, Rotation2d.fromRotations(0.0), true, 6.746, 21.429),
          new ModuleConfiguration(14, 24, 34, Rotation2d.fromRotations(0.0), true, 6.746, 21.429)
        };
        default -> {
          ModuleConfiguration[] configurations = new ModuleConfiguration[4];
          for (int i = 0; i < 4; i++) {
            configurations[i] =
                new ModuleConfiguration(0, 0, 0, new Rotation2d(), false, 6.746, 21.429);
          }
          yield configurations; // "Return" configurations statement but for arrow function, more
          // complex explanation but am too lazy rn
        }
      };

  public static final KrakenConfiguration KRAKEN_CONFIGURATION =
      new KrakenConfiguration(
          true,
          true,
          true,
          true,
          55.0,
          35.0,
          65.0,
          40.0,
          80.0,
          -80.0,
          40.0,
          -40.0,
          NeutralModeValue.Brake,
          NeutralModeValue.Coast);

  public static final ModuleGains MODULE_GAINS =
      switch (Constants.CURRENT_MODE) {
        case REAL -> new ModuleGains(0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 2.5, 0.0, 0.0);
        case SIM -> new ModuleGains(0.0, 2.06 / 15.32, 0.0, 0.1, 0.0, 0.0, 5.0, 0.0, 0.0);
        case REPLAY -> new ModuleGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        default -> new ModuleGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };

  public static final Translation2d[] MODULE_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(
            DRIVE_CONFIGURATION.TRACK_WIDTH_X_METER() / 2.0,
            DRIVE_CONFIGURATION.TRACK_WIDTH_Y_METER() / 2.0),
        new Translation2d(
            DRIVE_CONFIGURATION.TRACK_WIDTH_X_METER() / 2.0,
            -DRIVE_CONFIGURATION.TRACK_WIDTH_Y_METER() / 2.0),
        new Translation2d(
            -DRIVE_CONFIGURATION.TRACK_WIDTH_X_METER() / 2.0,
            DRIVE_CONFIGURATION.TRACK_WIDTH_Y_METER() / 2.0),
        new Translation2d(
            -DRIVE_CONFIGURATION.TRACK_WIDTH_X_METER() / 2.0,
            -DRIVE_CONFIGURATION.TRACK_WIDTH_Y_METER() / 2.0),
      };

  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(MODULE_TRANSLATIONS);
}
