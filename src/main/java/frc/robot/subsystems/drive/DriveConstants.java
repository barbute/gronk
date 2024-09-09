// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** Swerve constants class */
public class DriveConstants {
  public record DriveConfiguration(
      int GYRO_ID,
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

  public static final DriveConfiguration DRIVE_CONFIGURATION =
      switch (Constants.CURRENT_MODE) {
        case REAL, SIM -> new DriveConfiguration(
            10,
            Units.inchesToMeters(2.0),
            Units.inchesToMeters(21.75),
            Units.inchesToMeters(21.75),
            Units.feetToMeters(14.5),
            Units.feetToMeters(50.0),
            12.0,
            6.0);
        default -> new DriveConfiguration(10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };

  public static final ModuleConfiguration[] MODULE_CONFIGURATIONS =
      switch (Constants.CURRENT_MODE) {
        case REAL -> new ModuleConfiguration[] {
          new ModuleConfiguration(11, 21, 31, new Rotation2d(-2.1), true, 6.746, 21.429),
          new ModuleConfiguration(12, 22, 32, new Rotation2d(2.008), true, 6.746, 21.429),
          new ModuleConfiguration(13, 23, 33, new Rotation2d(-1.01), true, 6.746, 21.429),
          new ModuleConfiguration(14, 24, 34, new Rotation2d(-2.52), true, 6.746, 21.429)
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

  public static final ModuleGains MODULE_GAINS =
      switch (Constants.CURRENT_MODE) {
        case REAL -> new ModuleGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
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
