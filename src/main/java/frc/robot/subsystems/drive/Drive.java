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

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIGURATION;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.controllers.TeleoperatedController;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.characterization.DriveIdentificationRoutine;
import frc.robot.util.characterization.DriveIdentificationRoutine.MotorType;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  /** State the drive subsystem is in */
  public enum DriveState {
    /** Driving with input from driver controllers */
    TELEOPERATED,
    /** Driving based on preplanned trajectories */
    AUTONOMOUS,
    /** Driving based on a preplanned trajectory */
    TRAJECTORY,
    /** Driving to a location on a field automatically */
    AUTOALIGN,
    /** Characterizing */
    CHARACTERIZATION,
    /** Only runs drive volts, kV = voltage / velocity; sets the drive volts to 1.0 */
    SIMPLECHARACTERIZATION,
    /** Drivetrain is commanded to do nothing */
    STOPPED
  }

  private static final double MAX_LINEAR_SPEED_METER_PER_SEC =
      DRIVE_CONFIGURATION.MAX_LINEAR_VELOCITY_METER_PER_SEC();
  private static final double TRACK_WIDTH_X_METER = DRIVE_CONFIGURATION.TRACK_WIDTH_X_METER();
  private static final double TRACK_WIDTH_Y_METER = DRIVE_CONFIGURATION.TRACK_WIDTH_Y_METER();
  private static final double DRIVE_BASE_RADIUS_METER =
      Math.hypot(TRACK_WIDTH_X_METER / 2.0, TRACK_WIDTH_Y_METER / 2.0);
  private static final double MAX_ANGULAR_SPEED_RAD_PER_SEC =
      MAX_LINEAR_SPEED_METER_PER_SEC / DRIVE_BASE_RADIUS_METER;

  private final GyroIO GYRO_IO;
  private final GyroIOInputsAutoLogged GYRO_INPUTS = new GyroIOInputsAutoLogged();
  private final Module[] MODULES = new Module[4]; // FL, FR, BL, BR

  private SwerveDriveKinematics kinematics = DriveConstants.KINEMATICS;
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private TeleoperatedController teleoperatedController = null;
  private DriveState driveState = DriveState.STOPPED;
  /** The currently desired chassis speeds */
  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
  /** The currently desired PathPlanner chassis speeds */
  private ChassisSpeeds pathPlannerDesiredSpeeds = new ChassisSpeeds();

  private DriveIdentificationRoutine systemIdentificationRoutine = new DriveIdentificationRoutine();

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.GYRO_IO = gyroIO;
    MODULES[0] = new Module(flModuleIO, 0);
    MODULES[1] = new Module(frModuleIO, 1);
    MODULES[2] = new Module(blModuleIO, 2);
    MODULES[3] = new Module(brModuleIO, 3);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        (speeds) -> pathPlannerDesiredSpeeds = speeds,
        new HolonomicPathFollowerConfig(
            MAX_LINEAR_SPEED_METER_PER_SEC, DRIVE_BASE_RADIUS_METER, new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  public void periodic() {
    GYRO_IO.updateInputs(GYRO_INPUTS);
    Logger.processInputs("Drive/Gyro", GYRO_INPUTS);
    for (var module : MODULES) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : MODULES) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Read wheel positions and deltas from each module
    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    if (GYRO_INPUTS.connected) {
      // Use the real gyro angle
      rawGyroRotation = GYRO_INPUTS.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    // Apply odometry update
    poseEstimator.update(rawGyroRotation, modulePositions);

    // Set desired speeds and run desired actions based on the current commanded stated of the drive
    switch (driveState) {
      case TELEOPERATED:
        if (teleoperatedController != null) {
          desiredSpeeds =
              teleoperatedController.computeChassisSpeeds(
                  poseEstimator.getEstimatedPosition().getRotation(),
                  kinematics.toChassisSpeeds(getModuleStates()),
                  MAX_LINEAR_SPEED_METER_PER_SEC,
                  MAX_ANGULAR_SPEED_RAD_PER_SEC);
        }
        break;
      case AUTONOMOUS:
        desiredSpeeds = pathPlannerDesiredSpeeds;
        break;
      case TRAJECTORY:
        break;
      case AUTOALIGN:
        break;
      case CHARACTERIZATION:
        desiredSpeeds = null;
        break;
      case SIMPLECHARACTERIZATION:
        desiredSpeeds = null;
        break;
      case STOPPED:
        desiredSpeeds = null;
        stop();
        break;
      default:
        desiredSpeeds = null;
        break;
    }

    if (desiredSpeeds != null) {
      runSwerve(desiredSpeeds);
    }
  }

  /**
   * Sets the subsystem's desired state, logic runs in periodic()
   *
   * @param desiredState The desired state
   */
  public void setDriveState(DriveState desiredState) {
    driveState = desiredState;
    // TODO: Add logic to reset the heading controller when I make that if the state is the heading
    // controller
  }

  /**
   * Accept the joystick input from the controllers
   *
   * @param xSupplier Forward-backward input
   * @param ySupplier Left-right input
   * @param thetaSupplier Rotational input
   */
  public void acceptTeleoperatedInput(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
    teleoperatedController = new TeleoperatedController(xSupplier, ySupplier, thetaSupplier);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runSwerve(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED_METER_PER_SEC);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = MODULES[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runSwerve(new ChassisSpeeds());
  }

  /** Returns a command to run a full quasistatic, dynamic forward-backward test */
  public Command runIndetificationRoutineCommand() {
    return systemIdentificationRoutine.executeIdentificationRoutineCommand(
        this, MODULES, MotorType.KRAKENX60);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = MODULES[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = MODULES[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMeterPerSec() {
    return MAX_LINEAR_SPEED_METER_PER_SEC;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED_RAD_PER_SEC;
  }
}
