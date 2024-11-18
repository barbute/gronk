// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.debugging.LoggedTunableNumber;
import frc.robot.util.math.EqualsUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  /** Angular position goal for the Arm */
  public enum ArmGoal {
    /** Fixed setpoint for the subwoofer */
    SUBWOOFER(() -> Rotation2d.fromDegrees(0.0)),
    /** Fixed setpoint for optimal intaking */
    STOW(() -> Rotation2d.fromDegrees(0.0)),
    /** Fixed setpoint for optimal scoring into the amp */
    AMP(() -> Rotation2d.fromDegrees(90.0)),
    /** Obtain the best setpoint to shoot from robot's current position */
    AIM(() -> Rotation2d.fromDegrees(0.0)),
    /** Custom setpoint used for debugging purposes */
    CUSTOM(
        () -> Rotation2d.fromDegrees(new LoggedTunableNumber("Arm/CustomGoalDegrees", 0.0).get()));

    // We use suppliers so that dynamically updating setpoints can actually supply their changing
    // pose when we call them
    private Supplier<Rotation2d> goalSupplier;

    ArmGoal(Supplier<Rotation2d> positionGoalSupplier) {
      goalSupplier = positionGoalSupplier;
    }

    /**
     * @return The position goal
     */
    private Rotation2d getGoal() {
      return this.goalSupplier.get();
    }
  }

  private final ArmIO ARM_IO;
  private final ArmIOInputsAutoLogged ARM_INPUTS = new ArmIOInputsAutoLogged();

  private ArmGoal armGoal = null;
  private Rotation2d armPositionSetpoint = new Rotation2d();

  private TrapezoidProfile.Constraints currentConstraints = ArmConstants.MOTION_PROFILE_CONSTRAINTS;
  private TrapezoidProfile profile = new TrapezoidProfile(currentConstraints);
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  private final ArmFeedforward FEEDFORWARD =
      new ArmFeedforward(
          ArmConstants.ARM_GAINS.S(),
          ArmConstants.ARM_GAINS.G(),
          ArmConstants.ARM_GAINS.V(),
          ArmConstants.ARM_GAINS.A());

  private LoggedTunableNumber feedbackP =
      new LoggedTunableNumber("Arm/Feedback/P", ArmConstants.ARM_GAINS.P());
  private LoggedTunableNumber feedbackI =
      new LoggedTunableNumber("Arm/Feedback/I", ArmConstants.ARM_GAINS.I());
  private LoggedTunableNumber feedbackD =
      new LoggedTunableNumber("Arm/Feedback/D", ArmConstants.ARM_GAINS.D());
  private LoggedTunableNumber motionV =
      new LoggedTunableNumber("Arm/Motion/V", ArmConstants.MOTION_PROFILE_CONSTRAINTS.maxVelocity);
  private LoggedTunableNumber motionA =
      new LoggedTunableNumber(
          "Arm/Motion/A", ArmConstants.MOTION_PROFILE_CONSTRAINTS.maxAcceleration);

  private ArmVisualizer visualizer;

  /** Creates a new Arm. */
  public Arm(ArmIO armIO) {
    ARM_IO = armIO;
  }

  @Override
  public void periodic() {
    ARM_IO.updateInputs(ARM_INPUTS);
    Logger.processInputs("Arm/Inputs", ARM_INPUTS);

    if (DriverStation.isDisabled()) {
      stop();
      // Reset profile when disabled
      setpointState = new TrapezoidProfile.State(ARM_INPUTS.position.getRadians(), 0.0);
    }

    if (visualizer == null) {
      visualizer = new ArmVisualizer(getPosition());
    } else {
      visualizer.updateArmPosition(getPosition());
    }

    if (armGoal != null) {
      armPositionSetpoint = armGoal.getGoal();
      if (armGoal == ArmGoal.STOW) {
        armPositionSetpoint = ArmConstants.MIN_POSITION;
      }
      // setpointState =
      //     profile.calculate(
      //         0.02,
      //         setpointState,
      //         new TrapezoidProfile.State(
      //             MathUtil.clamp(
      //                 armPositionSetpoint.getRadians(),
      //                 ArmConstants.MIN_POSITION.getRadians(),
      //                 ArmConstants.MAX_POSITION.getRadians()),
      //             0.0));
      setpointState =
          profile.calculate(
              0.02,
              setpointState,
              new TrapezoidProfile.State(armPositionSetpoint.getRadians(), 0.0));
      if (armGoal == ArmGoal.STOW
          && EqualsUtil.epsilonEquals(
              armPositionSetpoint.getRadians(), ArmConstants.MIN_POSITION.getRadians())
          && atGoal()) {
        ARM_IO.stop();
      } else {
        ARM_IO.setArmPositionSetpoint(
            Rotation2d.fromRadians(setpointState.position),
            FEEDFORWARD.calculate(setpointState.position, setpointState.velocity));
      }
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> setFeedbackGains(feedbackP.get(), feedbackI.get(), feedbackD.get()),
        feedbackP,
        feedbackI,
        feedbackD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> setMotionConstraints(motionV.get(), motionA.get()), motionV, motionA);
  }

  /**
   * Sets the subsystem's desired goal, logic runs in periodic()
   *
   * @param desiredGoal The desired goal
   */
  public void setArmGoal(ArmGoal desiredGoal) {
    armGoal = desiredGoal;
  }

  public void setArmVoltage(double voltage) {
    ARM_IO.setArmVoltage(voltage);
  }

  /** Stops the IO, sets position setpoint to null, commands subsystem state to STOPPED */
  public void stop() {
    armPositionSetpoint = null;
    ARM_IO.stop();
  }

  /** Set the feedback controller's gains */
  private void setFeedbackGains(double p, double i, double d) {
    ARM_IO.setFeedbackGains(p, i, d);
  }

  /** Set the trapezoidal motion-profile's constraints */
  private void setMotionConstraints(double maxVelocity, double maxAcceleration) {
    currentConstraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
  }

  @AutoLogOutput(key = "Arm/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(
        setpointState.position,
        armPositionSetpoint.getRadians(),
        ArmConstants.POSITION_TOLERANCE.getRadians());
  }

  @AutoLogOutput(key = "Arm/PositionGoal")
  /**
   * @return The current goal position of the arm
   */
  public Rotation2d getPositionSetpoint() {
    if (armPositionSetpoint == null) {
      return new Rotation2d();
    } else {
      return armPositionSetpoint;
    }
  }

  @AutoLogOutput(key = "Arm/ProfilePositionSetpointRadians")
  public double getProfilePositionSetpointRadians() {
    return setpointState.position;
  }

  @AutoLogOutput(key = "Arm/ProfileVelocitySetpointRadians")
  public double getProfileVelocitySetpointRadians() {
    return setpointState.velocity;
  }

  @AutoLogOutput(key = "Arm/Error")
  /**
   * @return The error between the goal position and the current position of the mechanism
   */
  public Rotation2d getError() {
    return getPositionSetpoint().minus(getPosition());
  }

  /**
   * @return The position of the arm as a Rotation2d
   */
  public Rotation2d getPosition() {
    return ARM_INPUTS.position;
  }

  /**
   * @return The angular velocity of the arm in radians per second
   */
  public double getVelocityRadPerSec() {
    return ARM_INPUTS.velocityRadPerSec;
  }
}
