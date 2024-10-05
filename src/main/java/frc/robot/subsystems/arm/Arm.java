// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.debugging.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  /** State the arm subsystem is in */
  public enum ArmState {
    /** Fixed setpoint for the subwoofer */
    SUBWOOFER(() -> Rotation2d.fromDegrees(0.0)),
    /** Fixed setpoint for optimal intaking */
    STOW(() -> Rotation2d.fromDegrees(0.0)),
    /** Fixed setpoint for optimal scoring into the amp */
    AMP(() -> Rotation2d.fromDegrees(0.0)),
    /** Obtain the best setpoint to shoot from robot's current position */
    AIM(() -> Rotation2d.fromDegrees(0.0)),
    /** Custom setpoint used for debugging purposes */
    CUSTOM(
        () ->
            Rotation2d.fromDegrees(new LoggedTunableNumber("Arm/CustomSetpointDegree", 0.0).get())),
    /** Hold current position (where-ever the arm was when this state was invoked) */
    HOLD(() -> Rotation2d.fromDegrees(0.0)),
    /** Stop the arm */
    STOPPED(() -> Rotation2d.fromDegrees(0.0));

    // We use suppliers so that dynamically updating setpoints can actually supply their changing
    // pose when we call them
    private Supplier<Rotation2d> setpointSupplier;

    ArmState(Supplier<Rotation2d> positionSetpointSupplier) {
      setpointSupplier = positionSetpointSupplier;
    }

    /**
     * @return The position setpoint associated with that state
     */
    private Rotation2d getStateSetpoint() {
      return this.setpointSupplier.get();
    }
  }

  private final ArmIO ARM_IO;
  private final ArmIOInputsAutoLogged ARM_INPUTS = new ArmIOInputsAutoLogged();

  private ArmState armState = ArmState.STOPPED;
  private Rotation2d armPositionSetpoint = new Rotation2d();

  private TrapezoidProfile.Constraints currentConstraints = ArmConstants.MOTION_PROFILE_CONSTRAINTS;
  private TrapezoidProfile profile;
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

  /** Creates a new Arm. */
  public Arm(ArmIO armIO) {
    ARM_IO = armIO;
  }

  @Override
  public void periodic() {
    ARM_IO.updateInputs(ARM_INPUTS);
    Logger.processInputs("Arm/Inputs", ARM_INPUTS);

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
   * Sets the subsystem's desired state, logic runs in periodic()
   *
   * @param desiredState The desired state
   */
  public void setArmState(ArmState desiredState) {
    armState = desiredState;
  }

  /** Set the feedback controller's gains */
  private void setFeedbackGains(double p, double i, double d) {
    ARM_IO.setFeedbackGains(p, i, d);
  }

  /** Set the trapezoidal motion-profile's constraints */
  private void setMotionConstraints(double maxVelocity, double maxAcceleration) {
    currentConstraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
  }
}
