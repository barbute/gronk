// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
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
            Rotation2d.fromDegrees(new LoggedTunableNumber("Arm/CustomSetpointDegree", 0.0).get()));

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

  /** Creates a new Arm. */
  public Arm(ArmIO armIO) {
    ARM_IO = armIO;
  }

  @Override
  public void periodic() {
    ARM_IO.updateInputs(ARM_INPUTS);
    Logger.processInputs("Arm/Inputs", ARM_INPUTS);
  }
}
