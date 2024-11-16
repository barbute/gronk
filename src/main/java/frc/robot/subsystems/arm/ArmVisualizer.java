// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.littletonrobotics.junction.Logger;

/** Class to handle graphical visualization of the arm mechanism. */
public class ArmVisualizer {
  private final String LOG_KEY = "Arm/Visualier";

  /** The field that all mechanism ligaments are appended to. Units are in meters */
  private Mechanism2d armVisualField = new Mechanism2d(1.0, 1.0);

  private MechanismRoot2d armPivot = armVisualField.getRoot("Pivot", 0.5, 0.5);
  private MechanismLigament2d armLigament;

  /**
   * Creates a new visualizer
   *
   * @param initialPosition The starting position of the arm
   */
  public ArmVisualizer(Rotation2d initialPosition) {
    armLigament =
        armPivot.append(new MechanismLigament2d("Ligament", 0.4, initialPosition.getRadians()));

    Logger.recordOutput(LOG_KEY, armVisualField);
  }

  /**
   * Updates the arm position in the visualizer to the current position of the mechanism, then
   * pushes value to network tables
   *
   * @param currentArmPosition The current arm position of the mechanism
   */
  public void updateArmPosition(Rotation2d currentArmPosition) {
    if (currentArmPosition != null) {
      armLigament.setAngle(currentArmPosition);
    } else {
      armLigament.setAngle(Rotation2d.fromRadians(0.0));
    }

    Logger.recordOutput(LOG_KEY, armVisualField);
  }
}
