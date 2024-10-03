// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private final double LOOP_PERIOD_SEC = 0.02;

  private final SingleJointedArmSim ARM =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(2), ArmConstants.GEAR_RATIO, 0.0, 0.0, 0.0, 0.0, false, 0.0);

  public ArmIOSim() {}
}
