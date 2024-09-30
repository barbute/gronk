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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 PIGEON = new Pigeon2(10, DriveConstants.CANBUS);

  private final StatusSignal<Double> YAW = PIGEON.getYaw();
  private final StatusSignal<Double> YAW_VELOCITY_DEGREES_PER_SEC =
      PIGEON.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    PIGEON.getConfigurator().apply(new Pigeon2Configuration());
    PIGEON.getConfigurator().setYaw(0.0);

    YAW.setUpdateFrequency(100.0);
    YAW_VELOCITY_DEGREES_PER_SEC.setUpdateFrequency(100.0);

    PIGEON.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(YAW, YAW_VELOCITY_DEGREES_PER_SEC).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(YAW.getValueAsDouble());
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(YAW_VELOCITY_DEGREES_PER_SEC.getValueAsDouble());
  }
}
