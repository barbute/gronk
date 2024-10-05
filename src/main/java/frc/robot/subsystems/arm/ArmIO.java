// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction interface */
public interface ArmIO {
  /** Set of logged inputs from the subsystem */
  @AutoLog
  public class ArmIOInputs {
    public boolean leaderMotorConnected = true;
    public boolean followerMotorConnected = true;
    public boolean absoluteEncoderConnected = true;

    public Rotation2d position = new Rotation2d();
    public Rotation2d absoluteEncoderPosition = new Rotation2d();
    public double velocityRadPerSec = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmp = new double[] {};
    public double[] torqueCurrentAmp = new double[] {};
    public double[] temperatureCelsius = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run the motors at the specified voltage. */
  public default void setArmVoltage(double voltage) {}

  /** Run to the setpoint */
  public default void setArmPositionSetpoint(Rotation2d setpoint, double feedforward) {}

  /** Run the motors at the specified current */
  public default void setArmCurrent(double currentAmp) {}

  /** Set the on-board PID gains for the motors */
  public default void setFeedbackGains(double p, double i, double d) {}

  /** Enable or disable brake mode on the motors */
  public default void setBrakeMode(boolean enable) {}

  /** Set motor to cease motion */
  public default void stop() {}
}
