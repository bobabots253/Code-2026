package frc.robot.subsystems.shooter.flywheel;

import frc.robot.fieldSetup;

public class FlywheelConstants {
  public static final int flywheelMasterCanID = 15;
  public static final int flywheelFollowerCanID = 16;
  public static final int flywheelCurrentLimit = 50;

  public static final double flywheelVelocitykP = 0.0;
  public static final double flywheelVelocitykI = 0.0;
  public static final double flywheelVelocitykD = 0.0;
  public static final double flywheelPIDMaxOutput = 0.0;
  public static final double flywheelPIDMinOutput = 0.0;

  public static final double flywheelReduction = 1.0;
  public static final boolean flywheelEncoderInverted = false;
  public static final double flywheelEncoderPositionFactor = (2.0 * Math.PI) / flywheelReduction;
  public static final double flywheelEncoderVelocityFactor =
      (2.0 * Math.PI) / 60 / flywheelReduction;
  public static final double flywheelTolerance = 1.00;
}
