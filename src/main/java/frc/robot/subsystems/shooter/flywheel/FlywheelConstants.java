package frc.robot.subsystems.shooter.flywheel;

public class FlywheelConstants {
  // CAN Ids
  public static final int flywheelMasterCanID = 15;
  public static final int flywheelFollowerCanID = 16;
  // Current limit
  public static final int flywheelCurrentLimit = 50;

  // Encoder constants
  public static final double flywheelReduction = 1.0;
  public static final boolean flywheelEncoderInverted = false;
  public static final double flywheelEncoderPositionFactor = (2.0 * Math.PI) / flywheelReduction;
  public static final double flywheelEncoderVelocityFactor =
      (2.0 * Math.PI) / 60 / flywheelReduction;
  // Bang Bang and FF constants
  public static final double flywheelTolerance = 1.00;
  public static final double kS = 0.015;
  public static final double kV = 0.015;
  public static final double kA = 0.015;

  // State related constants
  public static final double debuggingVelocity = 0.0;
  public static final double jugglingVelocity = 0.0;

  // SparkIO state related constants
  public static final double ballDetectionThreshold = 10.47;
  public static final double idleTolerance = 10.47;
  public static final double kI_velocity = 0.0;
  public static final double kAntilag = 10.0;

  // PID Constants
  public static final double flywheelVelocitykP = 0.0;
  public static final double flywheelVelocitykI = 0.0;
  public static final double flywheelVelocitykD = 0.0;
}
