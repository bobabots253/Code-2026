package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.util.Units;

public class HoodConstants {
  public static final int sparkMasterHoodCanId = 14;
  public static final int hoodCurrentLimit = 45;
  public static final double sparkHoodkP = 0.01;
  public static final double sparkHoodkI = 0.00;
  public static final double sparkHoodkD = 0.00;

  public static final double sparkHoodProfiledkP = 0.01;
  public static final double sparkHoodProfiledkI = 0.00;
  public static final double sparkHoodProfiledkD = 0.00;
  public static final double sparkHoodProfiledkS = 0.00;
  public static final double sparkHoodProfiledkV = 0.00;
  public static final double sparkHoodMaxAccel = 0.0;
  public static final double sparkHoodMaxVelocity = 0.0;


  public static final double toleranceDeg = 1.0;
  public static final double toleranceRad = Units.degreesToRadians(toleranceDeg);


  public static final double hoodOffsetDeg = 1.0; // Degrees, Solve

  // Gear Ratio Calculations
  // Motor Rotations per Pinion Rotation
  private static final double kMotorToPinionReduction = 1.0;
  // Pinion Rotations per 1 full 360-degree Pivot Rotation
  private static final double kPinionToPivotReduction = 1.0;
  static final double kTotalReduction = kMotorToPinionReduction * kPinionToPivotReduction;

  public static final double minAngleRad = Units.degreesToRadians(45);
  public static final double maxAngleRad = Units.degreesToRadians(45);

  // Beg Design team for these values
  public static final double masterPositionConversionFactor =
      (2.0 * Math.PI) / kTotalReduction; // Motor Rotations -> Hood Radians, Solve
  public static final double masterVelocityConversionFactor =
      (2.0 * Math.PI) / (60.0 * kTotalReduction); // Motor RPM -> Hood Rad/Sec, Solve

  public static final double hoodOffset = Units.degreesToRadians(1.0); // Radians
}
