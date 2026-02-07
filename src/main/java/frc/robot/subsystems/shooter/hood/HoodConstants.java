package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.util.Units;

public class HoodConstants {
  public static final int sparkMasterHoodCanId = 14;

  public static final double sparkMasterHoodkP = 0.01;
  public static final double sparkMasterHoodkI = 0.00;
  public static final double sparkMasterHoodkD = 0.00;

  public static final double toleranceDeg = 1.0;
  public static final double toleranceRad = Units.degreesToRadians(toleranceDeg);

  public static final double minAngleDeg = 45.0;
  public static final double maxAngleDeg = 85.0;

  public static final double hoodOffsetDeg = 1.0; // Degrees, Solve

  // Gear Ratio Calculations
  // Motor Rotations per Pinion Rotation
  private static final double kMotorToPinionReduction = 1.0;
  // Pinion Rotations per 1 full 360-degree Pivot Rotation
  private static final double kPinionToPivotReduction = 1.0;
  private static final double kTotalReduction = kMotorToPinionReduction * kPinionToPivotReduction;

  public static final double minAngleRad = Math.toRadians(minAngleDeg);
  public static final double maxAngleRad = Math.toRadians(maxAngleDeg);

  // Beg Design team for these values
  public static final double masterPositionConversionFactor =
      (2.0 * Math.PI) / kTotalReduction; // Motor Rotations -> Hood Radians, Solve
  public static final double masterVelocityConversionFactor =
      (2.0 * Math.PI) / (60.0 * kTotalReduction); // Motor RPM -> Hood Rad/Sec, Solve

  public static final double hoodOffset = Units.degreesToRadians(1.0); // Radians
}
