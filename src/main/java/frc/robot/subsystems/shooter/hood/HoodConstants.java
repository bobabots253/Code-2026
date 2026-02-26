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

  private static final double kMotorToBottomPulleyReduction = 25.0; // 5:5
  private static final double kBottomPulleyToTopPulleyReduction = 22.0 / 22.0;

  public static final double toleranceDeg = 1.0;
  public static final double toleranceRad = Units.degreesToRadians(toleranceDeg);

  public static final double hoodOffsetDeg = 1.0; // Degrees, Solve

  private static final double kPinionHerringboneDiametralPitch = 10.0; // Solved via CAD
  private static final double kHoodHerringboneDiametralPitch = 10.0; // Solved via CAD
  private static final double kPinionHerringbonePitchDiameter = Units.inchesToMeters(1.4);
  private static final double kHoodHerringbonePitchDiameter = Units.inchesToMeters(14);
  // Because the pinion DP and the Hood DP are 1:1, change in arc lenght remains 1:1.
  private static final double kTotalReduction =
      kMotorToBottomPulleyReduction
          * kBottomPulleyToTopPulleyReduction
          * (kHoodHerringbonePitchDiameter / kPinionHerringbonePitchDiameter);

  public static final double minAngleRad = Units.degreesToRadians(0);
  public static final double maxAngleRad = Units.degreesToRadians(40);
  public static final double jugglingAngle = Units.degreesToRadians(0);
  public static final double debuggingAngle = Units.degreesToRadians(0);

  // Beg Design team for these values
  public static final double masterPositionConversionFactor =
      (2.0 * Math.PI) / kTotalReduction; // Motor Rotations -> Hood Radians, Solve
  public static final double masterVelocityConversionFactor =
      (2.0 * Math.PI) / (60.0 * kTotalReduction); // Motor RPM -> Hood Rad/Sec, Solve

  public static double hoodOffset = Units.degreesToRadians(45); // Radians
}
