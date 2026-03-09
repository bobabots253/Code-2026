package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.util.Units;

public class HoodConstants {
  public static final int sparkMasterHoodCanId = 14;

  public static final double sparkMasterHoodkP = 2.0;
  public static final double sparkMasterHoodkI = 0.00;
  public static final double sparkMasterHoodkD = 0.00;

  public static final double toleranceDeg = 1.0; // Do NOT use Degrees when plugging in
  public static final double toleranceRad = Units.degreesToRadians(toleranceDeg);

  public static final double minAngleDeg = 0.0;
  public static final double maxAngleDeg = 54.0;

  public static final double hoodOffsetRad = Units.degreesToRadians(45); // Radians, Solve

  // Gear Ratio Calculations
  // Motor Rotations per Bottom Pinion Rotation
  private static final double kMotorToBottomPulleyReduction = 25.0; // 5:5
  // Bottom Pulley Rotations per Top Pulley Rotation
  private static final double kBottomPulleyToTopPulleyReduction =
      22.0 / 22.0; // 22T/22T, CAD (18:24) is wrong
  // https://cad.onshape.com/documents/4e82030250652400505b5189/w/167427ae385287ae525c40f5/e/b221a965ecb48b8e800aa181
  private static final double kPinionHerringboneDiametralPitch = 10.0; // Solved via CAD
  private static final double kHoodHerringboneDiametralPitch = 10.0; // Solved via CAD
  private static final double kPinionHerringbonePitchDiameter = Units.inchesToMeters(0.9);
  private static final double kHoodHerringbonePitchDiameter = Units.inchesToMeters(14);
  private static final double kPinionHerringbonePitchRadius =
      kPinionHerringbonePitchDiameter / 2; // Unused
  private static final double kHoodHarringtonPitchRadius =
      kHoodHerringbonePitchDiameter / 2; // Unused
  // Because the pinion DP and the Hood DP are 1:1, change in arc lenght remains 1:1.
  private static final double kTotalReduction =
      kMotorToBottomPulleyReduction
          * kBottomPulleyToTopPulleyReduction
          * (kHoodHerringbonePitchDiameter / kPinionHerringbonePitchDiameter);
  // since kaden hasen't pushed make sure these are changed
  public static final double minAngleRad = Math.toRadians(minAngleDeg);
  public static final double maxAngleRad = Math.toRadians(maxAngleDeg);

  // Beg Design team for these values
  public static final double masterPositionConversionFactor =
      (2.0 * Math.PI) / kTotalReduction; // Motor Rotations -> Hood Radians, Solve
  public static final double masterVelocityConversionFactor =
      (2.0 * Math.PI) / (60.0 * kTotalReduction); // Motor RPM -> Hood Rad/Sec, Solve

  public static final double hoodOffset = Units.degreesToRadians(1.0); // Radians

  // Temporary holding for subsystem variables
  public static final double jugglingAngle =
      Units.degreesToRadians(71.5); // Check interpolator for values
  public static final double debuggingAngle = Units.degreesToRadians(35); // Change as needed

  public static final double staticAngle = Units.degreesToRadians(45.25);

  public static final double closedLoopAngularTolerance =
      Units.degreesToRadians(1.0); // Change to heuristic value

  public static final double kDebuggingVoltageUP = -1.0;
  public static final double kDebuggingVoltageDOWN = 1.0;

  // Used as an early warning if the Hood is jammed
  public static final double highCurrentAmps = 40.0;
}
