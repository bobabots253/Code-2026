package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.util.Units;

public class HoodConstants {
  public static final int sparkMasterHoodCanId = 14;

  public static final double sparkMasterHoodkP = 20.0; // TO-DO: Retune for faster response
  public static final double sparkMasterHoodkI = 0.00;
  public static final double sparkMasterHoodkD = 0.00;

  public static final double closedLoopAngularTolerance =
      Units.degreesToRadians(1.0); // Change to heuristic value

  // Used as an early warning if the Hood is jammed
  public static final double highCurrentThreshold = 35.0; // Amps

  public static final double minAngleRad = Math.toRadians(0.0); // Hood Deg -> Hood Rads
  public static final double maxAngleRad = Math.toRadians(54.0); // Hood Deg -> Hood Rads

  // Real Max Compression @ 44.5 degrees

  // Gear Ratio Calculations
  // Motor Rotations per Bottom Pinion Rotation
  private static final double kMotorToBottomPulleyReduction = 25.0; // 5:5
  // Bottom Pulley Rotations per Top Pulley Rotation
  private static final double kBottomPulleyToTopPulleyReduction =
      22.0 / 22.0; // 22T/22T, CAD (18:24) is wrong
  // https://cad.onshape.com/documents/4e82030250652400505b5189/w/167427ae385287ae525c40f5/e/b221a965ecb48b8e800aa181
  //   private static final double kPinionHerringboneDiametralPitch = 10.0; // Solved via CAD
  //   private static final double kHoodHerringboneDiametralPitch = 10.0; // Solved via CAD
  //   private static final double kPinionHerringbonePitchRadius =
  //       kPinionHerringbonePitchDiameter / 2; // Unused
  //   private static final double kHoodHarringtonPitchRadius =
  //       kHoodHerringbonePitchDiameter / 2; // Unused
  private static final double kPinionHerringbonePitchDiameter = Units.inchesToMeters(0.9);
  private static final double kHoodHerringbonePitchDiameter = Units.inchesToMeters(14);
  // Because the pinion DP and the Hood DP are 1:1, change in arc length remains constant.
  private static final double kTotalReduction =
      kMotorToBottomPulleyReduction
          * kBottomPulleyToTopPulleyReduction
          * (kHoodHerringbonePitchDiameter / kPinionHerringbonePitchDiameter);

  // THE USER MUST GET THIS RIGHT
  public static final double masterPositionConversionFactor =
      (2.0 * Math.PI) / kTotalReduction; // Motor Rotations -> Hood Radians, Solve
  public static final double masterVelocityConversionFactor =
      (2.0 * Math.PI) / (60.0 * kTotalReduction); // Motor RPM -> Hood Rad/Sec, Solve

  // ------- GOAL CONSTANTS -------- \\

  public static final double layupAngle = Units.degreesToRadians(41.9); // 41.55 @ 1.5m

  public static final double idleAngle = Units.degreesToRadians(38.87);

  public static final double debuggingAngleDown = Units.degreesToRadians(5.0); // Change as needed
  public static final double debuggingAngleUp = Units.degreesToRadians(42); // Change as needed

  public static final double jugglingAngle = Units.degreesToRadians(71.5);

  public static final double debuggingVoltageUP = -1.0;
  public static final double debuggingVoltageDOWN = 1.0;
}
