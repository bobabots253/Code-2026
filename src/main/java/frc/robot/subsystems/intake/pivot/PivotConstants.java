package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.util.Units;

public class PivotConstants {
  public static final int sparkMasterPivotCanId = 9;
  public static final int pivotCurrentLimit = 50; // amps
  public static final double pivotkP = 0.8;
  // big distance, big speed; small distance, small speed
  // start small 0.001, then step up slowly
  public static final double pivotkI = 0.0; // forgettable
  public static final double pivotkD = 0.0;
  // arriving soon, slow down
  public static final boolean pivotEncoderInverted = false;
  public static final double pivotEncoderPositionFactor = 0.0;
  public static final double kPlanetaryReduction = 15;
  public static final double kChainReduction = 3;
  public static final double kTotalReduction = kPlanetaryReduction * kChainReduction; // 45:1

  public static final double pivotPositionConversionFactor =
      (2 * Math.PI) / kTotalReduction; // motor rotations -> radians
  public static final double pivotVelocityConversionFactor =
      (2 * Math.PI) / (60 * kTotalReduction); // motor rotations per second -> radians per second

  // Temporary Values (Confirm Functionality)
  public static final double stowAngle = 0.0;
  public static final double deployedAngle = -2.19; // Not full stow prototype
  public static final double halfDeployedAngle = deployedAngle * 0.2; // was 0.25
  public static final double jugglingAngle = Units.degreesToRadians(0.0);
  public static final double debuggingAngle = Units.degreesToRadians(0.0); // Change as needed

  // Used as an early warning if the Pivot is jammed
  public static final double highCurrentAmps = 50.0;
}
