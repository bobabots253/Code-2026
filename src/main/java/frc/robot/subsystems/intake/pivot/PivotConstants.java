package frc.robot.subsystems.intake.pivot;

public class PivotConstants {
  public static final int pivotCanID = 9;
  public static final int pivotCurrentLimit = 50; // amps
  public static final double pivotkP = 0.0; 
  // big distance, big speed; small distance, small speed
  // start small 0.001, then step up slowly
  public static final double pivotkI = 0.0; // forgettable
  public static final double pivotkD = 0.0;
  // arriving soon, slow down
  public static final boolean pivotEncoderInverted = false;
  public static final double pivotEncoderPositionFactor = 0.0;
  public static final double kPlanetaryReduction = 1; // steal values from design
  public static final double kChainReduction = 1;
  public static final double kTotalReduction = kPlanetaryReduction * kChainReduction;

  public static final double pivotPositionConversionFactor = 
    (2 * Math.PI) / kTotalReduction; // motor rotations -> radians
  public static final double pivotVelocityConversionFactor = 
    (2 * Math.PI) / (60 * kTotalReduction); // motor rotations per second -> radians per second
}