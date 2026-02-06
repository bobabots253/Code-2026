package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.util.Units;

public class HoodConstants {
  public static final int sparkMasterHoodCanId = 14;

  public static final double sparkMasterFlyWheelkP = 0.01;
  public static final double sparkMasterFlyWheelkI = 0.00;
  public static final double sparkMasterFlyWheelkD = 0.00;

  public static final double toleranceDeg = 1.0;
  public static final double toleranceRad = Units.degreesToRadians(toleranceDeg);

  public static final double minAngleDeg = 45.0;
  public static final double maxAngleDeg = 85.0;

  public static final double minAngleRad = Math.toRadians(minAngleDeg);
  public static final double maxAngleRad = Math.toRadians(maxAngleDeg);

  // Beg Design team for these values
  public static final double masterPositionConversionFactor =
      1.0; // Motor Rotations -> Hood Radians
  public static final double masterVelocityConversionFactor = 1.0; // Motor RPM -> Hood Rad/Sec

  public static final double hoodOffset = Units.degreesToRadians(1.0); // Radians
}
