package frc.robot.util.shooterUtil;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.fieldSetup;

public class ShootOnTheFlyConstants {

  public static final double FLYWHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

  public static final double shooterHeightOffset = 0.5; // Meters, height of shooter from ground

  public static final double shortMissFlywheelCalibration = 0.3; // meters
  public static final double shortMissHoodCalibration = -5; // degrees

  // ------- Transform Constants -------- \\
  public static final Transform3d SHOOTER_TRANSFORM_CENTER =
      new Transform3d(-0.24, 0, 0.5, Rotation3d.kZero);

  // ------- Hub Shooting Constants -------- \\
  public static final double HUB_INNER_HEIGHT = fieldSetup.blueHubCenter.getZ(); // Same for RED

  // ------- Passing Constants -------- \\
  // Add any height above SHOOTER Z OFFSET to avoid math breaking
  // Relative to DS Perspective aka 6328's callouts
  public static final Translation3d BLUE_PASS_CR_TARGET = new Translation3d(2.00, 1.500, 0.501);

  public static final Translation3d BLUE_PASS_CL_TARGET = new Translation3d(2.00, 6.500, 0.501);

  public static final Translation3d RED_PASS_CL_TARGET = new Translation3d(14.50, 1.500, 0.501);

  public static final Translation3d RED_PASS_CR_TARGET = new Translation3d(14.50, 6.500, 0.501);

  public static final InterpolatingDoubleTreeMap FLYWHEEL_RPM_INTERPOLATOR =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap HOOD_DEGREES_INTERPOLATOR =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap FLYWHEEL_VELOCITY_INTERPOLATOR =
      new InterpolatingDoubleTreeMap();
  // Add Robot Transformation
  public static final Transform3d ROBOT_TO_SHOOTER_TRANSFORM =
      new Transform3d(
          new Translation3d(-0.25, 0.0, 0.5), // Meters, forward, left, up
          new Rotation3d(0.0, 0.0, 0.0)); // Radians, roll, pitch, yaw

  // Shooting RPMs for Hub Shooting
  // https://www.desmos.com/3d/rbrerrotsx
  // https://www.reca.lc/flywheel?currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=80&flywheelMomentOfInertia=%7B%22s%22%3A4.356%2C%22u%22%3A%22in2%2Albs%22%7D&flywheelRadius=%7B%22s%22%3A1.65%2C%22u%22%3A%22in%22%7D&flywheelRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Reduction%22%7D&flywheelWeight=%7B%22s%22%3A3.2%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%22%7D&motorRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Step-up%22%7D&projectileRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&projectileWeight=%7B%22s%22%3A1%2C%22u%22%3A%22lbs%22%7D&shooterMomentOfInertia=%7B%22s%22%3A2.4%2C%22u%22%3A%22in2%2Albs%22%7D&shooterRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&shooterTargetSpeed=%7B%22s%22%3A4000%2C%22u%22%3A%22rpm%22%7D&shooterWeight=%7B%22s%22%3A1.2%2C%22u%22%3A%22lbs%22%7D&useCustomFlywheelMoi=0&useCustomShooterMoi=0
  // Updated:
  // https://www.reca.lc/flywheel?currentLimit=%7B%22s%22%3A50%2C%22u%22%3A%22A%22%7D&efficiency=80&flywheelMomentOfInertia=%7B%22s%22%3A0.0002%2C%22u%22%3A%22in2%2Albs%22%7D&flywheelRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&flywheelRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Reduction%22%7D&flywheelWeight=%7B%22s%22%3A0.0001%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%20Vortex%22%7D&motorRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Step-up%22%7D&projectileRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&projectileWeight=%7B%22s%22%3A1%2C%22u%22%3A%22lbs%22%7D&shooterMomentOfInertia=%7B%22s%22%3A6.4%2C%22u%22%3A%22in2%2Albs%22%7D&shooterRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&shooterTargetSpeed=%7B%22s%22%3A4000%2C%22u%22%3A%22rpm%22%7D&shooterWeight=%7B%22s%22%3A3.2%2C%22u%22%3A%22lbs%22%7D&useCustomFlywheelMoi=0&useCustomShooterMoi=0
  static {
    // Key: Distance (meters), Value: Shooter Speed (RPM)
    // TO-DO: Combine Interpolators using Units.rotationsPerMinuteToRadiansPerSecond()
    // DO NOT USE THIS
    // FLYWHEEL_RPM_INTERPOLATOR.put(1.0, 3400.0); // Touching Hub //3400
    // FLYWHEEL_RPM_INTERPOLATOR.put(1.5, 3400.0); // 3400
    // FLYWHEEL_RPM_INTERPOLATOR.put(2.0, 3400.0); // 3400
    // FLYWHEEL_RPM_INTERPOLATOR.put(2.5, 3650.0); // 3650
    // FLYWHEEL_RPM_INTERPOLATOR.put(3.0, 3900.0); // 3900
    // FLYWHEEL_RPM_INTERPOLATOR.put(3.5, 3970.0); // 3970
    // FLYWHEEL_RPM_INTERPOLATOR.put(4.0, 4050.0); // 4050
    // FLYWHEEL_RPM_INTERPOLATOR.put(4.5, 4225.0); // 4225

    // Key: Distance (meters), Value: Shooter Velocity (rad/sec)
    // USE THIS, BUT FIX FIRST\]
    // Touching Hub
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(1.5, Units.rotationsPerMinuteToRadiansPerSecond(2500));
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(2.0, Units.rotationsPerMinuteToRadiansPerSecond(2800));
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(2.5, Units.rotationsPerMinuteToRadiansPerSecond(2900));
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(3.0, Units.rotationsPerMinuteToRadiansPerSecond(3000));
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(3.5, Units.rotationsPerMinuteToRadiansPerSecond(3000));
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(4.0, Units.rotationsPerMinuteToRadiansPerSecond(3000));
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(4.5, Units.rotationsPerMinuteToRadiansPerSecond(3100));
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(5.0, Units.rotationsPerMinuteToRadiansPerSecond(3200));
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(5.5, Units.rotationsPerMinuteToRadiansPerSecond(3200));
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(6.2, Units.rotationsPerMinuteToRadiansPerSecond(3400));
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(7.0, Units.rotationsPerMinuteToRadiansPerSecond(3500));
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(8.0, Units.rotationsPerMinuteToRadiansPerSecond(3500));

    // Check

    // Key: Distance (meters), Value: Hood Angle (DEGREES)
    // Touching Hub, Using Distance 2d
    HOOD_DEGREES_INTERPOLATOR.put(1.5, 41.98);
    HOOD_DEGREES_INTERPOLATOR.put(2.0, 39.14);
    HOOD_DEGREES_INTERPOLATOR.put(2.5, 41.90);
    HOOD_DEGREES_INTERPOLATOR.put(3.0, 41.90);
    HOOD_DEGREES_INTERPOLATOR.put(3.5, 38.87);
    HOOD_DEGREES_INTERPOLATOR.put(4.0, 32.27);
    HOOD_DEGREES_INTERPOLATOR.put(4.5, 27.50);
    HOOD_DEGREES_INTERPOLATOR.put(5.0, 24.0);
    HOOD_DEGREES_INTERPOLATOR.put(5.5, 16.0);
    HOOD_DEGREES_INTERPOLATOR.put(6.2, 5.0);
    HOOD_DEGREES_INTERPOLATOR.put(7.0, 5.0);
    HOOD_DEGREES_INTERPOLATOR.put(8.0, 5.0);

    // HOOD_DEGREES_INTERPOLATOR.put(4.0, 31.31); // Check
  }
}
