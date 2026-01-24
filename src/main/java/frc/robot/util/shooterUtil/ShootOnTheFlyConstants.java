package frc.robot.util.shooterUtil;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShootOnTheFlyConstants {

  public static final double shooterHeightOffset = 0.5; // Meters, height of shooter from ground
  public static final Transform3d SHOOTER_TRANSFORM_CENTER =
      new Transform3d(-0.24, 0, 0.5, Rotation3d.kZero);

  public static final InterpolatingDoubleTreeMap FLYWHEEL_RPM_INTERPOLATOR =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap HOOD_DEGREES_INTERPOLATOR =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap FLYWHEEL_VELOCITY_INTERPOLATOR =
      new InterpolatingDoubleTreeMap();
  // Add Robot Transformation
  public static final Transform3d ROBOT_TO_SHOOTER_TRANSFORM =
      new Transform3d(
          new Translation3d(0.5, 0.0, 0.5), // Meters, forward, left, up
          new Rotation3d(0.0, 0.0, 0.0)); // Radians, roll, pitch, yaw

  // Shooting RPMs for Hub Shooting
  // https://www.desmos.com/3d/rbrerrotsx
  // https://www.reca.lc/flywheel?currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=80&flywheelMomentOfInertia=%7B%22s%22%3A4.356%2C%22u%22%3A%22in2%2Albs%22%7D&flywheelRadius=%7B%22s%22%3A1.65%2C%22u%22%3A%22in%22%7D&flywheelRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Reduction%22%7D&flywheelWeight=%7B%22s%22%3A3.2%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%22%7D&motorRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Step-up%22%7D&projectileRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&projectileWeight=%7B%22s%22%3A1%2C%22u%22%3A%22lbs%22%7D&shooterMomentOfInertia=%7B%22s%22%3A2.4%2C%22u%22%3A%22in2%2Albs%22%7D&shooterRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&shooterTargetSpeed=%7B%22s%22%3A4000%2C%22u%22%3A%22rpm%22%7D&shooterWeight=%7B%22s%22%3A1.2%2C%22u%22%3A%22lbs%22%7D&useCustomFlywheelMoi=0&useCustomShooterMoi=0

  static {
    // Key: Distance (meters), Value: Shooter Speed (RPM)
    FLYWHEEL_RPM_INTERPOLATOR.put(1.0, 3400.0); // Touching Hub
    FLYWHEEL_RPM_INTERPOLATOR.put(1.5, 3400.0);
    FLYWHEEL_RPM_INTERPOLATOR.put(2.0, 3400.0);
    FLYWHEEL_RPM_INTERPOLATOR.put(2.5, 3650.0);
    FLYWHEEL_RPM_INTERPOLATOR.put(3.0, 3900.0);
    FLYWHEEL_RPM_INTERPOLATOR.put(3.5, 3970.0);
    FLYWHEEL_RPM_INTERPOLATOR.put(4.0, 4050.0);
    FLYWHEEL_RPM_INTERPOLATOR.put(4.5, 4225.0);

    // Key: Distance (meters), Value: Shooter Velocity (meters per second)
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(1.0, 6.39); // Touching Hub
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(1.5, 6.39);
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(2.0, 6.39);
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(2.5, 6.86);
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(3.0, 7.33);
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(3.5, 7.47);
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(4.0, 7.62);
    FLYWHEEL_VELOCITY_INTERPOLATOR.put(4.5, 7.95);

    // Key: Distance (meters), Value: Hood Angle (DEGREES)
    HOOD_DEGREES_INTERPOLATOR.put(1.0, 82.0); // Touching Hub
    HOOD_DEGREES_INTERPOLATOR.put(1.5, 77.0);
    HOOD_DEGREES_INTERPOLATOR.put(2.0, 70.0);
    HOOD_DEGREES_INTERPOLATOR.put(2.5, 69.0);
    HOOD_DEGREES_INTERPOLATOR.put(3.0, 68.0);
    HOOD_DEGREES_INTERPOLATOR.put(3.5, 64.0);
    HOOD_DEGREES_INTERPOLATOR.put(4.0, 61.0);
    HOOD_DEGREES_INTERPOLATOR.put(4.5, 60.0);
  }
}
