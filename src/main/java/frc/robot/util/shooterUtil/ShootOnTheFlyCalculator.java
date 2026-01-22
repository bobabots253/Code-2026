package frc.robot.util.shooterUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.swerveUtil.ChassisAccelerations;

/**
 * Provides static methods to calculate the effective target position to aim for when shooting on
 * the fly.
 */
public class ShootOnTheFlyCalculator {

  public static final double GRAVITY = 9.81;

  public record ShotSolution(double launchPitchRad, double launchSpeed, double flightTimeSeconds) {}

  /*
   * Generates a rough estimate of the time it will take for a projectile to reach a target. This helps seed the
   * iterative calculation of the effective target location.
   */
  public static double getCrappyTimeToShoot(Pose2d robotPose, Pose3d targetPose) {

    Transform3d diff = new Pose3d(robotPose).minus(targetPose);
    double xyDistance = new Translation2d(diff.getX(), diff.getY()).getNorm();
    double distance = diff.getTranslation().getNorm();

    double projectileVelocity =
        ShootOnTheFlyConstants.FLYWHEEL_VELOCITY_INTERPOLATOR.get(xyDistance);

    double time = distance / projectileVelocity;

    return time;
  }
  /*
   * Uses actual physics to more accurately estimate the time it will take for a projectile to reach a target.
   *  This helps seed the iterative calculation of the effective target location. Discounting air resistance.
   * Input: velocity, angle, startHeight, targetHeight
   */
  public static double getTimeToShootUsingPhysics(Pose3d robotPose, Pose3d targetPose) { // GRAVITY

    Transform3d diff = (robotPose).minus(targetPose);
    double xyDistance = new Translation2d(diff.getX(), diff.getY()).getNorm();

    double projectileVelocity =
        ShootOnTheFlyConstants.FLYWHEEL_VELOCITY_INTERPOLATOR.get(xyDistance);
    double angleRadians =
        Math.toRadians(ShootOnTheFlyConstants.HOOD_DEGREES_INTERPOLATOR.get(xyDistance));

    double vY =
        projectileVelocity
            * Math.sin(angleRadians); // Vertical component of velocity  (v * sin(theta))
    double timeToApex = vY / GRAVITY; // Time to reach the peak // t1 = v * sin(theta) / g

    double hMax = ShootOnTheFlyConstants.shooterHeightOffset + ((vY * vY) / (2 * GRAVITY));
    // h_max = h0 + (v * sin(theta))^2 / (2g)
    double heightFromApex = hMax - targetPose.getZ();

    double timeFromApexToHub =
        Math.sqrt((2 * heightFromApex) / GRAVITY); // t2 = sqrt( 2 * (h_max - h_f) / g )

    double totalTime = timeToApex + timeFromApexToHub;

    return totalTime;
  }

  public static Pose3d calculateEffectiveTargetLocation(
      Pose3d robotPose,
      Pose3d targetPose,
      ChassisSpeeds fieldRelRobotVelocity,
      ChassisAccelerations fieldRelRobotAcceleration,
      double goalPositionIterations,
      double accelerationCompensationFactor) {

    double shotTime = getTimeToShootUsingPhysics(robotPose, targetPose);

    Pose3d correctedTargetPose = new Pose3d();
    for (int i = 0; i < goalPositionIterations; i++) {
      double virtualGoalX =
          targetPose.getX()
              - shotTime
                  * (fieldRelRobotVelocity.vxMetersPerSecond
                      + fieldRelRobotAcceleration.axMetersPerSecondSquared
                          * accelerationCompensationFactor);
      double virtualGoalY =
          targetPose.getY()
              - shotTime
                  * (fieldRelRobotVelocity.vyMetersPerSecond
                      + fieldRelRobotAcceleration.ayMetersPerSecondSquared
                          * accelerationCompensationFactor);

      correctedTargetPose =
          new Pose3d(virtualGoalX, virtualGoalY, targetPose.getZ(), targetPose.getRotation());

      double newShotTime = getTimeToShootUsingPhysics(robotPose, correctedTargetPose);

      shotTime = newShotTime;
      if (Math.abs(newShotTime - shotTime) <= 0.010) {
        break;
      }
    }

    return correctedTargetPose;
  }
}
