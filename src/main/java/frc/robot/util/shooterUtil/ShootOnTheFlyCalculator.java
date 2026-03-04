package frc.robot.util.shooterUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.swerveUtil.ChassisAccelerations;
import org.littletonrobotics.junction.Logger;

/**
 * Provides static methods to calculate the effective target position to aim for when shooting on
 * the fly.
 */
public class ShootOnTheFlyCalculator {

  public static final double GRAVITY = 9.81;

  public record ShotSolution(double launchPitchRad, double launchSpeed, double flightTimeSeconds) {}

  /*
   * Uses actual physics to more accurately estimate the time it will take for a projectile to reach a target.
   *  This helps seed the iterative calculation of the effective target location. Discounting air resistance.
   */
  public static double getTimeToShootUsingPhysics(Pose3d robotPose, Pose3d targetPose) { // GRAVITY

    // I AM A BRICK
    // robotPose.minus(targetPose) returns a Transform3d in targetPose's LOCAL coordinates,
    // not field coords. Individual x and y components would be wrong if targetPose is rotated.
    // NOTE: Using the norm of Translation3d subtraction is always correct in field frame.
    Translation3d diff = robotPose.getTranslation().minus(targetPose.getTranslation());
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

    // Flat Shot Edge Case if shooter is above the target and apex is below shooter, heightFromApex
    // could be negative — clamp to zero
    double heightFromApex = Math.max(0.0, hMax - targetPose.getZ());

    double timeFromApexToHub =
        Math.sqrt((2 * heightFromApex) / GRAVITY); // t2 = sqrt( 2 * (h_max - h_f) / g )

    double totalTime = timeToApex + timeFromApexToHub;

    Logger.recordOutput("SOTFCalculator/TimeToShootUsingPhysics", totalTime);
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

      if (Math.abs(newShotTime - shotTime) <= 0.010) {
        shotTime = newShotTime;
        Logger.recordOutput("SOTFCalculator/ConvergedIteration", i + 1);
        break;
      }
      shotTime = newShotTime;
    }

    Logger.recordOutput("SOTFCalculator/EffectiveTargetPose", correctedTargetPose);
    Logger.recordOutput("SOTFCalculator/EffectiveShotTime", shotTime);

    return correctedTargetPose;
  }
}
