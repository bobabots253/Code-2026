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

  public static final double GRAVITY = 9.81; // m/s^2

  /**
   * Calculates the time it will take for a projectile to reach the target using basic physics
   * equations. Calculations ignore air resistance, magnus effect, etc. It's dtm. Why does this
   * exist? Used to seed iterative method to find effective target location.
   *
   * <p>Physics Broken Down Here for Sanity: vX = angularVelocity (rad/s) * flywheelRadius (m) t1 =
   * vY / g | aka time to reach apex hMax = shooterHeight + vY^2 / (2g) t2 = sqrt(2 * (hMax -
   * targetZ) / g) | aka time to fall from apex to target (if target is above apex)
   *
   * <p>totalTime = t1 + t2
   *
   * @param shooterPose - 3D pose of the shooter — PLEASE PLEASE PLEASE apply SHOOTER_TRANSFORM to
   *     shooter pose first
   * @param targetPose - 3D pose of some target (probably the hub's inner point)
   */
  public static double getTimeToShootUsingPhysics(Pose3d shooterPose, Pose3d targetPose) {

    // NOTE: Using the norm of Translation3d subtraction is always correct in field frame.
    Translation3d diff = shooterPose.getTranslation().minus(targetPose.getTranslation());
    double xyDistance = new Translation2d(diff.getX(), diff.getY()).getNorm();

    double projectileVelocityRPS =
        ShootOnTheFlyConstants.FLYWHEEL_VELOCITY_INTERPOLATOR.get(xyDistance);
    double projectileVelocityMPS =
        projectileVelocityRPS * ShootOnTheFlyConstants.FLYWHEEL_RADIUS_METERS;

    double hoodAngleRad =
        Math.toRadians(ShootOnTheFlyConstants.HOOD_DEGREES_INTERPOLATOR.get(xyDistance));

    double vY = projectileVelocityMPS * Math.sin(hoodAngleRad);
    double timeToApex = vY / GRAVITY;

    double hMax = ShootOnTheFlyConstants.shooterHeightOffset + ((vY * vY) / (2 * GRAVITY));

    // Flat Shot Edge Case if shooter is above the target and apex is below shooter, heightFromApex
    // could be negative — clamp to zero
    double heightFromApex = Math.max(0.0, hMax - targetPose.getZ());

    double timeFromApexToHub = Math.sqrt((2 * heightFromApex) / GRAVITY);

    double totalTime = timeToApex + timeFromApexToHub;

    Logger.recordOutput("SOTFCalculator/TimeToShootUsingPhysics", totalTime);
    return totalTime;
  }

  /**
   * Iteratively calculates the effective target location to aim for when shooting on the fly,
   * accounting for the robot's velocity and acceleration. The method uses the time to shoot
   * calculated from physics equations to estimate where the target will be when the projectile
   * reaches it, and iteratively refines this estimate until it converges or reaches the maximum
   * number of iterations. (Github Copilot wrote this comment, I just fixed some grammar). But it is
   * truth.
   *
   * <p>Breakdown: Shifts the calculated target position opposite to the robot's current velocity
   * vector. Ie: If the robot is moving towards the hub at lets say 1 m/s towards the red alliance,
   * with a calculated ball time of flight of 0.5 secs, shift the target position 0.5 meter towards
   * the hub. Because changing the virtual target changes the distance,which changes flight time,
   * iterate until convergence.
   *
   * <p>Summary: virtual_target = real_target - (velocity × flightTime)
   *
   * @param shooterPose - 3D shooter pose (robot pose + SHOOTER_TRANSFORM) PLEASE PLEASE PLEASE GET
   *     THIS RIGHT
   * @param targetPose - 3D real target pose (probably the hub's inner point)
   * @param fieldRelRobotVelocity - field-relative chassis speeds
   * @param fieldRelRobotAcceleration - field-relative chassis accelerations
   * @param goalPositionIterations - maximum number of iterations for convergence
   * @param accelerationCompensationFactor - scalar factor to compensate for acceleration
   */
  public static Pose3d calculateEffectiveTargetLocation(
      Pose3d shooterPose,
      Pose3d targetPose,
      ChassisSpeeds fieldRelRobotVelocity,
      ChassisAccelerations fieldRelRobotAcceleration,
      int goalPositionIterations,
      double accelerationCompensationFactor) {

    double shotTime = getTimeToShootUsingPhysics(shooterPose, targetPose);

    Pose3d correctedTargetPose = targetPose;
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

      double newShotTime = getTimeToShootUsingPhysics(shooterPose, correctedTargetPose);

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
