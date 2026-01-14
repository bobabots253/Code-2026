package frc.robot.util.shooterUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.swerveUtil.ChassisAccelerations;
import java.util.function.Function;

public final class ShooterCalculator {

  public static final double GRAVITY = 9.81;

  public record ShotSolution(double launchPitchRad, double launchSpeed, double flightTimeSeconds) {}

  public static double getTimeToShoot(
      Pose2d robotPose,
      Pose3d targetPose,
      Function<Double, Double> xyDistanceToProjectileVelocity) {
    Transform3d diff = new Pose3d(robotPose).minus(targetPose);
    double xyDistance = new Translation2d(diff.getX(), diff.getY()).getNorm();
    double distance = diff.getTranslation().getNorm();
    double projectileVelocity = xyDistanceToProjectileVelocity.apply(xyDistance);
    double time = distance / projectileVelocity;
    return time;
  }

  public static double getTimeToShoot(
      Pose3d shooterPose, Pose3d targetPose, double launchSpeed, double launchPitchRad) {
    Translation3d s = shooterPose.getTranslation();
    Translation3d t = targetPose.getTranslation();

    double dx = t.getX() - s.getX();
    double dy = t.getY() - s.getY();

    double horizontalDist = Math.hypot(dx, dy);
    double vHoriz = launchSpeed * Math.cos(launchPitchRad);

    if (vHoriz <= 1e-6) {
      throw new IllegalArgumentException("Horizontal velocity too small");
    }

    return horizontalDist / vHoriz;
  }

  public static Pose3d calculateEffectiveTargetLocation(
      Pose2d robotPose,
      Pose3d targetPose,
      ChassisSpeeds fieldRelRobotVelocity,
      ChassisAccelerations fieldRelRobotAcceleration,
      Function<Double, Double> xyDistanceToProjectileVelocity,
      double goalPositionIterations,
      double accelerationCompensationFactor) {

    double shotTime = getTimeToShoot(robotPose, targetPose, xyDistanceToProjectileVelocity);

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

      double newShotTime =
          getTimeToShoot(robotPose, correctedTargetPose, xyDistanceToProjectileVelocity);

      shotTime = newShotTime;
      if (Math.abs(newShotTime - shotTime) <= 0.010) {
        break;
      }
    }

    return correctedTargetPose;
  }

  public record InterceptSolution(
      Pose3d effectiveTargetPose,
      double launchPitchRad,
      double launchSpeed,
      double flightTime,
      double requiredYaw) {}

  public static InterceptSolution solveShootOnTheFly(
      Pose3d shooterPose,
      Pose3d targetPose,
      ChassisSpeeds fieldRelRobotVelocity,
      ChassisAccelerations fieldRelRobotAcceleration,
      double targetSpeedRps,
      int maxIterations,
      double timeTolerance) {

    ShotSolution sol =
        ShooterCalculator.solveBallisticWithSpeed(shooterPose, targetPose, targetSpeedRps);

    double t = sol.flightTimeSeconds();
    Pose3d effectiveTarget = targetPose;

    for (int i = 0; i < maxIterations; i++) {

      double dx = fieldRelRobotVelocity.vxMetersPerSecond * t;
      // + 0.5 * fieldRelRobotAcceleration.axMetersPerSecondSquared * t * t;

      double dy = fieldRelRobotVelocity.vyMetersPerSecond * t;
      // + 0.5 * fieldRelRobotAcceleration.ayMetersPerSecondSquared * t * t;

      effectiveTarget =
          new Pose3d(
              targetPose.getX() - dx,
              targetPose.getY() - dy,
              targetPose.getZ(),
              targetPose.getRotation());

      ShotSolution newSol =
          ShooterCalculator.solveBallisticWithSpeed(shooterPose, effectiveTarget, targetSpeedRps);

      if (Math.abs(newSol.flightTimeSeconds() - t) < timeTolerance) {
        return new InterceptSolution(
            effectiveTarget,
            newSol.launchPitchRad(),
            newSol.launchSpeed(),
            newSol.flightTimeSeconds(),
            0);
      }

      sol = newSol;
      t = newSol.flightTimeSeconds();
    }

    return new InterceptSolution(
        effectiveTarget, sol.launchPitchRad(), sol.launchSpeed(), sol.flightTimeSeconds(), 0);
  }

  public static ShotSolution solveBallisticWithSpeed(
      Pose3d shooterPose, Pose3d targetPose, double launchSpeed) {

    Translation3d s = shooterPose.getTranslation();
    Translation3d t = targetPose.getTranslation();

    double dx = t.getX() - s.getX();
    double dy = t.getY() - s.getY();
    double dz = t.getZ() - s.getZ();

    double d = Math.hypot(dx, dy);
    if (d < 1e-9) {
      throw new IllegalArgumentException("Horizontal distance too small");
    }

    double v2 = launchSpeed * launchSpeed;
    double g = GRAVITY;

    double discriminant = v2 * v2 - g * (g * d * d + 2.0 * dz * v2);
    if (discriminant < 0) {
      return new ShotSolution(0, 0, 0);
    }

    // LOW-ARC solution (use +Math.sqrt(...) for high arc)
    double tanTheta = (v2 + Math.sqrt(discriminant)) / (g * d);

    double launchPitch = Math.atan(tanTheta);

    double vHoriz = launchSpeed * Math.cos(launchPitch);
    double time = d / vHoriz;

    return new ShotSolution(launchPitch, launchSpeed, time);
  }
}
