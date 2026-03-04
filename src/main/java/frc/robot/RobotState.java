package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {

  /* A record is a class that is used to store an immutable dataset,
   * meaning all vars in a record are private & final
   * this means that once created, a record can't be changed, and
   * so we must create a new record every time we want a new set of data.
   * Records also automatically create getter and setter methods for all contained vars,
   * making it more efficient than creating custom classes.
   * More reading: https://docs.oracle.com/en/java/javase/17/language/records.html
   *
   * ShootingParameters are used for shooting on the fly
   * ShotCoordinator makes sure that the flywheel is spinning fast enough before
   * letting fuel in, to avoid jamming fuel in hood
   */

  public record ShootingParameters(
      Pose2d correctedTargetPose2d,
      double correctedTargetSpeedRPM,
      double correctTargetVelocity,
      double correctedTargetAngle,
      Rotation2d correctTargetRotation,
      double shooterToCorrectTargetPoseDistance,
      double shooterToCorrectTargetPoseDistance3D) {}

  public record ShotCoordinator(
      double flywheelRadPerSec, double flywheelGoalRadPerSec, double shotTolerance) {}

  @AutoLogOutput(key = "RobotState/ShotData")
  private ShootingParameters latestShotData =
      new ShootingParameters(Pose2d.kZero, 0.0, 0.0, 0.0, Rotation2d.kZero, 0.0, 0.0);

  private ShotCoordinator coordinator = new ShotCoordinator(0.0, 50000, 0.0);

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  public void setShootingParameters(ShootingParameters data) {
    this.latestShotData = data;
  }

  public void setShotCoordinator(ShotCoordinator data) {
    this.coordinator = data;
  }

  @AutoLogOutput(key = "RobotState/ShootingParameters")
  public ShootingParameters getCustomShotData() {
    return latestShotData;
  }

  public ShotCoordinator getShotCoordinatorData() {
    return coordinator;
  }
}
