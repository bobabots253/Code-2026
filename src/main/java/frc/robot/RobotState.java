package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  public record ShootingParameters(
      Pose2d correctedTargetPose2d,
      double correctedTargetSpeedRPM,
      double correctTargetVelocity,
      double correctedTargetAngle,
      Rotation2d correctTargetRotation,
      double shooterToCorrectTargetPoseDistance,
      double shooterToCorrectTargetPoseDistance3D) {}

  private ShootingParameters latestShotData =
      new ShootingParameters(Pose2d.kZero, 0.0, 0.0, 0.0, Rotation2d.kZero, 0.0, 0.0);

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  public void setShootingParameters(ShootingParameters data) {
    this.latestShotData = data;
  }

  @AutoLogOutput(key = "RobotState/ShootingParameters")
  public ShootingParameters getCustomShotData() {
    return latestShotData;
  }
}
