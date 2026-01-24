package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldSetup;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.shooterUtil.ShootOnTheFlyCalculator;
import frc.robot.util.shooterUtil.ShootOnTheFlyConstants;
import frc.robot.util.swerveUtil.ChassisAccelerations;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator extends SubsystemBase {
  private final SwerveSubsystem swerveSubsystem;

  private Pose3d targetLocation;
  private Translation3d blueHubTarget = FieldSetup.blueHubCenter;
  private Translation3d redHubTarget = FieldSetup.redHubCenter;

  private Pose3d shooterPose = Pose3d.kZero;
  private Pose3d correctedTargetPose = Pose3d.kZero;

  public ShotCalculator(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
  }

  @Override
  public void periodic() {
    Pose2d robotPose = swerveSubsystem.getPose();
    updateTargetByAlliance();

    shooterPose = new Pose3d(robotPose).plus(ShootOnTheFlyConstants.SHOOTER_TRANSFORM_CENTER);

    ChassisSpeeds drivetrainSpeeds = swerveSubsystem.getChassisSpeeds();
    ChassisAccelerations drivetrainAccelerations =
        swerveSubsystem.getFieldRelativeChassisAccelerations();

    Pose3d correctedTargetPose =
        ShootOnTheFlyCalculator.calculateEffectiveTargetLocation(
            shooterPose, targetLocation, drivetrainSpeeds, drivetrainAccelerations, 5, 0.005);

    Logger.recordOutput("ShotCalculator/CorrectedTargetPose", correctedTargetPose);
  }

  public Pose3d getCorrectedTargetPose3d() {
    return correctedTargetPose;
  }

  public double getCorrectedTargetSpeedRPM() {
    Double targetSpeedRPM;
    return targetSpeedRPM =
        ShootOnTheFlyConstants.FLYWHEEL_RPM_INTERPOLATOR.get(
            correctedTargetPose.getTranslation().getDistance(shooterPose.getTranslation()));
  }

  public double getCorrectedTargetAngle() {
    Double targetAngle;
    return targetAngle =
        ShootOnTheFlyConstants.HOOD_DEGREES_INTERPOLATOR.get(
            correctedTargetPose.getTranslation().getDistance(shooterPose.getTranslation()));
  }

  public void updateTargetByAlliance() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        targetLocation = new Pose3d(redHubTarget, new Rotation3d());
      } else {
        targetLocation = new Pose3d(blueHubTarget, new Rotation3d());
      }
    }
  }
}
