package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private Pose2d robotPose = Pose2d.kZero;

  private double angleToTargetRad;

  private double distance2D;
  private double distance3D;

  Double targetSpeedRPM;
  Double targetSpeedMPS;
  Double targetAngle;

  public ShotCalculator(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
  }

  @Override
  public void periodic() {
    robotPose = swerveSubsystem.getPose();
    updateTargetByAlliance();

    shooterPose = new Pose3d(robotPose).plus(ShootOnTheFlyConstants.SHOOTER_TRANSFORM_CENTER);

    ChassisSpeeds drivetrainSpeeds = swerveSubsystem.getChassisSpeeds();
    ChassisAccelerations drivetrainAccelerations =
        swerveSubsystem.getFieldRelativeChassisAccelerations();

    Pose3d correctedTargetPose =
        ShootOnTheFlyCalculator.calculateEffectiveTargetLocation(
            shooterPose, targetLocation, drivetrainSpeeds, drivetrainAccelerations, 5, 0.001);

    double deltaX = correctedTargetPose.getX() - robotPose.getX();
    double deltaY = correctedTargetPose.getY() - robotPose.getY();
    double atanParam = deltaY / deltaX;

    if (isRedAlliance()) {
      angleToTargetRad = Math.atan(atanParam) + Math.PI; // Testing: + (Math.PI / 2)
    } else {
      angleToTargetRad = Math.atan(atanParam); // Testing: + (Math.PI / 2)
    }

    distance2D =
        correctedTargetPose
            .toPose2d()
            .getTranslation()
            .getDistance(shooterPose.toPose2d().getTranslation());

    distance3D = correctedTargetPose.getTranslation().getDistance(shooterPose.getTranslation());

    targetSpeedRPM = ShootOnTheFlyConstants.FLYWHEEL_RPM_INTERPOLATOR.get(distance2D);
    targetSpeedMPS = ShootOnTheFlyConstants.FLYWHEEL_VELOCITY_INTERPOLATOR.get(distance2D);
    targetAngle = ShootOnTheFlyConstants.HOOD_DEGREES_INTERPOLATOR.get(distance2D);

    Logger.recordOutput("ShotCalculator/DeltaX", deltaX);
    Logger.recordOutput("ShotCalculator/DeltaY", deltaY);
    Logger.recordOutput("ShotCalculator/AngleToTargetRad", angleToTargetRad);
    Logger.recordOutput("ShotCalculator/CorrectedTargetPose", correctedTargetPose);
    Logger.recordOutput("ShotCalculator/Distance2D", distance2D);
    Logger.recordOutput("ShotCalculator/Distance3D", distance3D);
    Logger.recordOutput("ShotCalculator/CorrectedTargetSpeedRPM", getCorrectedTargetSpeedRPM());
    Logger.recordOutput("ShotCalculator/CorrectTargetVelocity", getCorrectTargetVelocity());
    Logger.recordOutput("ShotCalculator/getCorrectedTargetAngle", getCorrectedTargetAngle());
  }

  public Pose2d getCorrectedTargetPose2d() {
    return correctedTargetPose.toPose2d();
  }

  public double getCorrectedTargetSpeedRPM() {
    return targetSpeedRPM;
  }

  public double getCorrectTargetVelocity() {
    return targetSpeedMPS;
  }

  public double getCorrectedTargetAngle() {
    return targetAngle;
  }

  public Rotation2d getCorrectTargetRotation() {
    return new Rotation2d(angleToTargetRad);
  }

  public double getShooterToCorrectTargetPoseDistance() {
    return distance2D;
  }

  public double getShooterToCorrectTargetPoseDistance3D() {
    return distance3D;
  }

  public boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();

    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
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
