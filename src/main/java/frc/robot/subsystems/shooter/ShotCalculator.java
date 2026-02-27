package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import frc.robot.fieldSetup;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FullSubsystem;
import frc.robot.util.shooterUtil.ShootOnTheFlyConstants;
import frc.robot.util.swerveUtil.ChassisAccelerations;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator extends FullSubsystem {
  private final SwerveSubsystem swerveSubsystem;

  private Pose3d targetLocation;
  private Translation3d blueHubTarget = fieldSetup.blueHubCenter;
  private Translation3d redHubTarget = fieldSetup.redHubCenter;

  private Pose3d shooterPose = Pose3d.kZero;

  @AutoLogOutput(key = "ShotCalculator/CorrectedTargetPose")
  private Pose3d correctedTargetPose = Pose3d.kZero;

  private Pose2d robotPose = Pose2d.kZero;

  @AutoLogOutput(key = "ShotCalculator/DrivetrainSpeeds")
  ChassisSpeeds drivetrainSpeeds;

  ChassisAccelerations drivetrainAccelerations =
      new ChassisAccelerations(0.0, 0.0, 0.0); // Initialize with zero accelerations

  private double angleToTargetRad;

  private double distance2D;
  private double distance3D;

  Rotation2d fieldToHubAngle;

  double deltaX;
  double deltaY;

  Double targetSpeedRPM;
  Double targetSpeedMPS;
  Double targetAngle;

  public ShotCalculator(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
  }

  @Override
  public void periodic() {
    robotPose = swerveSubsystem.getPose(); // Good
    updateTargetByAlliance(); // Good

    shooterPose = new Pose3d(robotPose).plus(ShootOnTheFlyConstants.SHOOTER_TRANSFORM_CENTER);
    drivetrainSpeeds = swerveSubsystem.getChassisSpeeds();
    drivetrainAccelerations = swerveSubsystem.getFieldRelativeChassisAccelerations();

    correctedTargetPose = targetLocation; // Default to uncorrected target pose
    // ShootOnTheFlyCalculator.calculateEffectiveTargetLocation(
    //     shooterPose, targetLocation, drivetrainSpeeds, drivetrainAccelerations, 5, 0.001);

    deltaX = correctedTargetPose.getX() - robotPose.getX();
    deltaY = correctedTargetPose.getY() - robotPose.getY();

    // https://stackoverflow.com/questions/21483999/using-atan2-to-find-angle-between-two-vectors
    // I swear atan2 is supposed to be right
    // angleToTargetRad = Math.atan2(deltaY, deltaX);

    Translation2d fieldToHubAngleTranslation = targetLocation.toPose2d().getTranslation();
    fieldToHubAngle =
        (fieldToHubAngleTranslation)
            .minus(robotPose.getTranslation())
            .getAngle(); // Literally Just Atan2

    // double atanParam = deltaY / deltaX;

    // if (isRedAlliance()) {
    //   angleToTargetRad = Math.atan(atanParam) + Math.PI; // Testing: + (Math.PI / 2)
    // } else {
    //   angleToTargetRad = Math.atan(atanParam); // Testing: + (Math.PI / 2)
    // }

    distance2D =
        correctedTargetPose
            .toPose2d()
            .getTranslation()
            .getDistance(shooterPose.toPose2d().getTranslation());

    distance3D = correctedTargetPose.getTranslation().getDistance(shooterPose.getTranslation());

    targetSpeedRPM = ShootOnTheFlyConstants.FLYWHEEL_RPM_INTERPOLATOR.get(distance2D);
    targetSpeedMPS = ShootOnTheFlyConstants.FLYWHEEL_VELOCITY_INTERPOLATOR.get(distance2D);
    targetAngle = ShootOnTheFlyConstants.HOOD_DEGREES_INTERPOLATOR.get(distance2D);

    // Publish Shot Calculation Data to Robot State
    RobotState.getInstance()
        .setShootingParameters(
            new RobotState.ShootingParameters(
                getCorrectedTargetPose2d(),
                getCorrectedTargetSpeedRPM(),
                getCorrectTargetVelocity(),
                getCorrectedTargetAngle(),
                getCorrectTargetRotation(),
                getShooterToCorrectTargetPoseDistance(),
                getShooterToCorrectTargetPoseDistance3D()));
  }

  /*
   * Note: the rotation of this Pose2d is meaningless
   */
  public Pose2d getCorrectedTargetPose2d() {
    return correctedTargetPose.toPose2d();
  }

  @AutoLogOutput(key = "ShotCalculator/CorrectedTargetSpeedRPM")
  public double getCorrectedTargetSpeedRPM() {
    return targetSpeedRPM;
  }

  @AutoLogOutput(key = "ShotCalculator/CorrectTargetVelocity")
  public double getCorrectTargetVelocity() {
    return targetSpeedMPS;
  }

  @AutoLogOutput(key = "ShotCalculator/CorrectedTargetAngle")
  public double getCorrectedTargetAngle() {
    return targetAngle;
  }

  @AutoLogOutput(key = "ShotCalculator/CorrectTargetRotation")
  /**
   * The heading the robot needs to face to aim at the target location. Calculated as atan2(targetY
   * - robotY, targetX - robotX). DO NOT DO THE FLIPPING HERE
   */
  public Rotation2d getCorrectTargetRotation() {
    return new Rotation2d(angleToTargetRad);
  }

  @AutoLogOutput(key = "ShotCalculator/FieldToHubAngle")
  public Rotation2d getFieldToHubAngle() {
    return fieldToHubAngle;
  }

  @AutoLogOutput(key = "ShotCalculator/Distance2D")
  public double getShooterToCorrectTargetPoseDistance() {
    return distance2D;
  }

  @AutoLogOutput(key = "ShotCalculator/Distance3D")
  public double getShooterToCorrectTargetPoseDistance3D() {
    return distance3D;
  }

  public boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();

    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  // Return This Util in Constants.java
  // Periodic Command Scheduler Overflow Handling
  @AutoLogOutput(key = "ShotCalculator/TargetLocation")
  public Pose3d updateTargetByAlliance() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        targetLocation = new Pose3d(redHubTarget, new Rotation3d());
      } else {
        targetLocation = new Pose3d(blueHubTarget, new Rotation3d());
      }
    } else {
      targetLocation = new Pose3d(blueHubTarget, new Rotation3d());
    }
    return targetLocation;
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("ShotCalculator/TargetLocation", targetLocation);
  }
}
