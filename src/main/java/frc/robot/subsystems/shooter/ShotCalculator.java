package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  private Pose2d targetLocation;
  private Translation2d blueHubTarget = fieldSetup.blueHubCenter.toTranslation2d();
  private Translation2d redHubTarget = fieldSetup.redHubCenter.toTranslation2d();

  private Pose2d shooterPose = Pose2d.kZero;

  @AutoLogOutput(key = "ShotCalculator/CorrectedTargetPose")
  private Pose2d correctedTargetPose = Pose2d.kZero;

  private Pose2d robotPose = Pose2d.kZero;

  @AutoLogOutput(key = "ShotCalculator/DrivetrainSpeeds")
  ChassisSpeeds drivetrainSpeeds;

  @AutoLogOutput(key = "ShotCalculator/Error")
  private Rotation2d angularError = Rotation2d.kZero;

  ChassisAccelerations drivetrainAccelerations =
      new ChassisAccelerations(0.0, 0.0, 0.0); // Initialize with zero accelerations

  private double distanceToHub2D;
  private double distanceToHub3D;

  Rotation2d fieldToHubAngle;

  Double targetSpeedRPM = 0.0;
  Double targetSpeedMPS = 0.0;
  Double targetAngle = 0.0;

  public ShotCalculator(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
  }

  @Override
  public void periodic() {
    robotPose = swerveSubsystem.getPose(); // Good
    updateTargetByAlliance(); // Good

    correctedTargetPose = targetLocation;

    shooterPose = (robotPose);
    // .plus(ShootOnTheFlyConstants.SHOOTER_TRANSFORM_CENTER);

    drivetrainSpeeds = swerveSubsystem.getChassisSpeeds();
    drivetrainAccelerations = swerveSubsystem.getFieldRelativeChassisAccelerations();

    // Default to uncorrected target pose
    // ShootOnTheFlyCalculator.calculateEffectiveTargetLocation(
    //     shooterPose, targetLocation, drivetrainSpeeds, drivetrainAccelerations, 5, 0.001)

    // https://stackoverflow.com/questions/21483999/using-atan2-to-find-angle-between-two-vectors
    // 12:48 josh is saying that its using the shooter pose which is transformed off the robot pose
    // so check this value by measuring from shooter to center of hub.

    Translation2d fieldToHubTranslation = correctedTargetPose.getTranslation();

    fieldToHubAngle =
        new Rotation2d(
            Math.atan2(
                fieldToHubTranslation.getY() - shooterPose.getTranslation().getY(),
                fieldToHubTranslation.getX() - shooterPose.getTranslation().getX()));

    // 6328: Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();

    distanceToHub2D =
        correctedTargetPose.getTranslation().getDistance(shooterPose.getTranslation());

    distanceToHub3D =
        correctedTargetPose.getTranslation().getDistance(shooterPose.getTranslation());

    targetSpeedRPM = ShootOnTheFlyConstants.FLYWHEEL_RPM_INTERPOLATOR.get(distanceToHub2D);
    targetSpeedMPS = ShootOnTheFlyConstants.FLYWHEEL_VELOCITY_INTERPOLATOR.get(distanceToHub2D);
    targetAngle = ShootOnTheFlyConstants.HOOD_DEGREES_INTERPOLATOR.get(distanceToHub2D);

    angularError = fieldToHubAngle.minus(robotPose.getRotation());

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
    return correctedTargetPose;
  }

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
    return (fieldToHubAngle);
  }

  @AutoLogOutput(key = "ShotCalculator/FieldToHubAngle")
  public Rotation2d getFieldToHubAngle() {
    return (fieldToHubAngle);
  }

  // log this value if you can 12:50
  @AutoLogOutput(key = "ShotCalculator/Distance2D")
  public double getShooterToCorrectTargetPoseDistance() {
    return distanceToHub2D;
  }

  public double getShooterToCorrectTargetPoseDistance3D() {
    return distanceToHub3D;
  }

  // Return This Util in Constants.java
  // Periodic Command Scheduler Overflow Handling
  @AutoLogOutput(key = "ShotCalculator/TargetLocation")
  public Pose2d updateTargetByAlliance() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        targetLocation = new Pose2d(redHubTarget, new Rotation2d());
      } else {
        targetLocation = new Pose2d(blueHubTarget, new Rotation2d());
      }
    } else {
      targetLocation = new Pose2d(blueHubTarget, new Rotation2d());
    }
    return targetLocation;
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("ShotCalculator/TargetLocation", targetLocation);
  }
}
