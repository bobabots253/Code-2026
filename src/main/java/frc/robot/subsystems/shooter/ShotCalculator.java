package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import frc.robot.fieldSetup;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FullSubsystem;
import frc.robot.util.shooterUtil.ShootOnTheFlyCalculator;
import frc.robot.util.shooterUtil.ShootOnTheFlyConstants;
import frc.robot.util.swerveUtil.ChassisAccelerations;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator extends FullSubsystem {

  public enum CompensationMode {
    // Aim directly at hub. No robot motion factored in.
    STATIC,
    // Shift virtual target opposite to robot velocity × flight time.
    VELOCITY,
    // Shift virtual target opposite to robot velocity + acceleration × flight time.
    VELOCITY_AND_ACCELERATION
  }

  private CompensationMode currentCompensationMode = CompensationMode.STATIC;

  // Tunable Constants for SOTF Calculations
  private static final double ACCELERATION_COMPENSATION_FACTOR = 0.001;
  private static final int SOTF_ITERATIONS = 5;

  private final SwerveSubsystem swerveSubsystem;

  private Pose2d targetLocation = Pose2d.kZero;
  private Translation2d blueHubTarget = fieldSetup.blueHubCenter.toTranslation2d();
  private Translation2d redHubTarget = fieldSetup.redHubCenter.toTranslation2d();

  @AutoLogOutput(key = "ShotCalculator/CorrectedTargetPose")
  private Pose3d correctedTargetPose3d = Pose3d.kZero;

  private Pose3d shooterPose3d = Pose3d.kZero;

  private Pose2d robotPose = Pose2d.kZero;

  @AutoLogOutput(key = "ShotCalculator/DrivetrainSpeeds")
  private ChassisSpeeds drivetrainSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  @AutoLogOutput(key = "ShotCalculator/AngularError")
  private Rotation2d angularError = Rotation2d.kZero;

  ChassisAccelerations drivetrainAccelerations =
      new ChassisAccelerations(0.0, 0.0, 0.0); // Initialize with zero accelerations

  private double distanceToHub2D = 0.0;
  private double distanceToHub3D = 0.0;

  private Rotation2d fieldToHubAngle = Rotation2d.kZero;

  private double targetSpeedRPM = 0.0;
  private double targetSpeedMPS = 0.0;
  private double targetAngleDeg = 0.0;

  public ShotCalculator(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
  }

  public void setCompensationMode(CompensationMode mode) {
    this.currentCompensationMode = mode;
  }

  @AutoLogOutput(key = "ShotCalculator/CompensationMode")
  public String getCompensationModeString() {
    return currentCompensationMode.name();
  }

  @Override
  public void periodic() {
    robotPose = swerveSubsystem.getPose(); // Good
    updateTargetByAlliance(); // Good

    drivetrainSpeeds = swerveSubsystem.getChassisSpeeds();
    drivetrainAccelerations = swerveSubsystem.getFieldRelativeChassisAccelerations();

    shooterPose3d = new Pose3d(robotPose).plus(ShootOnTheFlyConstants.SHOOTER_TRANSFORM_CENTER);

    Pose3d rawTargetPose3d =
        new Pose3d(
            targetLocation.getX(),
            targetLocation.getY(),
            fieldSetup.blueHubCenter.getZ(), // CHECK THIS VALUE, hub inner opening (meters)
            Pose3d.kZero.getRotation());

    switch (currentCompensationMode) {
      case STATIC:
        // Default to uncorrected target pose
        correctedTargetPose3d = rawTargetPose3d;
        break;
      case VELOCITY:
        correctedTargetPose3d =
            ShootOnTheFlyCalculator.calculateEffectiveTargetLocation(
                shooterPose3d,
                rawTargetPose3d,
                drivetrainSpeeds,
                drivetrainAccelerations,
                SOTF_ITERATIONS,
                0.0);
        break;
      case VELOCITY_AND_ACCELERATION:
        correctedTargetPose3d =
            ShootOnTheFlyCalculator.calculateEffectiveTargetLocation(
                shooterPose3d,
                rawTargetPose3d,
                drivetrainSpeeds,
                drivetrainAccelerations,
                SOTF_ITERATIONS,
                ACCELERATION_COMPENSATION_FACTOR);
        break;
    }

    // https://stackoverflow.com/questions/21483999/using-atan2-to-find-angle-between-two-vectors

    Translation2d correctedTargetXYCoords =
        new Translation2d(correctedTargetPose3d.getX(), correctedTargetPose3d.getY());
    Translation2d shooterXYCoords = shooterPose3d.toPose2d().getTranslation();

    fieldToHubAngle = correctedTargetXYCoords.minus(shooterXYCoords).getAngle();
    // Functionally equivalent to atan2(dy, dx) but avoids the manual subtraction + easier to read

    distanceToHub2D = shooterXYCoords.getDistance(correctedTargetXYCoords);
    distanceToHub3D =
        shooterPose3d.getTranslation().getDistance(correctedTargetPose3d.getTranslation());

    targetSpeedRPM = ShootOnTheFlyConstants.FLYWHEEL_RPM_INTERPOLATOR.get(distanceToHub2D);
    targetSpeedMPS = ShootOnTheFlyConstants.FLYWHEEL_VELOCITY_INTERPOLATOR.get(distanceToHub2D);
    targetAngleDeg = ShootOnTheFlyConstants.HOOD_DEGREES_INTERPOLATOR.get(distanceToHub2D);

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
    return new Pose2d(correctedTargetPose3d.getX(), correctedTargetPose3d.getY(), new Rotation2d());
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
    return targetAngleDeg;
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
