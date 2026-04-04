package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
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

  public enum ShotMode {
    HUB,
    PASS
  }

  public enum PassSide {
    CLOSE_LEFT, // high Y — near left bump
    CLOSE_RIGHT // low  Y — near right bump
  }

  public enum CompensationMode {
    // Aim directly at hub. No robot motion factored in.
    STATIC,
    // Shift virtual target opposite to robot velocity × flight time.
    VELOCITY,
    // Shift virtual target opposite to robot velocity + acceleration × flight time.
    VELOCITY_AND_ACCELERATION
  }

  // Internal mode tracking
  private ShotMode currentShotMode = ShotMode.HUB; // Default Non-Null
  private PassSide currentPassSide = PassSide.CLOSE_RIGHT; // Default Non-Null
  private CompensationMode currentCompensationMode = CompensationMode.STATIC; // Keep for Week 2

  // Tunable Constants for SOTF Calculations
  private static final double ACCELERATION_COMPENSATION_FACTOR = 0.001;
  private static final int SOTF_ITERATIONS = 5;

  private final SwerveSubsystem swerveSubsystem;

  private Pose2d robotPose = Pose2d.kZero;
  private Pose3d shooterPose3d = Pose3d.kZero;
  private Pose3d rawTargetPose3d = Pose3d.kZero; // selected target before compensation
  private Pose3d correctedTargetPose3d = Pose3d.kZero; // selected target after compensation

  private Pose2d hubTargetLocation = Pose2d.kZero;
  private Translation3d passTargetLocation = ShootOnTheFlyConstants.BLUE_PASS_CL_TARGET;

  private Translation2d blueHubTarget = fieldSetup.blueHubCenter.toTranslation2d();
  private Translation2d redHubTarget = fieldSetup.redHubCenter.toTranslation2d();

  private ChassisSpeeds drivetrainSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  ChassisAccelerations drivetrainAccelerations =
      new ChassisAccelerations(0.0, 0.0, 0.0); // Initialize with zero accelerations

  private Rotation2d fieldToTargetAngle = Rotation2d.kZero; // Used by both Shot Modes
  private Rotation2d angularError = Rotation2d.kZero;

  private double distanceToTarget2D = 0.0;
  private double distanceToTarget3D = 0.0;

  private double targetSpeedRadPerSec = 0.0;
  private double targetAngleDeg = 0.0;

  public ShotCalculator(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
  }

  // ------- Setter Methods -------- \\

  public void setShotMode(ShotMode mode) {
    this.currentShotMode = mode;
  }

  public Command toggleShotMode(ShotMode toggleRequest) {
    return runOnce(() -> setShotMode(toggleRequest));
  }

  /**
   * Only affects outputs when active mode is ShotMode.PASS.
   *
   * @param side - relative to DS perspective
   */
  public void setPassSide(PassSide side) {
    this.currentPassSide = side;
  }

  public Command togglePassMode(PassSide toggleRequest) {
    return runOnce(() -> setPassSide(toggleRequest));
  }

  // Use this in RobotContainer to change to passing in one call
  public void setPass(PassSide side) {
    this.currentShotMode = ShotMode.PASS;
    this.currentPassSide = side;
  }

  public Command togglePass(PassSide toggleRequest) {
    return runOnce(() -> setPass(toggleRequest));
  }

  public void setCompensationMode(CompensationMode mode) {
    this.currentCompensationMode = mode;
  }

  @AutoLogOutput(key = "ShotCalculator/ShotMode")
  public String getShotMode() {
    return currentShotMode.name();
  }

  @AutoLogOutput(key = "ShotCalculator/CompensationMode")
  public String getCompensationModeString() {
    return currentCompensationMode.name();
  }

  @Override
  public void periodic() {

    // Update local odometry variable at the start of the periodic block
    robotPose = swerveSubsystem.getPose(); // Good
    drivetrainSpeeds = swerveSubsystem.getChassisSpeeds();
    drivetrainAccelerations = swerveSubsystem.getFieldRelativeChassisAccelerations();

    // Update Shooter Pose by transforming robot odometry by offsets
    shooterPose3d = new Pose3d(robotPose).plus(ShootOnTheFlyConstants.SHOOTER_TRANSFORM_CENTER);

    // Taarget is both ShotMode and Alliance
    updateTargetByAlliance(); // Good
    switch (currentShotMode) {
      case HUB:
        rawTargetPose3d =
            new Pose3d(
                hubTargetLocation.getX(),
                hubTargetLocation.getY(),
                ShootOnTheFlyConstants.HUB_INNER_HEIGHT,
                Pose3d.kZero.getRotation());
        break;

      case PASS:
        rawTargetPose3d =
            new Pose3d(
                passTargetLocation.getX(),
                passTargetLocation.getY(),
                passTargetLocation.getZ(),
                Pose3d.kZero.getRotation());
        break;
    }

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

    fieldToTargetAngle = correctedTargetXYCoords.minus(shooterXYCoords).getAngle();
    // Functionally equivalent to atan2(dy, dx) but avoids the manual subtraction + easier to read

    distanceToTarget2D =
        shooterXYCoords.getDistance(correctedTargetXYCoords)
            + ShootOnTheFlyConstants.shortMissFlywheelCalibration;
    distanceToTarget3D =
        shooterPose3d.getTranslation().getDistance(correctedTargetPose3d.getTranslation());

    targetSpeedRadPerSec =
        ShootOnTheFlyConstants.FLYWHEEL_VELOCITY_INTERPOLATOR.get(distanceToTarget2D);
    targetAngleDeg =
        ShootOnTheFlyConstants.HOOD_DEGREES_INTERPOLATOR.get(distanceToTarget2D)
            + ShootOnTheFlyConstants.shortMissHoodCalibration;

    angularError = fieldToTargetAngle.minus(robotPose.getRotation());

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

  @AutoLogOutput(key = "ShotCalculator/CorrectTargetRotation")
  public Rotation2d getCorrectTargetRotation() {
    return fieldToTargetAngle;
  }

  public Pose2d getCorrectedTargetPose2d() {
    return new Pose2d(correctedTargetPose3d.getX(), correctedTargetPose3d.getY(), new Rotation2d());
  }

  // Placeholder
  public double getCorrectedTargetSpeedRPM() {
    return 0;
  }

  @AutoLogOutput(key = "ShotCalculator/CorrectTargetVelocity")
  public double getCorrectTargetVelocity() {
    return targetSpeedRadPerSec;
  }

  @AutoLogOutput(key = "ShotCalculator/CorrectedTargetAngle")
  public double getCorrectedTargetAngle() {
    return targetAngleDeg;
  }

  @AutoLogOutput(key = "ShotCalculator/Distance2D")
  public double getShooterToCorrectTargetPoseDistance() {
    return distanceToTarget2D;
  }

  public double getShooterToCorrectTargetPoseDistance3D() {
    return distanceToTarget3D;
  }

  @AutoLogOutput(key = "ShotCalculator/AngularError")
  public Rotation2d getAngularError() {
    return angularError;
  }

  // Return This Util in Constants.java
  // Periodic Command Scheduler Overflow Handling
  public void updateTargetByAlliance() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    if (isRed) {
      hubTargetLocation = new Pose2d(redHubTarget, new Rotation2d());
      passTargetLocation =
          (currentPassSide == PassSide.CLOSE_LEFT)
              ? ShootOnTheFlyConstants.RED_PASS_CL_TARGET
              : ShootOnTheFlyConstants.RED_PASS_CR_TARGET;
    } else {
      hubTargetLocation = new Pose2d(blueHubTarget, new Rotation2d());
      passTargetLocation =
          (currentPassSide == PassSide.CLOSE_LEFT)
              ? ShootOnTheFlyConstants.BLUE_PASS_CL_TARGET
              : ShootOnTheFlyConstants.BLUE_PASS_CR_TARGET;
    }
  }

  //     Alliance alliance = DriverStation.getAlliance().get();
  //     boolean isRed = alliance == Alliance.Red;

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("ShotCalculator/HubTargetLocation", hubTargetLocation);
    Logger.recordOutput("ShotCalculator/PassTargetLocation", passTargetLocation);
    // Logger.recordOutput("ShotCalculator/RawTargetPose3d", rawTargetPose3d);
    // Logger.recordOutput("ShotCalculator/CorrectedTargetPose3d", correctedTargetPose3d);
    // Logger.recordOutput("ShotCalculator/ShooterPose3d", shooterPose3d);
    // Logger.recordOutput("ShotCalculator/DrivetrainSpeeds", drivetrainSpeeds);
  }
}
