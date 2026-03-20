/*
 * Unifying the best parts from FRC 1678 (2024), FRC 254 (FRC 2025), FRC 2910 (FRC 2025), FRC 6328 (FRC 2026), etc.
 */

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.angularStdDevBaseline;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import static frc.robot.subsystems.vision.VisionConstants.cameraStdDevFactors;
import static frc.robot.subsystems.vision.VisionConstants.linearStdDevBaseline;
import static frc.robot.subsystems.vision.VisionConstants.maxZError;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.FullSubsystem;

public class Vision extends FullSubsystem {
  /*
   * Pipeline:
   *  1. Read all cameras. Send single NT flush after all cameras.
   *  2. For each observation from each camera, run the rejection checks.
   *  3. For single-tag observations that pass intial, run quality gate check.
   *  4. For multi-tag observations, run the std. dev. euqation
   * 
   *  Then, accumulate accepted observations into a list.
   *  Sort by timestamps.
   *  Submit all sorted observations to the pose estimator consumer.
   */
  private final VisionConsumer consumer;
  private final Supplier<Rotation2d> gyroRotationSupplier;
  private final Supplier<ChassisSpeeds> robotSpeedsSupplier;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;

  // NT check: latencySubscriber not updated (handled in VisionIOLimelight).
  // Frame-based check: no frames received for frameDisconnectedTimeoutSec.
  private final Timer[] frameDisconnectTimers;
  private final Alert[] disconnectedAlerts;

  // Logging key strings
  private final String[] cameraInputKeys;
  private final String[] cameraPosesAcceptedKeys;
  private final String[] cameraLogPrefixes;

  // There ought to be a better way to do this, bruh
  // Tag Detection time persistence for AdvantageKit/Scope visualisation
  private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  // Note: When exclusiveTagId != NO_EXCLUSIVE_TAG, only observations containing that
  // tag ID are accepted. Volatile bc I might need in command thread.
  private volatile int exclusiveTagId = VisionConstants.NO_EXCLUSIVE_TAG;

  // Per-LL accepted and all pose lists. Cleared and reused each loop.
  private final ArrayList<Pose3d>[] perCameraRobotPoses;
  private final ArrayList<Pose3d>[] perCameraRobotPosesAccepted;

  // Global summary lists. Cleared and reused each loop.
  private final ArrayList<Pose3d> allRobotPoses;
  private final ArrayList<Pose3d> allRobotPosesAccepted;

  // Output buffers
  private Pose3d[] allRobotPosesBuffer         = new Pose3d[0];
  private Pose3d[] allRobotPosesAcceptedBuffer = new Pose3d[0];
  private Pose3d[] perCameraBuffer             = new Pose3d[8];
  private static final Pose3d[] EMPTY_POSE3D   = new Pose3d[0];

  private final ArrayList<PendingObservation> pendingObservations = new ArrayList<>(16);

  private Rotation3d cachedGyroRotation3d = new Rotation3d();
  private double lastGyroRadians = Double.NaN; // DO NOT USE 0.0

  /*
   * One fully-processed observation, send this boy
   */
  private record PendingObservation(
      double timestamp,
      Pose2d pose,
      Matrix<N3, N1> stdDevs) {}

  public Vision(
      VisionConsumer consumer,
      Supplier<Rotation2d> gyroRotationSupplier,
      Supplier<ChassisSpeeds> robotSpeedsSupplier,
      VisionIO... io) {
    this.consumer = consumer;
    this.gyroRotationSupplier = gyroRotationSupplier;
    this.robotSpeedsSupplier = robotSpeedsSupplier;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {

    // Instead of creating new lists each loop, clear allocated lists to reuse memory and reduce GC
    // overhead
    // allTagPoses.clear();
    allRobotPoses.clear();
    allRobotPosesAccepted.clear();
    // allRobotPosesRejected.clear();

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Update Robot State
    Rotation3d currentGyroRotation = new Rotation3d(gyroRotationSupplier.get());
    ChassisSpeeds currentSpeeds = robotSpeedsSupplier.get();

    // Calculate Speed Magnitudes
    double linearSpeed =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    double angularSpeed = Math.abs(currentSpeeds.omegaRadiansPerSecond);

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      // List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      // List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      // for (int tagId : inputs[cameraIndex].tagIds) {
      //   var tagPose = aprilTagLayout.getTagPose(tagId);
      //   if (tagPose.isPresent()) {
      //     tagPoses.add(tagPose.get());
      //   }
      // }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        if (!rejectPose) {
          if (linearSpeed > VisionConstants.maxLinearSpeed
              || angularSpeed > VisionConstants.maxAngularSpeed) {
            rejectPose = true;
          }
        }

        if (!rejectPose) {
          if (Math.abs(
                  Math.toDegrees(
                      (observation.pose().getRotation().minus(currentGyroRotation).getAngle())))
              > VisionConstants.maxGyroError) {
            rejectPose = true;
          }
        }

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          // robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate dynamically standard deviation scalar factor
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0)
                / observation.tagCount(); // Square tag count if vision is more trustworthy

        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;

        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= VisionConstants.linearStdDevMegatag2Factor;
          angularStdDev *= VisionConstants.angularStdDevMegatag2Factor;
        }

        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, 9999999999999999999.0));
      }

      // Log camera data (Disable in Match to save Bandwidth)
      // Use Limelight Replay Feature Instead of Logging Poses to Dashboard
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      // allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      // allRobotPosesRejected.addAll(robotPosesRejected);
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  @Override
  public void periodicAfterScheduler() {
    // Log summary data
    // Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    // Logger.recordOutput(
    //     "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }
}
