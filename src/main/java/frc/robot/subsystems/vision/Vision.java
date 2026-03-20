/*
 * Unifying the best parts from FRC 1678 (2024), FRC 254 (FRC 2025), FRC 2910 (FRC 2025), FRC 6328 (FRC 2026), etc.
 */

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.FullSubsystem;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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
   *
   *  Memory Estimations (I am not a memory expert):
   *  8 - Per-LL Oberservations: I am expecting maybe 1-2 frames per 50 hz, maybe 3-4. Double for safety.
   *  Java's backend defaults ArrayList internal array length to an Object sized at [10]. I don't think that
   *  matter too much but yeah.
   *  16 - Per-LL Oberservation x Cameras (4 poseObs  x 4 LL)
   *  Don't overcommit memory.
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
  // JVM has type-erasure for casted ArrayLists
  // Bad programming habits, fix later.
  @SuppressWarnings("unchecked")
  private final ArrayList<Pose3d>[] perCameraRobotPoses;

  @SuppressWarnings("unchecked")
  private final ArrayList<Pose3d>[] perCameraRobotPosesAccepted;

  // Global summary lists. Cleared and reused each loop.
  private final ArrayList<Pose3d> allRobotPoses;
  private final ArrayList<Pose3d> allRobotPosesAccepted;

  // Output buffers
  private Pose3d[] allRobotPosesBuffer = new Pose3d[0];
  private Pose3d[] allRobotPosesAcceptedBuffer = new Pose3d[0];
  private Pose3d[] perCameraBuffer = new Pose3d[PER_CAMERA_CAPACITY];
  private static final Pose3d[] EMPTY_POSE3D = new Pose3d[0];

  private Rotation3d cachedGyroRotation3d = new Rotation3d();
  private double lastGyroRadians = Double.NaN; // DO NOT USE 0.0

  private static final int PER_CAMERA_CAPACITY = 8;
  private static final int PENDING_OBSERVATION_CAPACITY = 16;

  private final ArrayList<PendingObservation> pendingObservations =
      new ArrayList<>(PENDING_OBSERVATION_CAPACITY);

  /*
   * One fully-processed observation, send this boy
   */
  private record PendingObservation(double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {}

  // GG
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
    frameDisconnectTimers = new Timer[io.length];
    disconnectedAlerts = new Alert[io.length];
    cameraInputKeys = new String[io.length];
    cameraPosesAcceptedKeys = new String[io.length];
    cameraLogPrefixes = new String[io.length];
    perCameraRobotPoses = new ArrayList[io.length];
    perCameraRobotPosesAccepted = new ArrayList[io.length];

    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();

      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
      cameraInputKeys[i] = "Vision/Camera" + i;
      cameraPosesAcceptedKeys[i] = "Vision/Camera" + i + "/RobotPosesAccepted";
      cameraLogPrefixes[i] = "Vision/Camera" + i;

      // Initialize disconnected alerts
      // I don't think this should be too memory intensive
      frameDisconnectTimers[i] = new Timer();
      frameDisconnectTimers[i].start();
      disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " is disconnected.", AlertType.kWarning);

      // Pre-allocate per-camera lists with initial capacity.
      // Size up bc we want to avoid re-allocation
      perCameraRobotPoses[i] = new ArrayList<>(PER_CAMERA_CAPACITY);
      perCameraRobotPosesAccepted[i] = new ArrayList<>(PER_CAMERA_CAPACITY);
    }

    // Initial capacity for the camera summary lists
    allRobotPoses = new ArrayList<>(io.length * PER_CAMERA_CAPACITY);
    allRobotPosesAccepted = new ArrayList<>(io.length * PER_CAMERA_CAPACITY);
  }

  // ------- TAG EXCLUSION UTIL  -------- \\

  public void setExclusiveTag(int tagId) {
    this.exclusiveTagId = tagId;
  }

  public void clearExclusiveTag() {
    this.exclusiveTagId = VisionConstants.NO_EXCLUSIVE_TAG;
  }

  public int getExclusiveTag() {
    return exclusiveTagId;
  }

  // ------- THROTTLING UTIL  -------- \\
  // Stolen from 2910

  public void setThrottleValue(int throttleValue) {
    for (VisionIO camera : io) {
      camera.setThrottleValue(throttleValue);
    }
  }

  @Override
  public void periodic() {

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs(cameraInputKeys[i], inputs[i]);
    }

    // Updated all 4 cameras then flush
    NetworkTableInstance.getDefault().flush();

    // Cache Gyro Rotation
    double currentGyroRad = gyroRotationSupplier.get().getRadians();
    // The value of the lastGyroRadians intializes at NaN.
    if (Double.isNaN(lastGyroRadians) || currentGyroRad != lastGyroRadians) {
      cachedGyroRotation3d = new Rotation3d(gyroRotationSupplier.get());
      lastGyroRadians = currentGyroRad;
    }

    // Check if Robot is lightning McQueen
    // I swear, do this calc once, not per camera.
    ChassisSpeeds currentSpeeds = robotSpeedsSupplier.get();
    // Calculate Speed Magnitudes
    double linearSpeed =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    double angularSpeed = Math.abs(currentSpeeds.omegaRadiansPerSecond);

    // Clear buffers
    allRobotPoses.clear();
    allRobotPosesAccepted.clear();
    pendingObservations.clear();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update dual disconnected alert
      // Reset timer when new frame arrives
      // NT check handled by [cameraIndex].connected

      // Stolen from 6328
      if (inputs[cameraIndex].lastFrameTimestampSec > 0
          && inputs[cameraIndex].poseObservations.length > 0) {
        frameDisconnectTimers[cameraIndex].reset();
      }

      boolean frameDisconnected =
          frameDisconnectTimers[cameraIndex].hasElapsed(
              VisionConstants.frameDisconnectedTimeoutSec);
      boolean disconnected = !inputs[cameraIndex].connected || frameDisconnected;

      if (disconnected) {
        String reason =
            !inputs[cameraIndex].connected
                ? "NT disconnected"
                : "No Frames Received for "
                    + VisionConstants.frameDisconnectedTimeoutSec
                    + " seconds";
        disconnectedAlerts[cameraIndex].setText(
            "Vision camera " + cameraIndex + " disconnected (" + reason + ").");
      }
      disconnectedAlerts[cameraIndex].set(disconnected);
      // ---

      // TO-DO: Add util to update when DS is disabled

      // If exclyusive tags are included and not seen, skip all observations.
      int exclusion = exclusiveTagId;
      if (exclusion != VisionConstants.NO_EXCLUSIVE_TAG) {
        boolean cameraSeesExclusive = false;
        for (int id : inputs[cameraIndex].tagIds) {
          if (id == exclusion) {
            cameraSeesExclusive = true;
            break;
          }
        }
        if (!cameraSeesExclusive) {
          // Log that we're filtering this camera, then skip.
          Logger.recordOutput(cameraLogPrefixes[cameraIndex] + "/ExclusiveTagFiltered", true);
          continue;
        }
      }
      Logger.recordOutput(cameraLogPrefixes[cameraIndex] + "/ExclusiveTagFiltered", false);

      // Clear Per-LL Pose lists
      ArrayList<Pose3d> robotPoses = perCameraRobotPoses[cameraIndex];
      ArrayList<Pose3d> robotPosesAccepted = perCameraRobotPosesAccepted[cameraIndex];
      robotPoses.clear();
      robotPosesAccepted.clear();

      // Update Per-Observation filtering
      for (var observation : inputs[cameraIndex].poseObservations) {

        // See pipeline above
        boolean reject = (observation.tagCount() == 0);

        if (!reject) reject = (Math.abs(observation.pose().getZ()) > VisionConstants.maxZError);

        // Margin added recommended by 6328
        if (!reject) {
          double x = observation.pose().getX();
          double y = observation.pose().getY();
          reject =
              x < -VisionConstants.fieldBorderMargin
                  || x
                      > VisionConstants.aprilTagLayout.getFieldLength()
                          + VisionConstants.fieldBorderMargin
                  || y < -VisionConstants.fieldBorderMargin
                  || y
                      > VisionConstants.aprilTagLayout.getFieldWidth()
                          + VisionConstants.fieldBorderMargin;
        }

        // Technically this shouldnt really be needed
        if (!reject) reject = (linearSpeed > VisionConstants.maxLinearSpeed);

        // Reject if yawRate is too high = timestamp might not be as reliable
        if (!reject) reject = (angularSpeed > VisionConstants.maxAngularSpeed);

        // Check reprojection error for MT2 measurements
        if (!reject) {
          double gyroVisionDifferenceDeg =
              Math.abs(
                  Math.toDegrees(
                      observation.pose().getRotation().minus(cachedGyroRotation3d).getAngle()));
          reject = (gyroVisionDifferenceDeg > VisionConstants.maxGyroError);
        }

        // Log all pose observation
        robotPoses.add(observation.pose());

        if (reject) {
          // Observation logged but not accepted.
          continue;
        }

        // Single Tag and Multi-Tag need to be weighted differently
        // Taken from 254
        final boolean isSingleTag = (observation.tagCount() == 1);
        Pose2d acceptedPose2d;
        double xyStdDev;
        double thetaStdDev;

        if (isSingleTag) {
          if (observation.averageTagArea() < VisionConstants.singleTagMinAreaPercent) {
            // Rm if too laggy
            Logger.recordOutput(cameraLogPrefixes[cameraIndex] + "/RejectedSingleTagArea", true);
            continue;
          }

          if (angularSpeed > VisionConstants.singleTagMaxAngularVelocityRadPerSec) {
            Logger.recordOutput(
                cameraLogPrefixes[cameraIndex] + "/RejectedSingleTagAngularVel", true);
            continue;
          }

          // If singleTag is accepted, use MT2 translation
          acceptedPose2d = observation.pose().toPose2d();
          xyStdDev = calculateXYStdDev(observation, cameraIndex, inputs[cameraIndex]);
          thetaStdDev = VisionConstants.singleTagThetaStdDev;

        } else {
          acceptedPose2d = observation.pose().toPose2d();
          xyStdDev = calculateXYStdDev(observation, cameraIndex, inputs[cameraIndex]);
          thetaStdDev = Double.POSITIVE_INFINITY;
          Logger.recordOutput(cameraLogPrefixes[cameraIndex] + "/AcceptedMultiTag", acceptedPose2d);

          // Utility for forcing AdvantageScope to keep Tag visible
          for (int tagId : inputs[cameraIndex].tagIds) {
            lastTagDetectionTimes.put(tagId, Timer.getTimestamp());
          }

          // Aggregate accepted poses into a list
          robotPosesAccepted.add(observation.pose());

          // Sorting by timestamp is done at the end of the loop
          pendingObservations.add(
              new PendingObservation(
                  observation.timestamp(),
                  acceptedPose2d,
                  VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
        }

        Logger.recordOutput(cameraPosesAcceptedKeys[cameraIndex], toArray(robotPosesAccepted));
        Logger.recordOutput(
            cameraLogPrefixes[cameraIndex] + "/TagCount", inputs[cameraIndex].tagIds.length);
        Logger.recordOutput(cameraLogPrefixes[cameraIndex] + "/Connected", !disconnected);
      }
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  private double calculateXYStdDev(
      VisionIO.PoseObservation observation,
      int cameraIndex,
      VisionIOInputsAutoLogged cameraInputs) {

    // Original method
    double modelStdDev =
        VisionConstants.linearStdDevBaseline
            * Math.pow(observation.averageTagDistance(), 2) // Modify to Tune
            / Math.pow(observation.tagCount(), 1); // Modify to Tune

    // Per-camera trust multiplier
    if (cameraIndex < VisionConstants.cameraStdDevFactors.length) {
      modelStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
    }

    // Quality scaling from Limelight's native stddevs (254)
    // If Limelight provides MT2 std devs and they're larger,
    // take a weighted blend in the next calculation.

    double[] llStdDevs = cameraInputs.limelightStdDevs;
    if (llStdDevs.length >= VisionConstants.kExpectedStdDevLen
        && observation.type() == PoseObservationType.MEGATAG_2) {
      double llXStd = llStdDevs[VisionConstants.kMT2XStdDevIndex];
      double llYStd = llStdDevs[VisionConstants.kMT2YStdDevIndex];
      // Take the higher uncertainty
      double llXYMax = Math.max(llXStd, llYStd);

      // Blend-- if Limelight is more pessimistic, weight its view more.
      // Inverse Variance Weighting between two estimates
      double blended =
          modelStdDev * (1.0 - VisionConstants.limelightStdDevWeight)
              + llXYMax * VisionConstants.limelightStdDevWeight;
      modelStdDev = Math.max(modelStdDev, blended); // never trust more than our model alone
    }

    return modelStdDev;
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

  // ------- UTIL  -------- \\

  // Because the buffer length grows, this util helps alleviate some of the array allocation
  private Pose3d[] toArray(ArrayList<Pose3d> list) {
    int size = list.size();
    if (size == 0) return EMPTY_POSE3D;
    if (perCameraBuffer.length < size) {
      perCameraBuffer = new Pose3d[size * 2];
    }
    return list.toArray(perCameraBuffer);
  }
}
