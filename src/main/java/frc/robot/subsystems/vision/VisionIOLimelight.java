// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  // Storing NT once avoids repeated static lookup
  private static final NetworkTableInstance NT = NetworkTableInstance.getDefault();
 
  private final Supplier<Rotation2d> rotationSupplier;
  private final Supplier<Double> yawRateSupplier; // rad/s

  private final DoubleArrayPublisher orientationPublisher;
  private final DoubleSubscriber     latencySubscriber;       // pipeline latency (ms)
  private final DoubleArraySubscriber megatag2Subscriber;     // botpose_orb_wpiblue
  private final DoubleArraySubscriber limelightStdDevsSubscriber; // native units by LL
  private final DoubleSubscriber     throttlePublisher;

  // Pre-allocate buffers
  private final ArrayList<PoseObservation> poseObservationCache = new ArrayList<>(8);

  // Rm Hashset method for preventing tagId duplication
  // 32 tag is totally unrealistic but let's just not get a overflow error
  private static final int MAX_TAGS = 32;
  private final int[] tagIdScratchLottery  = new int[MAX_TAGS];
  private int tagIdCount = 0;

  // Output array for inputs.poseObservations
  // Rm the PoseObservation[n] memory allocation required
  private PoseObservation[] poseObservationOutput = new PoseObservation[8];

  // Default a blank array to store the std dev in case it doesn't work
  private static final double[] stdDevCache = new double[0];

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier, Supplier<Double> yawRateSupplier) {

    var table = NT.getTable(name);

    this.rotationSupplier = rotationSupplier;
    this.yawRateSupplier  = yawRateSupplier;

    orientationPublisher =
        table.getDoubleArrayTopic("robot_orientation_set").publish();
        // [yaw,yawrate,pitch,pitchrate,roll,rollrate] - Degrees / Degrees per second
    latencySubscriber =
        table.getDoubleTopic("tl").subscribe(0.0);
        // Select pipeline's latency in ms. Total Latency = tl + cl.
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[]{});
        // Translation (X,Y,Z) in meters Rotation(Roll,Pitch,Yaw) in degrees

    limelightStdDevsSubscriber =
        table.getDoubleArrayTopic("stddevs").subscribe(stdDevCache);

    // Not to sure if this is going to work
    // NT topics are write only so im goofing a DoubleSubscriber to publish changes
    throttlePublisher =
        table.getDoubleTopic("throttle_set").subscribe(0.0);

  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    // -------  NT CHECK  -------- \\

    // Latency topic not updated in 250ms = NT disconnected
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000)
            < VisionConstants.ntDisconnectedTimeoutMs;

    // -------  ROBOT ORIENTATION PUBLISHING  -------- \\

    // Publish BEFORE reading poses, the User must get this right
    // Added yawRate bc it helps with latency compensation
    // Note: flush() is NOT called here

    // Update orientation for MegaTag 2
    double yawDeg = rotationSupplier.get().getDegrees();
    double yawRateDegPerSec = Units.radiansToDegrees(yawRateSupplier.get());
    orientationPublisher.accept(
        new double[] {yawDeg, yawRateDegPerSec, 0.0, 0.0, 0.0, 0.0});

    // -------  LL NATIVE STD DEV  -------- \\
    inputs.limelightStdDevs = limelightStdDevsSubscriber.get(stdDevCache);

    // -------  PROCESS THE POSE OBSERVATION  -------- \\
    // **insert pain

    // Clear Pre-allocated buffers
    poseObservationCache.clear();
    tagIdCount = 0;

    // Process Buffer Frames
    //readQueue() returns all frames captured since the last call (0 or multiple)
    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;

       // Use the NT server timestamp of this sample (multiply conversion for seconds)
      inputs.lastFrameTimestampSec = rawSample.timestamp * 1.0e-6;

      // I might go insane but here we go
      // Collect frame tagIds without duplicates (without HashSets)
      // botpose_orb_wpiblue data starts at index 11, 7 values per tag
      for (int i = 11; i < rawSample.value.length; i += 7) {
        int id = (int) rawSample.value[i];
        boolean found = false;
        for (int j = 0; j < tagIdCount; j++) {
          if (tagIdScratchLottery[j] == id) { found = true; break; }
        }
        if (!found && tagIdCount < MAX_TAGS) {
          tagIdScratchLottery[tagIdCount++] = id;
        }
      }

      // Build Pose Observation
      // botpose_orb_wpiblue array layout:
      //   [0-2]  : X, Y, Z (meters)
      //   [3-5]  : roll, pitch, yaw (degrees)
      //   [6]    : total latency in ms (tl + cl. Use this, not just tl)
      //   [7]    : tag count
      //   [8]    : tag span (i don't care about this)
      //   [9]    : average tag distance (meters)
      //   [10]   : average tag area (percent of image, 0–100) -> convert to fraction
      //   [11...]: tag data (7 values per tag)
      poseObservationCache.add(
          new PoseObservation(
              // Timestamp: NT server publish time minus total pipeline latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
              parsePose(rawSample.value),
              0.0,
              (int) rawSample.value[7],          // tagCount
              rawSample.value[9],                 // averageTagDistance (meters)
              rawSample.value[10] / 100.0,        // averageTagArea: convert % to fraction
              PoseObservationType.MEGATAG_2));
    }

    // Write Pose Observations
    int observationSize = poseObservationCache.size();
    if (poseObservationOutput.length < observationSize) {
      poseObservationOutput = new PoseObservation[observationSize * 2];
      // If not enough space, double that size. 
    }

    // Note: toArray writes into the provided array and returns it
    inputs.poseObservations = poseObservationCache.toArray(poseObservationOutput);

    // Write Tag ID's
    // Only reallocate inputs.tagIds if the count changed
    if (inputs.tagIds.length != tagIdCount) {
      inputs.tagIds = new int[tagIdCount];
    }
    System.arraycopy(tagIdScratchLottery, 0, inputs.tagIds, 0, tagIdCount);

  }

  /**
   * Parses the 3D pose from a Limelight botpose array. Rotation3D(units::radian_t roll,
   * units::radian_t pitch, units::radian_t yaw)
   */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }

  @Override
  public void setThrottleValue(int throttleValue) {
    NT.getTable(orientationPublisher.getTopic().getName().split("/")[0])
        .getEntry("throttle_set")
        .setNumber(throttleValue);
  }
}
