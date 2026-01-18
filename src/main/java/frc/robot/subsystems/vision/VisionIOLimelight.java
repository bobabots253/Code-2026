// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;
    orientationPublisher =
        table
            .getDoubleArrayTopic("robot_orientation_set")
            .publish(); // [yaw,yawrate,pitch,pitchrate,roll,rollrate] - Degrees / Degrees per
    // second
    latencySubscriber =
        table
            .getDoubleTopic("tl")
            .subscribe(0.0); // Select pipeline's latency in ms. Total Latency = tl + cl.
    txSubscriber =
        table.getDoubleTopic("tx").subscribe(0.0); // Horizontal Offset From Crosshair To Target
    tySubscriber =
        table.getDoubleTopic("ty").subscribe(0.0); // Vertical Offset From Crosshair To Target
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    // Translation (X,Y,Z) in meters Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl),
    // tag count, tag span, average tag distance from camera, average tag area ()
    // Should be "botpose_wpiblue", but don't fix what isn't broken
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, using only the first tag because ambiguity isn't applicable for multitag
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_1));
    }
    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, zeroed because the pose is already disambiguated
              0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_2));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
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
}
