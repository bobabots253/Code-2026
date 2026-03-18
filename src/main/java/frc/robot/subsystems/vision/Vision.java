// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.angularStdDevBaseline;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import static frc.robot.subsystems.vision.VisionConstants.cameraStdDevFactors;
import static frc.robot.subsystems.vision.VisionConstants.linearStdDevBaseline;
import static frc.robot.subsystems.vision.VisionConstants.maxZError;

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
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.FullSubsystem;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends FullSubsystem {
  private final VisionConsumer consumer;
  private final Supplier<Rotation2d> gyroRotationSupplier;
  private final Supplier<ChassisSpeeds> robotSpeedsSupplier;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  // Initialize logging values
  // List<Pose3d> allTagPoses = new LinkedList<>();
  List<Pose3d> allRobotPoses = new LinkedList<>();
  List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
  // List<Pose3d> allRobotPosesRejected = new LinkedList<>();

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
