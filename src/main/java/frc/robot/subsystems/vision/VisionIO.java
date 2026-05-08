// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {

    public boolean connected = false;
    public double lastFrameTimestampSec = 0.0;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
    public double[] limelightStdDevs = new double[0];
  }

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      double averageTagArea,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_2,
    PHOTONVISION
  }

  default void updateInputs(VisionIOInputs inputs) {}

  default void setThrottleValue(int throttleValue) {}
}
