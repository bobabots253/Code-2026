// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String cameraPurple = "limelight-purple";
  public static String cameraOrange = "limelight-orange";
  public static String cameraGreen = "limelight-green";
  public static String cameraBlue = "limelight-blue";

  // Robot to camera transforms
  // (Not used by Limelight, configure in LL Finder UI instead)
  public static Transform3d cameraTransformToPurple =
      new Transform3d(0.1397, -0.3302, 0.1778, new Rotation3d(0.0, 0.38, 90.0)); // updated 1/22
  public static Transform3d cameraTransformToOrange =
      new Transform3d(-0.1524, -0.3302, 0.1778, new Rotation3d(0.0, 0.38, 90.0)); // updated 1/22
  public static Transform3d cameraTransformToGreen =
      new Transform3d(-0.127, 0.3302, 0.18415, new Rotation3d(0.0, 0.38, -90.0)); // updated 1/22
  public static Transform3d cameraTransformToBlue =
      new Transform3d(0.127, -0.3302, 0.18415, new Rotation3d(0.0, 0.38, -90.0)); // updated 1/22

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.25;
  public static double maxZError = 0.25;

  // Standard deviation baselines at 1 meter
  public static double linearStdDevBaseline = 0.01; // Meters
  public static double angularStdDevBaseline = 0.03; // Radians

  // Standard deviation multipliers for each camera
  // Manual Variance Weighting: (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        0.1, // Camera 0
        0.1, 0.5, 0.5
      }; 
      //Blue Camera - Cannot be trusted
      // MT2 on LL4 needs filtering for jumping
      // Add rotation limit

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.2; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // Never Trust
}
