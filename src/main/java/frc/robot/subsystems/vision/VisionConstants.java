// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout - Test FieldConstants from 6328
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); // Fix to 2026 Layout
  //   FieldConstants();

  // Camera names, must match names configured on coprocessor
  public static String cameraPurple = "limelight-purple"; // BL
  public static String cameraOrange = "limelight-orange"; // BR
  public static String cameraYellow = "limelight-yellow"; // FL
  public static String cameraPink = "limelight-pink"; // FR

  // Unused
  public static String cameraPlaceholder = "limelight-placeholder";

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
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines at 1 meter
  public static double linearStdDevBaseline = 0.04; // Meters
  public static double angularStdDevBaseline = Double.POSITIVE_INFINITY; // Radians

  // Standard deviation multipliers for each camera
  // Manual Variance Weighting: (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0 - Yellow - L
        1.0, // Camera 1 - Purple - L
        2.0, // Camera 2 - Pink - R
        2.0, // Camera 3 - Orange - L
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // Never Trust

  // Clamping ranges for vision estimates (see units)
  static final double maxLinearSpeed = 2.0; // Meters per second
  static final double maxAngularSpeed =
      DegreesPerSecond.of(270).in(RadiansPerSecond); // Radians per second
  static final double maxGyroError = 1.0; // Degrees
  static final double maxTranslationError = 1.0; // Meters
  static final int LOCK_MODE = 10;
}
