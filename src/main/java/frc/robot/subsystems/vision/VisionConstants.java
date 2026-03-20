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

public final class VisionConstants {

  private VisionConstants() {}

  // ------- APRILTAG FIELD CONSTANTS -------- \\

public static final AprilTagFieldLayout aprilTagLayout;
  static {
      aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
      // All California District Events and DCMP should be WELDED fields.
  }

  // ------- LIMELIGHT NAME -------- \\

  // The User MUST get this right
  public static final String cameraPurple = "limelight-purple"; 
  public static final String cameraOrange = "limelight-orange"; 
  public static final String cameraYellow = "limelight-yellow"; 
  public static final String cameraPink = "limelight-pink"; 

  // Unused
  public static String cameraPlaceholder = "limelight-placeholder";

  // ------- LIMELIGHT TRANSFORM (SIM) -------- \\

  public static Transform3d cameraTransformToPurple =
      new Transform3d(0.1397, -0.3302, 0.1778, new Rotation3d(0.0, 0.38, 90.0)); // updated 1/22
  public static Transform3d cameraTransformToOrange =
      new Transform3d(-0.1524, -0.3302, 0.1778, new Rotation3d(0.0, 0.38, 90.0)); // updated 1/22
  public static Transform3d cameraTransformToGreen =
      new Transform3d(-0.127, 0.3302, 0.18415, new Rotation3d(0.0, 0.38, -90.0)); // updated 1/22
  public static Transform3d cameraTransformToBlue =
      new Transform3d(0.127, -0.3302, 0.18415, new Rotation3d(0.0, 0.38, -90.0)); // updated 1/22


  // ------- PER-LIMELIGHT TRUST MULTIPLIERS -------- \\
  // Note: This is indexed in the same order that the cameras are passed into the Vision constructor
  // Increase the factor equals less trust in that limelight

  public static final double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0 - Yellow - L
        1.0, // Camera 1 - Purple - L
        1.0, // Camera 2 - Pink - R
        1.0, // Camera 3 - Orange - L
      };

  // ------- BUMPER CLIPPING MARGIN -------- \\

  public static final double fieldBorderMargin = 0.5; // meters

  // ------- DISCONNECTION UTIL -------- \\

  public static final double ntDisconnectedTimeoutMs = 250.0; // please work
  public static final double frameDisconnectedTimeoutSec = 0.5;

  // ------- POSE FILTERING CONSTANTS -------- \\

  public static final double maxZError = 0.5; // meters

  public static final double maxLinearSpeed = 4.0; // m/s

  public static final double maxAngularSpeed = Math.toRadians(720); // rad/s (~2 full rotations)

  public static final double maxGyroError = 1.0; // Degrees

  // Only for when only one tag is present
  public static final double singleTagMinAreaPercent = 0.10; // Percent of Image frame
  public static final double singleTagMaxAngularVelocityRadPerSec = Math.toRadians(360); // rad/s
  public static final double singleTagThetaStdDev = Double.POSITIVE_INFINITY; // Redundant but wtv

  // Standard deviation baselines at 1 meter
  public static final double linearStdDevBaseline = 0.04; // Meters
  public static final double angularStdDevBaseline = Double.POSITIVE_INFINITY; // Radians

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // Never Trust

  // Scale factor applied to Std. Devs. for Limelight's native Std. Devs.
  // Set to 0.0 to disable the native std dev influence.
  public static final double limelightStdDevWeight = 0.5;

  // ------- MISC / UTIL  -------- \\

  // Index constants for limelight NT entry for std. dev.
  // Layout: [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1yaw, MT2x, MT2y, MT2z, MT2roll, MT2pitch, MT2yaw] - IIRC
  public static final int kMT2XStdDevIndex   = 6;
  public static final int kMT2YStdDevIndex   = 7;
  public static final int kMT2YawStdDevIndex = 11;
  public static final int kExpectedStdDevLen = 12;


  // Used for avoiding goofy floating point overflow
  public static final double LARGE_VARIANCE = 1e9;

  // Default (-1): No Tag ID filtering
  public static final int NO_EXCLUSIVE_TAG = -1;

  // How long AdvnatageScope holds onto the pose visualizer
  // So I don't need super-ision to see the poses :skull:
  public static final double tagLogPersistenceSeconds = 0.1;
}
