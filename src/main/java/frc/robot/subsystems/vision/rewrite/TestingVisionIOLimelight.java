package frc.robot.subsystems.vision.rewrite;
// package frc.robot.subsystems.vision;

// import java.util.concurrent.atomic.AtomicReference;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

// public class TestingVisionIOLimelight implements TestingVisionIO{
//     NetworkTable tableA =
//             NetworkTableInstance.getDefault().getTable(TestingVisionConstants.cameraATableName);
//     NetworkTable tableB =
//             NetworkTableInstance.getDefault().getTable(TestingVisionConstants.cameraBTableName);
//     AtomicReference<VisionIOInputs> latestInputs = new AtomicReference<>(new VisionIOInputs());

//     // private static final double[] DEFAULT_STDDEVS = new double[
//     //     TestingVisionConstants.linearStdDevBaseline,
//     //     TestingVisionConstants.angularStdDevBaseline];

//     public TestingVisionIOLimelight(){
//         setLLSettings();
//     }

// /** Configures Limelight camera poses in robot coordinate system. */
//     private void setLLSettings() {

//         double[] cameraAPose = {
//             Constants.VisionConstants.kRobotToCameraAForward,
//             Constants.VisionConstants.kRobotToCameraASide,
//             VisionConstants.kCameraAHeightOffGroundMeters,
//             0.0,
//             VisionConstants.kCameraAPitchDegrees,
//             VisionConstants.kCameraAYawOffset.getDegrees()
//         };

//         tableA.getEntry("camerapose_robotspace_set").setDoubleArray(cameraAPose);

//         // double[] cameraXPose = {
//         //     Constants.VisionConstants.kRobotToCameraXForward,
//         //     Constants.VisionConstants.kRobotToCameraXSide,
//         //     VisionConstants.kCameraXHeightOffGroundMeters,
//         //     0.0,
//         //     VisionConstants.kCameraXPitchDegrees,
//         //     VisionConstants.kCameraXYawOffset.getDegrees()
//         // };

//         // tableX.getEntry("camerapose_robotspace_set").setDoubleArray(cameraXPose);
//     }

// }
