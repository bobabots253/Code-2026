// package frc.robot.commands;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.swerve.SwerveSubsystem;

// public class CrossTheBumpCommand extends Command{
//     // Directional Enums
//     public enum CrossingDirection {
//         TO_NEUTRAL, TO_ALLIANCE
//     }

//     // State Machine Tracking Enum
//     private enum BumpState {
//         APPROACHING,
//         CLIMBING,
//         CRESTING,
//         DESCENDING,
//         LEVELING,
//         COMPLETED,
//         TIMED_OUT
//     }

//     public static final class BumpConstants {

//     public static double approachSpeedMs = 1.0;
//     public static double climbingSpeedMs = 0.6;
//     public static double levelingSpeedMs = 0.4;

//     public static double pitchClimbingThreshold = Units.degreesToRadians(3.0);
//     }

//     private final SwerveSubsystem drive;
//     private final CrossingDirection  direction;
// }
