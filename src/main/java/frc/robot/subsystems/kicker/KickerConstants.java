package frc.robot.subsystems.kicker;

public class KickerConstants {
  public static final int sparkMasterKicker = 13;

  // If using the FlywheelSubsystem of Closed-Loop control, make sure kP is really small
  public static final double sparkMasterKickerkP = 0.0001; //
  public static final double sparkMasterKickerkI = 0.0; // default: 0.0
  public static final double sparkMasterKickerkD = 0.0; // default: 0.0

  public static final double kickerReduction = 1.0; // Update with actual reduction ratio

  public static final double masterKickerEncoderPositionFactor =
      (2.0 * Math.PI) / kickerReduction; // Rotor Rotations -> Wheel Radians
  public static final double masterKickerEncoderVelocityFactor =
      (2.0 * Math.PI) / 60.0 / kickerReduction; // Rotor RPM -> Wheel Rad/Sec

  // FF Values collected from SysId Characterization
  public static final double kS = 0.15;
  public static final double kV = 0.12;
  public static final double kA = 0.05; // Torque Control Substitute

  public static final double maxAcceleration = 400.0; // rad/s^2, check with ReCalc
}
