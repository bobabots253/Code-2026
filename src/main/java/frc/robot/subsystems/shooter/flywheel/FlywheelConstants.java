package frc.robot.subsystems.shooter.flywheel;

public class FlywheelConstants {
  // public static final FlywheelConfig flywheelConfig =
  //     switch (Constants.getRobot()) {
  //       case LEBOBAJAMES -> new FlywheelConfig(15, 16, (1.0), 9000.0);
  //       case SIMBOT -> new FlywheelConfig(0, 0, (0.5), 9000.0);
  //     };

  // public record FlywheelConfig(int masterId, int followerId, double outputReduction, double
  // accelerationLimit) {}

  // public static final Gains defaultGains =
  //     switch (Constants.getRobot()) {
  //       case LEBOBAJAMES -> new Gains(0.01, 0, 0.0006, 0.0, 0.00, 0);
  //       case SIMBOT -> new Gains(0.05, 0.0, 0.0, 0.0, 0.0, 0.0);
  //     };

  // public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public static final int sparkMasterFlywheelCanId = 15;
  public static final int sparkFollowerFlywheelCanId = 16;

  public static final double sparkMasterFlyWheelkP = 0.0001; // Should be super small
  public static final double sparkMasterFlyWheelkI = 0.0; // default: 0.0
  public static final double sparkMasterFlyWheelkD = 0.0; // default: 0.0

  public static final double flywheelReductionRatio = 1.0;

  public static final double masterFlywheelEncoderPositionFactor =
      (2.0 * Math.PI) / flywheelReductionRatio; // Rotor Rotations -> Wheel Radians
  public static final double masterFlywheelEncoderVelocityFactor =
      (2.0 * Math.PI) / 60.0 / flywheelReductionRatio; // Rotor RPM -> Wheel Rad/Sec

  public static final double followerFlywheelEncoderPositionFactor =
      (2.0 * Math.PI) / flywheelReductionRatio; // Rotor Rotations -> Wheel Radians
  public static final double followerFlywheelEncoderVelocityFactor =
      (2.0 * Math.PI) / 60.0 / flywheelReductionRatio; // Rotor RPM -> Wheel Rad/Sec

  // FF Values collected from SysId Characterization
  public static final double kS = 0.15;
  public static final double kV = 0.12;
  public static final double kA = 0.05; // Torque Control Substitute

  public static final double maxAcceleration = 400.0; // rad/s^2, check with ReCalc
}
