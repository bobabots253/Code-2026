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

  public static final double sparkMasterFlyWheelkP = 0.01;
  public static final double sparkMasterFlyWheelkI = 0.0;
  public static final double sparkMasterFlyWheelkD = 0.0;

  public static final double sparkMasterFlyWheelkS = 0.0;
  public static final double sparkMasterFlyWheelkV = 0.0;
  public static final double sparkMasterFlyWheelkA = 0.0;

  public static final double flywheelReductionRatio = 1.0;
}
