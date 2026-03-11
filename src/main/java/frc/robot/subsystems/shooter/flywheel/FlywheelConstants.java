package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class FlywheelConstants {

  public static final int sparkMasterFlywheelCanId = 15;
  public static final int sparkFollowerFlywheelCanId = 16;

  public static final double sparkMasterFlyWheelkP =
      0.65 * Units.inchesToMeters(2) * 0.2; //  V*s/m -> V*s/rad
  public static final double sparkMasterFlyWheelkI = 0.0; // default: 0.0
  public static final double sparkMasterFlyWheelkD = 0.0; // default: 0.0

  // Used as an early warning if the Flywheel is jammed
  public static final double highCurrentThreshold = 60.0; // Amps

  public static final double closedLoopVelocityTolerance = 0.0; // Change to heuristic value, HALT
  public static final double kShotTolerance = Units.rotationsPerMinuteToRadiansPerSecond(200);

  public static final double flywheelReductionRatio = 1.0; // Used in Max Accel Calculations

  public static final double masterFlywheelEncoderPositionFactor =
      (2.0 * Math.PI) / flywheelReductionRatio; // Rotor Rotations -> Wheel Radians
  public static final double masterFlywheelEncoderVelocityFactor =
      (2.0 * Math.PI) / 60.0 / flywheelReductionRatio; // Rotor RPM -> Wheel Rad/Sec

  public static final double followerFlywheelEncoderPositionFactor =
      (2.0 * Math.PI) / flywheelReductionRatio; // Rotor Rotations -> Wheel Radians
  public static final double followerFlywheelEncoderVelocityFactor =
      (2.0 * Math.PI) / 60.0 / flywheelReductionRatio; // Rotor RPM -> Wheel Rad/Sec

  // FF Values collected from Re.Calc Ghost Characterization
  // https://www.reca.lc/flywheel?currentLimit=%7B%22s%22%3A50%2C%22u%22%3A%22A%22%7D&efficiency=80&flywheelMomentOfInertia=%7B%22s%22%3A0.0002%2C%22u%22%3A%22in2%2Albs%22%7D&flywheelRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&flywheelRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Reduction%22%7D&flywheelWeight=%7B%22s%22%3A0.0001%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%20Vortex%22%7D&motorRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Step-up%22%7D&projectileRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&projectileWeight=%7B%22s%22%3A1%2C%22u%22%3A%22lbs%22%7D&shooterMomentOfInertia=%7B%22s%22%3A6.4%2C%22u%22%3A%22in2%2Albs%22%7D&shooterRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&shooterTargetSpeed=%7B%22s%22%3A4000%2C%22u%22%3A%22rpm%22%7D&shooterWeight=%7B%22s%22%3A3.2%2C%22u%22%3A%22lbs%22%7D&useCustomFlywheelMoi=0&useCustomShooterMoi=0
  public static final double kS = 0.00; // Can properly tune after comp
  public static final double kV = 0.33 * Units.inchesToMeters(2) * 1.0; // V*s/m -> V*s/rad
  public static final double kA = 0.60 * Units.inchesToMeters(2) * 1.0; // V*s/m -> V*s/rad

  // Calculating Max Acceleration of a Two-Motor Flywheel System
  // Use the flywheelReductionRatio constant from above
  public static final double vortexStallTorque = 3.6;
  public static final double vortexCount = 2.0;
  public static final double vortexMaxEfficiency = 0.8; // decimalPercentage
  public static final double flywheelMaxTorque =
      (vortexStallTorque * vortexCount) * flywheelReductionRatio * vortexMaxEfficiency;
  public static final double shooterwheelMOI = 0.00187289378195; // kg * m^2
  public static final double maxAcceleration =
      flywheelMaxTorque / shooterwheelMOI; // rad/s^2, check with ReCalc

  public static final double idleTolerance = 20.94 / 2; // rad/sec, 100 rpm

  public static final double flywheelControlVelocityTolerance = 20.94; // rad/sec, 200 rpm
  public static final double ballDetectionThreshold =
      -300.94; // rad/sec^2, CHECK physically check how much the velocity drops from one shot

  /*
   * Solving for kI_velocity:
   * 1. Run the flywheel at 2V, 4V, 6V, 8V, and 10V (Cap at 75% MAX_FREE_SPEED)
   * 2. Wait for the speed to reach steady state
   * 3. Record the stabilized velocity (Rad/sec) and Output Current (Amps)
   * 4. Plot X (velocity) and Y (current) on graph
   * 5. Get LOBF or COBF to solve.
   * I think this might work. Ideally, If it takes 5A to maintain 100 rad/s, 5/100 = 0.05 amps/(rad/sec)
   * I don't care about static current, that's dtm.
   * Optimized method would use an Interpolating Tree Map
   */
  // Make sure this value is able to hold the motor at the setpoint while in IDLE phase
  // If there is too much drift, this value is too low
  public static final double kIdleVelocityLinearCoefficient =
      0.0; // amps/(rad/sec) SOLVE KADEN, DONT FORGET.

  public static final InterpolatingDoubleTreeMap IDLE_VOLTAGE_INTERPOLATOR =
      new InterpolatingDoubleTreeMap();

  // Key:
  static {
    IDLE_VOLTAGE_INTERPOLATOR.put(Units.rotationsPerMinuteToRadiansPerSecond(1000), 1.95);
    IDLE_VOLTAGE_INTERPOLATOR.put(Units.rotationsPerMinuteToRadiansPerSecond(2100), 3.90);
    IDLE_VOLTAGE_INTERPOLATOR.put(Units.rotationsPerMinuteToRadiansPerSecond(3300), 6.0);
    IDLE_VOLTAGE_INTERPOLATOR.put(Units.rotationsPerMinuteToRadiansPerSecond(3800), 7.0);
    IDLE_VOLTAGE_INTERPOLATOR.put(Units.rotationsPerMinuteToRadiansPerSecond(4370), 8.0);
    IDLE_VOLTAGE_INTERPOLATOR.put(Units.rotationsPerMinuteToRadiansPerSecond(4900), 9.0);
    IDLE_VOLTAGE_INTERPOLATOR.put(Units.rotationsPerMinuteToRadiansPerSecond(5490), 10.0); // 14.5
    // IDLE_VOLTAGE_INTERPOLATOR.put(0.0, 0.0);
    // IDLE_VOLTAGE_INTERPOLATOR.put(0.0, 0.0);
  }

  /*
  NOTE: TUNE Idle Phase first, PLEASE
   * 1. Set flywheel sampled shooting speed
   * 2. Shoot a ball and record velocity
   * 3. Look at Velocity Drop and the Recovery Time
   * 4. Look at re.calc's expected drop in RPM
   * 5. If it drops too much, increase AntiLag value
   * 6. Test with at least 3 balls, catch if Ball 2 flies shorter than Ball 1
   * Note: Do NOT over-torque the ball, as it will lose grip
   */
  // This value accounts for the amount of torque it takes for the ball to clear shooter wheel
  // compression
  public static final double kAntilag = 1.0; // volts now

  // ------- GOAL CONSTANTS -------- \\

  public static final double debuggingVelocity = Units.rotationsPerMinuteToRadiansPerSecond(3300);

  public static final double layupVelocity = Units.rotationsPerMinuteToRadiansPerSecond(3000);

  // Testing Voltage Values
  public static final double kDebuggingVoltage = 10; // Volts

  // Testing Current Values
  public static final double debuggingCurrent = -15; // Amps

  public static final double jugglingVelocity = Units.rotationsPerMinuteToRadiansPerSecond(1000);
}
