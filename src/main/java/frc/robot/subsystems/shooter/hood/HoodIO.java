package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    boolean motorConnected = false;
    double positionRads = 0.0;
    double positionRadsPerSec = 0.0;
    double positionDegrees = 0.0;
    double velocityDegPerSec = 0.0;
    double appliedVoltage = 0.0;
    double supplyCurrentAmps = 0.0;
    double torqueCurrentAmps = 0.0;
  }

  public static enum HoodIOOutputMode {
    COAST,
    BRAKE,
    CLOSED_LOOP
  }

  public static class HoodIOOutputs {
    public HoodIOOutputMode mode = HoodIOOutputMode.BRAKE;
    public double positionDegrees = 0.0;
    // PID Gains
    public double kP = 0.0;
    public double kD = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void applyOutputs(HoodIOOutputs outputs) {}
}
