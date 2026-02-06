package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean masterMotorConnected = true;

    public double positionRads = 0.0;
    public double positionRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
  }

  public static enum HoodIOOutputMode {
    COAST,
    BRAKE,
    CLOSED_LOOP
  }

  public static class HoodIOOutputs {
    public HoodIOOutputMode mode = HoodIOOutputMode.BRAKE;
    public double positionRad = 0.0;
    public double velocityRadsPerSec = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void applyOutputs(HoodIOOutputs outputs) {}

  default void runOpenLoop(double decimalPercentage) {}

  default void stop() {}
}
