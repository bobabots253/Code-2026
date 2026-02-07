package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public boolean masterMotorConnected = true;

    public double masterPositionRads = 0.0;
    public double masterVelocityRads = 0.0;
    public double masterAppliedVolts = 0.0;
    public double masterSupplyCurrentAmps = 0.0;
    public double masterTorqueCurrentAmps = 0.0;
  }

  public static enum KickerIOOutputMode {
    COAST,
    VELOCITY_SETPOINT,
    VOLTAGE
  }

  public static class KickerIOOutputs {
    public KickerIOOutputMode mode = KickerIOOutputMode.COAST;
    public double velocityRadsPerSec = 0.0;
  }

  default void updateInputs(KickerIOInputs inputs) {}

  default void applyOutputs(KickerIOOutputs outputs) {}

  default void runVelocity(double velocity) {}

  default void runVolts(double masterVolts) {}

  default void stop() {}
}
