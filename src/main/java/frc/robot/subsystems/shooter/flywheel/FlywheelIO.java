package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean connected;
    public double flywheelRPM;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;

    public boolean followerConnected;
    public double followerSupplyCurrentAmps;
  }

  public static enum FlywheelIOOutputMode {
    COAST,
    DUTY_CYCLE_BANG_BANG,
    TORQUE_CURRENT_BANG_BANG
  }

  public static class FlywheelIOOutputs {
    public FlywheelIOOutputMode mode = FlywheelIOOutputMode.COAST;
    public double velocityRadsPerSec = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void setFlywheelRPM(FlywheelIOOutputs outputs) {}
}
