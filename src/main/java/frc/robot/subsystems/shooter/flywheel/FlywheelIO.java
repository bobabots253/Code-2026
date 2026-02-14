package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean flywheelMasterConnected = false;
    public double flywheelMasterVolts = 0.0;
    public double flywheelMasterCurrentAmps = 0.0;
    public double flywheelMasterTorqueCurrent = 0.0;
    public double flywheelMasterVelocityRad = 0.0;
    public double flywheelMasterPosRad = 0.0;

    public boolean flywheelFollowerConnected = false;
    public double flywheelFollowerVolts = 0.0;
    public double flywheelFollowerCurrentAmps = 0.0;
    public double flywheelFollowerTorqueCurrent = 0.0;
    public double flywheelFollowerVelocityRad = 0.0;
    public double flywheelFollowerPosRad = 0.0;
  }

  public static enum FlywheelOutputMode {
    BANG_BANG,
    VOLTAGE,
    COAST;
  }

  public static class FlywheelIOOutputs {
    double velocityRadsPerSec = 0.0;
    FlywheelOutputMode mode = FlywheelOutputMode.COAST;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void applyOutputs(FlywheelIOOutputs outputs) {}

  default void runVolts(double volts) {}

  default void runVelocity(double velocity) {}

  default void stop() {}
}
