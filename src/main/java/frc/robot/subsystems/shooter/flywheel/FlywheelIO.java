package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean masterMotorConnected = true;
    public boolean followerMotorConnected = true;

    public double masterPositionRads = 0.0;
    public double masterVelocityRads = 0.0;
    public double masterAppliedVolts = 0.0;
    public double masterSupplyCurrentAmps = 0.0;
    public double masterTorqueCurrentAmps = 0.0;

    public double followerPositionRads = 0.0;
    public double followerVelocityRads = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerTorqueCurrentAmps = 0.0;
  }

  public static enum FlywheelIOOutputMode {
    COAST,
    VELOCITY_SETPOINT
  }

  public static class FlywheelIOOutputs {
    public FlywheelIOOutputMode mode = FlywheelIOOutputMode.COAST;
    public double velocityRadsPerSec = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void applyOutputs(FlywheelIOOutputs outputs) {}

  default void runVelocity(double velocity) {}

  default void runVolts(double masterVolts) {}

  default void stop() {}
}
