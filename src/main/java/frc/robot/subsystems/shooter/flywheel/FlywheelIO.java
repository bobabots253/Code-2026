package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean masterMotorConnected = true;
    public boolean followerMotorConnected = true;

    public double masterVelocityRads = 0.0;
    public double masterAppliedVolts = 0.0;
    public double masterSupplyCurrentAmps = 0.0;
    public double masterTempCelsius = 0.0;
    public double masterAccelerationRadPerSecSquared = 0.0;

    public double followerVelocityRads = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerTempCelsius = 0.0;
    public double followerAccelerationRadPerSecSquared = 0.0;
  }

  public static enum FlywheelIOOutputMode {
    COAST,
    VELOCITY_SETPOINT,
    VOLTAGE,
    CURRENT
  }

  public static class FlywheelIOOutputs {
    @AutoLogOutput(key = "FlywheelOutputs/Mode")
    public FlywheelIOOutputMode mode = FlywheelIOOutputMode.COAST;

    @AutoLogOutput(key = "FlywheelOutputs/VelocityRadPerSec")
    public double velocityRadsPerSec = 0.0;

    @AutoLogOutput(key = "FlywheelOutputs/MeasuredVelocityRadPerSec")
    public double measuredVelocityRadPerSec = 0.0;

    @AutoLogOutput public double calculatedAcceleration = 0.0;

    @AutoLogOutput(key = "FlywheelOutputs/Voltage")
    public double voltage = 0.0;

    @AutoLogOutput(key = "FlywheelOutputs/Current")
    public double current = 0.0;

    @AutoLogOutput(key = "FlywheelOutputs/isWarm")
    public boolean isWarm = false;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void applyOutputs(FlywheelIOOutputs outputs) {}

  default void runVelocity(double velocity) {}

  default void runVolts(double masterVolts) {}

  default void stop() {}
}
