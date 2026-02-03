package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public class FlywheelIOInputs {
    public boolean flywheelConnected = false;
    public double flywheelAppliedVolts = 0.0;
    public double flywheelCurrentAmps = 0.0;
    public double flywheelVelocity = 0.0;
    public boolean kickerConnected = false;
    public double kickerAppliedVolts = 0.0;
    public double kickerCurrentAmps = 0.0;
    public double kickerVelocity = 0.0;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setRPM(double velocity) {}

  public default void setKickerRPM(double velocity) {}
}
