package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double currentAmps = 0.0;
  }

  public static enum PivotIOOutputMode {
    IDLE,
    RUNNING,
    BRAKE;
  }

  public static class PivotIOOutputs {
    public PivotIOOutputMode mode = PivotIOOutputMode.BRAKE;
    public double positionRads = 0.0;
  }

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void applyOutputs(PivotIOOutputs outputs) {}

  public default void setPercentageOpenLoop(double decimalPercentage) {}

  public default void stop() {}
}