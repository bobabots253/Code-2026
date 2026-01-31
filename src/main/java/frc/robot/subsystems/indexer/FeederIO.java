package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public boolean feederConnected = false;
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;
  }

  public default void updateInputs(FeederIOInputs inputs) {}
  public default void setFeederOpenLoop(double output) {}
}
