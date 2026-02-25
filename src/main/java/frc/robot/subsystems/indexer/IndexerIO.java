package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public class IndexerIOInputs {
    public boolean masterMotorConnected = true;

    public double masterAppliedVolts = 0.0;
    public double masterSupplyCurrentAmps = 0.0;
  }

  public static enum IndexerIOOutputMode {
    BRAKE,
    VOLTAGE,
    CURRENT
  }

  public static class IndexerIOOutputs {
    public IndexerIOOutputMode mode = IndexerIOOutputMode.BRAKE;
    public double voltage = 0.0;
    public double current = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void applyOutputs(IndexerIOOutputs outputs) {}
}
