package frc.robot.subsystems.agitator;

import org.littletonrobotics.junction.AutoLog;

public interface AgitatorIO {
  @AutoLog
  public class AgitatorIOInputs {
    public boolean masterMotorConnected = true;

    public double masterAppliedVolts = 0.0;
    public double masterSupplyCurrentAmps = 0.0;
  }

  public static enum AgitatorIOOutputMode {
    BRAKE,
    VOLTAGE
  }

  public static class AgitatorIOOutputs {
    public AgitatorIOOutputMode mode = AgitatorIOOutputMode.BRAKE;
    public double voltage = 0.0;
  }

  default void updateInputs(AgitatorIOInputs inputs) {}

  default void applyOutputs(AgitatorIOOutputs outputs) {}
}
