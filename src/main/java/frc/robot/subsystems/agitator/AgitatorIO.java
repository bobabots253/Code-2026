package frc.robot.subsystems.agitator;

import org.littletonrobotics.junction.AutoLog;

public interface AgitatorIO {
  @AutoLog
  public static class AgitatorInputs {
    public boolean masterMotorConnected = true;
    public double masterAppliedVolts = 0.00;
    public double masterSupplyCurrentAmps = 0.00;
  }

  public static enum AgitatiorIOOutputMode {
    BRAKE,
    VOLTAGE
  }

  public static class AgitatorIOOutputs {
    public double voltage = 0.0;
    public AgitatiorIOOutputMode mode = AgitatiorIOOutputMode.BRAKE;
  }

  public default void updateInputs(AgitatorInputs inputs) {}

  public default void applyOutputs(AgitatorIOOutputs outputs) {}
}