package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public boolean masterMotorConnected = true;

    public double masterPositionRads = 0.0;
    public double masterVelocityRads = 0.0;
    public double masterAppliedVolts = 0.0;
    public double masterSupplyCurrentAmps = 0.0;
  }

  public static enum PivotIOOutputMode {
    BRAKE,
    CLOSED_LOOP
  }

  public static class PivotIOOutputs {
    public PivotIOOutputMode mode = PivotIOOutputMode.BRAKE;
    public double positionRad = 0.0;
  }

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void applyOutputs(PivotIOOutputs outputs) {}

  public default void stop() {}
}
