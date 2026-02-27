package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean masterMotorConnected = true;

    public double masterPositionRads = 0.0;
    public double masterVelocityRads = 0.0;
    public double masterAppliedVolts = 0.0;
    public double masterSupplyCurrentAmps = 0.0;
  }

  public static enum HoodIOOutputMode {
    BRAKE,
    CLOSED_LOOP,
    VOLTAGE
  }

  public static class HoodIOOutputs {
    public HoodIOOutputMode mode = HoodIOOutputMode.BRAKE;
    public double positionRad = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double voltage = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void applyOutputs(HoodIOOutputs outputs) {}

  default void runOpenLoop(double decimalPercentage) {}
}
