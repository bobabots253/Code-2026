package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean masterNeoConnected = false;
    public double hoodVolts = 0.0;
    public double hoodCurrentAmps = 0.0;
    public double hoodPosRad = 0.0;
    public double hoodVelocityRad = 0.0;
  }

  public static enum HoodIOMode {
    BRAKE,
    CLOSED_LOOP_CONTROL,
    PROFILED_CONTROL,
    VOLTAGE;
  }

  public static class HoodIOOutputs {
    public HoodIOMode mode = HoodIOMode.BRAKE;
    public double hoodSetPosRad = 0.0;
    public double hoodSetVelocityRad = 0.0;
    public double hoodDecimalPercentOutput = 0.0;
  }

  default void updateInputs(HoodIOInputs inputs) {}

  default void zeroHood(HoodIOInputs inputs) {}

  default void applyOutputs(HoodIOOutputs outputs) {}

  default void setPercentVoltage(double decimalPercent) {}

  default void setClosedLoopControl(HoodIOOutputs outputs) {}

  default void setProfiledControl(HoodIOOutputs outputs) {}
}
