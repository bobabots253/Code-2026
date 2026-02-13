package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean hoodSparkConnected = false;
    public double hoodVolts = 0.0;
    public double hoodCurrentAmps = 0.0;
    public double hoodPosRad = 0.0;
    public double hoodVelocityRad = 0.0;
    public double hoodPosDeg = 0.0;
  }

  public static enum HoodIOMode {
    BRAKE,
    CLOSED_LOOP_CONTROL,
    VOLTAGE;
  }

  public static class HoodIOOutputs {
    public HoodIOMode mode = HoodIOMode.BRAKE;
    public double hoodSetPosDeg = 0.0;
  }

  default void updateInputs(HoodIOInputsAutoLogged inputs) {}

  default void applyOutputs(HoodIOOutputs outputs) {}

  default void setPercentVoltage(double decimalPercent) {}

  default void setPosition(double setpoint) {}
}
