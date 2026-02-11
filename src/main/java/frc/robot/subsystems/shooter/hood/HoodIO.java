package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
  public boolean hoodSparkConnected = false;
  public double hoodCurrentAmps = 0.0;
  public double hoodCurrentTorque = 0.0;
  public double hoodPosRad = 0.0;
  public double hoodVelocityRad = 0.0;
  public double hoodPosDeg = 0.0;
}

  public static enum HoodIOStates {
    BRAKE_MODE,
    COAST_MODE, 
    CLOSED_LOOP_CONTROL;
  }

  public static class HoodIOOutputs {
  public double hoodSetPosRad = 0.0;
  }
  
  default void updateInputs() {}
  default void applyOutputs() {}
  default void setPercentVoltage (double decimalPercent) {}
  default void setPosition (double setpoint) {}
  default void stop () {}
}
