package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean hoodConnected = false;
    public double hoodPosition = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodCurrentAmps = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setHoodPos(double setAngle) {}
}
