package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean masterMotorConnected = true;
    public boolean followerMotorConnected = true;

    public double masterPositionRads = 0.0;
    public double masterVelocityRpm = 0.0;
    public double masterAppliedVolts = 0.0;
    public double masterSupplyCurrentAmps = 0.0;
    public double masterTorqueCurrentAmps = 0.0;

    public double followerPositionRads = 0.0;
    public double followerVelocityRpm = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerTorqueCurrentAmps = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void runRPMSetpoint(double masterRPM) {}

  default void runVolts(double masterVolts) {}

  default void stop() {}
}
