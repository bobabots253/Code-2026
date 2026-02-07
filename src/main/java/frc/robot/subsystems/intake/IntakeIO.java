package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {

    public boolean intakeConnected = true;
    public double positionRad = 0.0;
    public double velocityRad = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public static enum intakeIOOutputMode {
    COAST,
    VOLTAGE
  }

  public class IntakeIOOutputs {
    public intakeIOOutputMode mode = intakeIOOutputMode.COAST;
    public double velocityRadiansPerSecond = (0.0);
  }

  public default void updateOutputs(IntakeIOOutputs outputs) {}

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakePosition(double setpoint) {}
}
