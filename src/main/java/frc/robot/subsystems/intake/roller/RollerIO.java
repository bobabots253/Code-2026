package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public class RollerIOInputs {
    public boolean masterMotorConnected = true;

    public double masterAppliedVolts = 0.0;
  }

  public static enum RollerIOOutputMode {
    BRAKE,
    VOLTAGE
  }

  public static class RollerIOOutputs {
    public RollerIOOutputMode mode = RollerIOOutputMode.BRAKE;
    public double voltage = 0.0;
  }

  default void updateInputs(RollerIOInputs inputs) {}

  default void applyOutputs(RollerIOOutputs outputs) {}
}
