package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public class RollerIOInputs {
    // public boolean rollerConnected = false;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;
  }

  public default void updateInputs(RollerIOInputs inputs) {}

  public default void setRollerOpenLoop(double voltage) {}
}
