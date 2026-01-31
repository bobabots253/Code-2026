package frc.robot.subsystems.intake;

public interface IntakeIO {
  public static class IntakeIOInputs {
    public boolean intakeConnected = false;
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
  public default void setIntakeOpenLoop(double output) {}
}
