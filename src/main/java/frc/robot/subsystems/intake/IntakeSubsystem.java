package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final rollerIO io;
  private final IntakeIO pivotIo;
  private final Alert pivotDisconnectedAlert;
  private final Alert rollerDisconnectedAlert;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(rollerIO io, IntakeIO pivotIo) {}
}
