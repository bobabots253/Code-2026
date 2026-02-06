package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final rollerIO io;
  private final IntakeIO pivotIo;
  private final Alert pivotDisconnectedAlert;
  private final Alert rollerDisconnectedAlert;
  private final IntakeIOInputsAutoLogged pivotInputs = new IntakeIOInputsAutoLogged();
  private final rollerIOInputsAutoLogged rollerInputs = new rollerIOInputsAutoLogged();

  public IntakeSubsystem(rollerIO io, IntakeIO pivotIo) {
    this.io = io;
    this.pivotIo = pivotIo;
    pivotDisconnectedAlert = new Alert("Disconnected pivot motor in intake", AlertType.kError);
    rollerDisconnectedAlert = new Alert("Disconnected roller motor in intake", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(rollerInputs);
    pivotIo.updateInputs(pivotInputs);
    Logger.processInputs("Pivot", pivotInputs);
    Logger.processInputs("Intake Rollers", rollerInputs);
    pivotDisconnectedAlert.set(!pivotInputs.intakeConnected);
    rollerDisconnectedAlert.set(!rollerInputs.rollerConnected);
  }

  // these commands should be used to run the intake rollers forward (intake), and backward (extake)
  // research command syntax and then check back with me (Finn), when you think you have code that
  // works.
  // you will need to return something that isn't null. That's just a placeholder so that there are
  // no errors in code.

  // for Jasmine and Jessika
  public Command intake() {
    return null;
  }

  public Command extake() {
    return null;
  }

  // these commands should extend and retract the pivot. Also research command syntax and whatnot.
  // for Yo
  public Command extendPivot() {
    return null;
  }

  public Command retractPivot() {
    return null;
  }
}
