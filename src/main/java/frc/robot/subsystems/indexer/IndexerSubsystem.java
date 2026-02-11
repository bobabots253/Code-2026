package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {
  private final AggitatorIO aggitatorio;
  private final FeederIO feederio;
  private final Alert agitatorDisconnectedAlert;
  private final Alert feederDisconnectedAlert;
  private final AggitatorIOInputsAutoLogged aggitatorInputs = new AggitatorIOInputsAutoLogged();
  private final FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();

  public IndexerSubsystem(FeederIO feederio, AggitatorIO aggitatorio) {
    this.feederio = feederio;
    this.aggitatorio = aggitatorio;
    agitatorDisconnectedAlert = new Alert("Disconnected Aggitator motor in indexter", AlertType.kError);
    feederDisconnectedAlert = new Alert("Disconnected Feeder motor in indexter", AlertType.kError);
  }
    @Override
  public void periodic() {
    feederio.updateInputs(feederInputs);
    aggitatorio.updateInputs(aggitatorInputs);
    Logger.processInputs("Aggitator", aggitatorInputs);
    Logger.processInputs("Feeder", feederInputs);
    agitatorDisconnectedAlert.set(!aggitatorInputs.AggitatorConnected);
    feederDisconnectedAlert.set(!feederInputs.feederConnected);
  }
public Command runFeeder;
  
}
