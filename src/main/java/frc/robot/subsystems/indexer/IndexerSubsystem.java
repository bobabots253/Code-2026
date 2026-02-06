public class IndexerSubsystem extends SubsystemBase{
    private final AggitatorIO aggitatorio;
private final FeederIO feederio;
private final Alert agitatorDisconnectedAlert;
private final Alert feederDisconnectedAlert;
private final AggitatorIOInputsAutoLogged aggitatorInputs = new AggitatorIOInputsAutoLogged();
private final FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();
public IndexerSubsystem(FeederIo feederio,AggitatorIo aggitatorio){
this.feederio = feederio;
this.aggitatorio = aggitatorio;
agitatorDisconnectedAlert =
new Alert("Disconnected aggitator motor in indexter",AlertType.kError);
feederDisconnectedAlert =
new Alert("Disconnected feeder motor in indexter",AlertType.kError);



}






}