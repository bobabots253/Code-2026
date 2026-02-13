package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends FullSubsystem {
  private HoodIO hoodIO;
  private final Alert hoodDisconnected;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();
  private final Debouncer hoodDebouncer = new Debouncer(0.2, DebounceType.kFalling);

  public HoodSubsystem(HoodIO hoodIO) {
    this.hoodIO = hoodIO;
    hoodDisconnected = new Alert("Hood Motor Disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(inputs);
    Logger.processInputs("Hood / Inputs", inputs);

    hoodDisconnected.set(
        Robot.showHardwareAlerts() && !hoodDebouncer.calculate(inputs.hoodSparkConnected));
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Hood / Outputs", outputs.mode);
    hoodIO.applyOutputs(outputs);
  }
}
