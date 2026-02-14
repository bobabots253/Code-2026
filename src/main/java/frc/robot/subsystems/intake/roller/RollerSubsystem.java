package frc.robot.subsystems.intake.roller;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.intake.roller.RollerIO.RollerIOOutputMode;
import frc.robot.subsystems.intake.roller.RollerIO.RollerIOOutputs;
import frc.robot.util.FullSubsystem;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
// lalalalalalala

public class RollerSubsystem extends FullSubsystem {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
  private final RollerIOOutputs outputs = new RollerIOOutputs();
  private final Alert RollerDisconnectedAlert = new Alert("disconnected intake rollers",AlertType.kError);
  private final Debouncer RollerDebouncer = new Debouncer(0.2,DebounceType.kFalling);


  public RollerSubsystem(RollerIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake roller log", inputs);
    RollerDisconnectedAlert.set(Robot.showHardwareAlerts() && RollerDebouncer.calculate(inputs.RollerConnected));
    
  }

    @Override
  public void periodicAfterScheduler() {
  io.updateOutputs(outputs);
  Logger.recordOutput("Intake roller log", outputs.mode);
  }

  private void runVoltage(double volts) {
    outputs.mode = RollerIOOutputMode.VOLTAGE;
    outputs.rollerSpeed = volts;
    //lalalallalalalllaalalalalalalalallalalalala
  }

  private void stop() {
    outputs.mode = RollerIOOutputMode.COAST;
    outputs.rollerSpeed = 0;
  }

  public Command stopCommand() {
    return runOnce(this:: stop);
  }

  public Command setVoltage(DoubleSupplier volts) {
    return runEnd(() -> runVoltage(volts.getAsDouble()),this:: stop);
  }
  //lalalalalalallalalalalalalala
  


}
