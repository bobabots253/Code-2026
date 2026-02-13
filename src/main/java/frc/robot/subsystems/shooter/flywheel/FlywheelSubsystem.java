package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.fieldSetup;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import frc.robot.util.FullSubsystem;

import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends FullSubsystem {
  private final FlywheelIO io;
  private final Alert flywheelMasterDisconnectedAlert;
  private final Alert flywheelFollowerDisconnectedAlert;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();
  private final Debouncer flywheelDebouncer = new Debouncer(0.2,DebounceType.kFalling);

  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;
    flywheelMasterDisconnectedAlert = new Alert("Disconnected Master motor in flywheel", AlertType.kError);
    flywheelFollowerDisconnectedAlert = new Alert("Disconnected Follower motor in flywheel", AlertType.kError);

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel / Inputs", inputs);
    flywheelMasterDisconnectedAlert.set(
    Robot.showHardwareAlerts() && flywheelDebouncer.calculate(inputs.flywheelMasterConnected));
    flywheelFollowerDisconnectedAlert.set(
    Robot.showHardwareAlerts() && flywheelDebouncer.calculate(inputs.flywheelFollowerConnected));
  }

  @Override
  public void periodicAfterScheduler(){
    Logger.recordOutput("Flywheel / Outputs", outputs.mode);
    io.applyOutputs(outputs);

  }
}
