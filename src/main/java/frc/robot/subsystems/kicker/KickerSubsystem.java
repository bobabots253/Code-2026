package frc.robot.subsystems.kicker;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.kicker.KickerIO.KickerIOOutputMode;
import frc.robot.subsystems.kicker.KickerIO.KickerIOOutputs;
import frc.robot.util.FullSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class KickerSubsystem extends FullSubsystem {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
  private final KickerIOOutputs outputs = new KickerIOOutputs();

  private final Debouncer masterVortexConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private Alert masterDisconnectedAlert;

  public KickerSubsystem(KickerIO io) {
    this.io = io;

    masterDisconnectedAlert = new Alert("Kicker motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);

    masterDisconnectedAlert.set(
        Robot.showHardwareAlerts()
            && !masterVortexConnectedDebouncer.calculate(inputs.masterMotorConnected));
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Kicker/Mode", outputs.mode);
    io.applyOutputs(outputs);
  }

  private void runVelocity(double velocityRadsPerSec) {
    outputs.mode = KickerIOOutputMode.VELOCITY_SETPOINT;
    outputs.velocityRadsPerSec = velocityRadsPerSec;
  }

  // Implement this when I fix the ShotCalculator
  //   public Command runTrackTargetCommand() {
  //     return runEnd(
  //         () -> runVelocity(ShotCalculator.getInstance()...),
  //         this::stop);
  //   }

  public Command runSetVelocityCommand(DoubleSupplier velocity) {
    return runEnd(() -> runVelocity(velocity.getAsDouble()), this::stop);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  @SuppressWarnings("unused")
  private boolean isDrawingHighCurrent() {
    return Math.abs(inputs.masterSupplyCurrentAmps) > 50.0;
  }

  public double getVelocity() {
    return inputs.masterVelocityRads;
  }

  private void stop() {
    outputs.mode = KickerIOOutputMode.COAST;
    outputs.velocityRadsPerSec = 0.0;
  }
}
