package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputMode;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import frc.robot.util.FullSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends FullSubsystem {

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  private Alert masterDisconnected;
  private Alert followerDisconnected;

  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;

    masterDisconnected = new Alert("Master flywheel disconnected!", Alert.AlertType.kWarning);
    followerDisconnected = new Alert("Follower flywheel disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    masterDisconnected.set(Robot.showHardwareAlerts() && (!inputs.masterMotorConnected));
    followerDisconnected.set(Robot.showHardwareAlerts() && (!inputs.followerMotorConnected));
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Flywheel/Mode", outputs.mode);
    io.applyOutputs(outputs);
  }

  private void runVelocity(double velocityRadsPerSec) {
    outputs.mode = FlywheelIOOutputMode.VELOCITY_SETPOINT;
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
    return Math.abs(inputs.masterSupplyCurrentAmps) > 50.0
        || Math.abs(inputs.followerSupplyCurrentAmps) > 50.0;
  }

  public double getVelocity() {
    return inputs.masterVelocityRads;
  }

  private void stop() {
    outputs.mode = FlywheelIOOutputMode.COAST;
    outputs.velocityRadsPerSec = 0.0;
  }
}
