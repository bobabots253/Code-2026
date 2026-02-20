package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOOutputMode;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOOutputs;
import frc.robot.util.FullSubsystem;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends FullSubsystem {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final IndexerIOOutputs outputs = new IndexerIOOutputs();

  private Alert masterDisconnected;

  @RequiredArgsConstructor
  public enum Goal {
    IDLE(() -> 0.0),
    INDEXING(() -> IndexerConstants.indexingVolts),
    STOW(() -> IndexerConstants.stowVolts), // Change if Necessary
    JUGGLE(() -> IndexerConstants.jugglingVolts),
    DEBUGGING(() -> IndexerConstants.debuggingVolts);

    // Required Arguement for each enum state
    private final DoubleSupplier voltage;

    /** Returns the current target voltage for this goal state. */
    private double getGoal() {
      return voltage.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Indexer/Goal")
  private Goal currentGoal = Goal.IDLE;

  public IndexerSubsystem(IndexerIO io) {
    this.io = io;

    masterDisconnected = new Alert("Indexer motor disconnected!", Alert.AlertType.kWarning);

    setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Indexer Idle"));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    masterDisconnected.set(Robot.showHardwareAlerts() && (!inputs.masterMotorConnected));

    // Force IDLE state if the robot is disabled
    if (DriverStation.isDisabled()) {
      setGoal(Goal.IDLE);
    }

    // Re-poll the supplier every loop to handle new updates
    if (currentGoal == Goal.IDLE) {
      stop();
    } else {
      runVoltage(currentGoal.getGoal());
    }
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Indexer/Mode", outputs.mode);
    io.applyOutputs(outputs);
  }

  /**
   * Sets the current goal state for the subsystem.
   *
   * @param desiredGoal The new goal to "transition" to.
   */
  private void setGoal(Goal desiredGoal) {
    this.currentGoal = desiredGoal;
  }

  /**
   * Update the io for voltage open loop control
   *
   * @param voltage the new voltage output request.
   */
  private void runVoltage(double voltage) {
    outputs.mode = IndexerIOOutputMode.VOLTAGE;
    outputs.voltage = voltage;
  }

  @AutoLogOutput(key = "Indexer/MeasuredVoltage")
  public double getVoltage() {
    return inputs.masterAppliedVolts;
  }

  public Command indexCommand() {
    return startEnd(() -> setGoal(Goal.INDEXING), () -> setGoal(Goal.IDLE))
        .withName("Indexer Index");
  }

  public Command stowCommand() {
    return startEnd(() -> setGoal(Goal.STOW), () -> setGoal(Goal.IDLE)).withName("Indexer Stow");
  }

  public Command juggleCommand() {
    return startEnd(() -> setGoal(Goal.JUGGLE), () -> setGoal(Goal.IDLE))
        .withName("Indexer Juggle");
  }

  public Command runDebuggingCommand() {
    return startEnd(() -> setGoal(Goal.DEBUGGING), () -> setGoal(Goal.IDLE))
        .withName("Indexer Debug");
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  /** Manual override command to run a specific voltage. Note: This bypasses the "state machine". */
  public Command runSetVoltageCommand(DoubleSupplier voltage) {
    return runEnd(() -> runVoltage(voltage.getAsDouble()), this::stop);
  }

  private void stop() {
    outputs.mode = IndexerIOOutputMode.BRAKE;
  }
}
