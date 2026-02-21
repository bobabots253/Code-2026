package frc.robot.subsystems.agitator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.agitator.AgitatorIO.AgitatorIOOutputMode;
import frc.robot.subsystems.agitator.AgitatorIO.AgitatorIOOutputs;
import frc.robot.util.FullSubsystem;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AgitatorSubsystem extends FullSubsystem {
  private final AgitatorIO io;
  private final AgitatorIOInputsAutoLogged inputs = new AgitatorIOInputsAutoLogged();
  private final AgitatorIOOutputs outputs = new AgitatorIOOutputs();

  private Alert masterDisconnected;

  @RequiredArgsConstructor
  public enum Goal {
    IDLE(() -> 0.0),
    INDEXING(() -> AgitatorConstants.indexingVolts),
    PASSIVE(() -> AgitatorConstants.passiveVolts),
    INTAKING(() -> AgitatorConstants.intakingVolts),
    JUGGLE(() -> AgitatorConstants.jugglingVolts),
    DEBUGGING(() -> AgitatorConstants.debuggingVolts);

    // Required Arguement for each enum state
    private final DoubleSupplier voltage;

    /** Returns the current target voltage for this goal state. */
    private double getGoal() {
      return voltage.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Agitator/Goal")
  private Goal currentGoal = Goal.IDLE;

  public AgitatorSubsystem(AgitatorIO io) {
    this.io = io;

    masterDisconnected = new Alert("Agitator motor disconnected!", Alert.AlertType.kWarning);

    setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Agitator Idle"));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Agitator", inputs);

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
    Logger.recordOutput("Agitator/Mode", outputs.mode);
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
    outputs.mode = AgitatorIOOutputMode.VOLTAGE;
    outputs.voltage = voltage;
  }

  @AutoLogOutput(key = "Agitator/MeasuredVoltage")
  public double getVoltage() {
    return inputs.masterAppliedVolts;
  }

  public Command indexCommand() {
    return startEnd(() -> setGoal(Goal.INDEXING), () -> setGoal(Goal.IDLE))
        .withName("Agitator Index");
  }

  public Command passiveCommand() {
    return startEnd(() -> setGoal(Goal.PASSIVE), () -> setGoal(Goal.IDLE))
        .withName("Agitator Passive");
  }

  public Command intakeCommand() {
    return startEnd(() -> setGoal(Goal.INTAKING), () -> setGoal(Goal.IDLE))
        .withName("Agitator Intaking");
  }

  public Command juggleCommand() {
    return startEnd(() -> setGoal(Goal.JUGGLE), () -> setGoal(Goal.IDLE))
        .withName("Agitator Juggle");
  }

  public Command runDebuggingCommand() {
    return startEnd(() -> setGoal(Goal.DEBUGGING), () -> setGoal(Goal.IDLE))
        .withName("Agitator Debug");
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  /** Manual override command to run a specific voltage. Note: This bypasses the "state machine". */
  public Command runSetVoltageCommand(DoubleSupplier voltage) {
    return runEnd(() -> runVoltage(voltage.getAsDouble()), this::stop);
  }

  private void stop() {
    outputs.mode = AgitatorIOOutputMode.BRAKE;
  }
}
