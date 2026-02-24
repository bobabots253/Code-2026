package frc.robot.subsystems.intake.roller;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.intake.roller.RollerIO.RollerIOOutputMode;
import frc.robot.subsystems.intake.roller.RollerIO.RollerIOOutputs;
import frc.robot.util.FullSubsystem;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RollerSubsystem extends FullSubsystem {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
  private final RollerIOOutputs outputs = new RollerIOOutputs();

  private Alert masterDisconnected;

  @RequiredArgsConstructor
  public enum Goal {
    IDLE(() -> 0.0),
    DEPLOYED(() -> RollerConstants.intakingVolts),
    STOW(() -> RollerConstants.stowVolts), // Change if Necessary
    JUGGLE(() -> RollerConstants.jugglingVolts),
    DEBUGGING(() -> RollerConstants.debuggingVolts);

    // Required Arguement for each enum state
    private final DoubleSupplier voltage;

    /** Returns the current target voltage for this goal state. */
    private double getGoal() {
      return voltage.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Roller/Goal")
  private Goal currentGoal = Goal.IDLE;

  public RollerSubsystem(RollerIO io) {
    this.io = io;

    masterDisconnected = new Alert("Roller motor disconnected!", Alert.AlertType.kWarning);

    setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Roller Idle"));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);

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
    Logger.recordOutput("Roller/Mode", outputs.mode);
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
    outputs.mode = RollerIOOutputMode.VOLTAGE;
    outputs.voltage = voltage;
  }

  @AutoLogOutput(key = "Roller/MeasuredVoltage")
  public double getVoltage() {
    return inputs.masterAppliedVolts;
  }

  public Command intakeCommand() {
    return startEnd(() -> setGoal(Goal.DEPLOYED), () -> setGoal(Goal.IDLE))
        .withName("Roller Deploy");
  }

  public Command simpleDeployCommand() {
    return run(() -> io.simpleVoltage(RollerConstants.intakingVolts));
  }

  public Command simpleStowCommand() {
    return run(() -> io.simpleVoltage(RollerConstants.stowVolts));
  }

  public Command stowCommand() {
    return startEnd(() -> setGoal(Goal.STOW), () -> setGoal(Goal.IDLE)).withName("Roller Stow");
  }

  public Command juggleCommand() {
    return startEnd(() -> setGoal(Goal.JUGGLE), () -> setGoal(Goal.IDLE)).withName("Roller Juggle");
  }

  public Command runDebuggingCommand() {
    return startEnd(() -> setGoal(Goal.DEBUGGING), () -> setGoal(Goal.IDLE))
        .withName("Roller Debug");
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  /** Manual override command to run a specific voltage. Note: This bypasses the "state machine". */
  public Command runSetVoltageCommand(DoubleSupplier voltage) {
    return runEnd(() -> runVoltage(voltage.getAsDouble()), this::stop);
  }

  private void stop() {
    outputs.mode = RollerIOOutputMode.BRAKE;
  }
}
