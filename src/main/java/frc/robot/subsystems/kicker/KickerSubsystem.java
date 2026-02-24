package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.kicker.KickerIO.KickerIOOutputMode;
import frc.robot.subsystems.kicker.KickerIO.KickerIOOutputs;
import frc.robot.util.FullSubsystem;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class KickerSubsystem extends FullSubsystem {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
  private final KickerIOOutputs outputs = new KickerIOOutputs();

  private Alert masterDisconnectedAlert;

  @RequiredArgsConstructor
  /*
   * Defines all possible states for the kicker. Each goal state has a DoubleSupplier arguement that updates from Robot State
   */

  // ADD MVP STATE
  public enum Goal {
    // Stop the kicker and chooses COAST Mode
    IDLE(() -> 0.0),
    // Velocity setpoint calculated by the ShotCalculator for the Hub.
    PREPARE_HUB(() -> KickerConstants.prepareHubVelocity),
    // Currently mirrors PREPARE_HUB but I added it so it can edited/tuned separately
    SHOOT(() -> KickerConstants.shootingVelocity), // Change if Necessary
    // Low-speed state for juggling them balls into our own hopper LOL
    JUGGLE(() -> KickerConstants.jugglingVelocity),
    // Static speed state for subsystem testing
    DEBUGGING(() -> KickerConstants.debuggingVelocity),

    INDEXING(() -> KickerConstants.indexingVoltage);
    // Required Arguement for each enum state
    private final DoubleSupplier velocityRadsPerSec; // might be volts, who knows
    /** Returns the current target velocity for this goal state. */
    private double getGoal() {
      return velocityRadsPerSec.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Kicker/Goal")
  private Goal currentGoal = Goal.IDLE;

  public KickerSubsystem(KickerIO io) {
    this.io = io;

    masterDisconnectedAlert = new Alert("Kicker motor disconnected!", Alert.AlertType.kWarning);

    setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Kicker Idle"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);

    masterDisconnectedAlert.set(Robot.showHardwareAlerts() && (!inputs.masterMotorConnected));

    // Force IDLE state if the robot is disabled so it doesn't jump rpms on enable
    if (DriverStation.isDisabled()) {
      setGoal(Goal.IDLE);
    }
    // Re-poll the supplier every loop to handle new shot calculations
    if (currentGoal == Goal.IDLE) {
      stop();
    } else {
      runVoltage(currentGoal.getGoal());
    }
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Kicker/Mode", outputs.mode);
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

  @AutoLogOutput(key = "Kicker/AtGoal")
  /**
   * Returns true if the kicker is within the velocity tolerance. Note: IDLE check catches the
   * exception because I'm lazy
   */
  public boolean atGoal() {
    return currentGoal == Goal.IDLE
        || Math.abs(getVelocity() - currentGoal.getGoal())
            <= KickerConstants.closedLoopVelocityTolerance;
  }

  public Command shootCommand() {
    return startEnd(() -> setGoal(Goal.SHOOT), () -> setGoal(Goal.IDLE)).withName("Kicker Shoot");
  }

  public Command prepareHubCommand() {
    return startEnd(() -> setGoal(Goal.PREPARE_HUB), () -> setGoal(Goal.IDLE))
        .withName("Kicker Prepare Hub");
  }

  public Command indexCommand() {
    return startEnd(() -> setGoal(Goal.INDEXING), () -> setGoal(Goal.IDLE))
        .withName("Kicker Index");
  }

  public Command juggleCommand() {
    return startEnd(() -> setGoal(Goal.JUGGLE), () -> setGoal(Goal.IDLE)).withName("Kicker Juggle");
  }

  public Command runDebuggingCommand() {
    return startEnd(() -> setGoal(Goal.DEBUGGING), () -> setGoal(Goal.IDLE))
        .withName("Kicker Debug");
  }

  /**
   * Update the io for velocity closed loop control and applies the new velocity setpoint
   *
   * @param velocityRadsPerSec the new velocity setpoint.
   */
  private void runVelocity(double velocityRadsPerSec) {
    outputs.mode = KickerIOOutputMode.VELOCITY_SETPOINT;
    outputs.velocityRadsPerSec = velocityRadsPerSec;
  }

  private void runVoltage(double volts) {
    outputs.mode = KickerIOOutputMode.VOLTAGE;
    outputs.voltage = volts;
  }

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

  @AutoLogOutput(key = "Kicker/MeasuredVelocity")
  public double getVelocity() {
    return inputs.masterVelocityRads;
  }

  private void stop() {
    outputs.mode = KickerIOOutputMode.COAST;
    outputs.velocityRadsPerSec = 0.0;
  }
}
