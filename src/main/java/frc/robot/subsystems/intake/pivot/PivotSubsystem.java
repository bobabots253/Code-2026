package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.pivot.PivotIO.PivotIOOutputMode;
import frc.robot.subsystems.intake.pivot.PivotIO.PivotIOOutputs;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.util.FullSubsystem;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends FullSubsystem {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final PivotIOOutputs outputs = new PivotIOOutputs();

  private Alert masterDisconnected;

  @RequiredArgsConstructor
  public enum Goal {
    IDLE(() -> 0.0),
    DEPLOYED(() -> PivotConstants.deployedAngle),
    STOW(() -> PivotConstants.stowAngle), // Change if Necessary
    JUGGLE(() -> HoodConstants.jugglingAngle),
    DEBUGGING(() -> HoodConstants.debuggingAngle);

    // Required Arguement for each enum state
    private final DoubleSupplier angleRads;

    /** Returns the current target angle for this goal state. */
    private double getGoal() {
      return angleRads.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Pivot/Goal")
  private Goal currentGoal = Goal.IDLE;

  public PivotSubsystem(PivotIO io) {
    this.io = io;

    masterDisconnected = new Alert("Pivot motor disconnected!", Alert.AlertType.kWarning);

    setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Pivot Idle"));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    // Force IDLE state if the robot is disabled so it doesn't snap to last hood angle on enable
    if (DriverStation.isDisabled()) {
      setGoal(Goal.IDLE);
    }

    // Re-poll the supplier every loop to handle new shot calculations
    if (currentGoal == Goal.IDLE) {
      stop();
    } else {
      runAngular(currentGoal.getGoal());
    }
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Pivot/Mode", outputs.mode);
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

  @AutoLogOutput(key = "Pivot/AtGoal")
  /**
   * Returns true if the pivot is within the angular tolerance. Note: IDLE check catches the
   * exception because I'm lazy
   */
  public boolean atGoal() {
    return currentGoal == Goal.IDLE
        || Math.abs(getAngle() - currentGoal.getGoal()) <= HoodConstants.closedLoopAngularTolerance;
  }

  /**
   * Update the io for angular closed loop control and applies the new angular setpoint
   *
   * @param angleRads the new angular setpoint.
   */
  private void runAngular(double angleRads) {
    outputs.mode = PivotIOOutputMode.CLOSED_LOOP;
    outputs.positionRad = angleRads;
  }

  @AutoLogOutput(key = "Pivot/MeasuredAngleRads")
  public double getAngle() {
    return inputs.masterPositionRads;
  }

  @SuppressWarnings("unused")
  private boolean isDrawingHighCurrent() {
    return Math.abs(inputs.masterSupplyCurrentAmps) > 50.0;
  }

  public Command deployCommand() {
    return startEnd(() -> setGoal(Goal.DEPLOYED), () -> setGoal(Goal.IDLE))
        .withName("Pivot Deploy");
  }

  public Command stowCommand() {
    return startEnd(() -> setGoal(Goal.STOW), () -> setGoal(Goal.IDLE)).withName("Pivot Stow");
  }

  public Command juggleCommand() {
    return startEnd(() -> setGoal(Goal.JUGGLE), () -> setGoal(Goal.IDLE)).withName("Pivot Juggle");
  }

  public Command runDebuggingCommand() {
    return startEnd(() -> setGoal(Goal.DEBUGGING), () -> setGoal(Goal.IDLE))
        .withName("Pivot Debug");
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  /**
   * Manual override command to run a specific velocity. Note: This bypasses the "state machine".
   */
  public Command runSetAngularCommand(DoubleSupplier angle) {
    return runEnd(() -> runAngular(angle.getAsDouble()), this::stop);
  }

  private void stop() {
    outputs.mode = PivotIOOutputMode.BRAKE;
  }
}
