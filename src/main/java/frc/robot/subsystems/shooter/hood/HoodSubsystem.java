package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputMode;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import frc.robot.util.FullSubsystem;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends FullSubsystem {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  private Alert masterDisconnected;

  @RequiredArgsConstructor
  /*
   * Defines all possible states for the hood. Each goal state has a DoubleSupplier arguement that updates from Robot State
   */
  // ADD MVP STATE
  public enum Goal {
    // Stop ClosedLoopControl on the Hood, Remains in Brake Mode
    IDLE(() -> 0.0),
    // Angular setpoint calculated by the ShotCalculator for the Hub.
    PREPARE_HUB(() -> RobotState.getInstance().getCustomShotData().correctedTargetAngle()),
    // Currently mirrors PREPARE_HUB but I added it so it can edited/tuned separately
    SHOOT(
        () ->
            RobotState.getInstance()
                .getCustomShotData()
                .correctedTargetAngle()), // Change if Necessary
    // Static angle for juggling them balls into our own hopper LOL
    JUGGLE(() -> HoodConstants.jugglingAngle),
    // Static angular state for subsystem testing
    DEBUGGING(() -> HoodConstants.debuggingAngle),
    DEBUGGING_VOLT_UP(() -> HoodConstants.kDebuggingVoltageUP),
    DEBUGGING_VOLT_DOWN(() -> HoodConstants.kDebuggingVoltageDOWN),
    STATIC(() -> HoodConstants.staticAngle);

    // Required Arguement for each enum state
    private final DoubleSupplier angleRads;

    /** Returns the current target angle for this goal state. */
    private double getGoal() {
      return angleRads.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Hood/Goal")
  private Goal currentGoal = Goal.IDLE;

  public HoodSubsystem(HoodIO io) {
    this.io = io;

    masterDisconnected = new Alert("Hood motor disconnected!", Alert.AlertType.kWarning);

    setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Hood Idle"));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    masterDisconnected.set(Robot.showHardwareAlerts() && (!inputs.masterMotorConnected));

    // Force IDLE state if the robot is disabled so it doesn't snap to last hood angle on enable
    if (DriverStation.isDisabled()) {
      setGoal(Goal.IDLE);
    }

    // Re-poll the supplier every loop to handle new shot calculations
    if (currentGoal == Goal.IDLE) {
      stop();
    } else if (currentGoal == Goal.DEBUGGING_VOLT_UP || currentGoal == Goal.DEBUGGING_VOLT_DOWN) {
      runVoltage(currentGoal.getGoal());
    } else {
      runAngular(currentGoal.getGoal());
    }
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Hood/Mode", outputs.mode);
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

  @AutoLogOutput(key = "Hood/AtGoal")
  /**
   * Returns true if the hood is within the angular tolerance. Note: IDLE check catches the
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
    outputs.mode = HoodIOOutputMode.CLOSED_LOOP;
    outputs.positionRad = angleRads;
  }

  /**
   * Update the io for voltage control and applies the new voltage setpoint
   *
   * @param voltage the new voltage setpoint.
   */
  private void runVoltage(double voltage) {
    outputs.mode = HoodIOOutputMode.VOLTAGE;
    outputs.voltage = voltage;
  }

  @AutoLogOutput(key = "Hood/MeasuredAngleRads")
  public double getAngle() {
    return inputs.masterPositionRads;
  }

  @AutoLogOutput(key= "Hood/MeasuredCurrentAmps")
  public double getCurrentAmps(){
    return inputs.masterSupplyCurrentAmps;
  }

  public Command shootCommand() {
    return run(() ->
            runAngular(RobotState.getInstance().getCustomShotData().correctedTargetAngle()))
        .withName("Hood Shoot");
  }

  public Command dynamicUpdatedShootCommand(DoubleSupplier positionRad) {
    return run(() -> runAngular(positionRad.getAsDouble())).withName("Hood Shoot");
  }

  public Command prepareHubCommand() {
    return startEnd(() -> setGoal(Goal.PREPARE_HUB), () -> setGoal(Goal.IDLE))
        .withName("Hood Prepare Hub");
  }

  public Command juggleCommand() {
    return startEnd(() -> setGoal(Goal.JUGGLE), () -> setGoal(Goal.IDLE)).withName("Hood Juggle");
  }

  public Command runDebuggingCommand() {
    return startEnd(() -> setGoal(Goal.DEBUGGING), () -> setGoal(Goal.IDLE)).withName("Hood Debug");
  }

  public Command runStaticAngleCommand() {
    return startEnd(() -> setGoal(Goal.STATIC), () -> setGoal(Goal.IDLE)).withName("Hood Static");
  }

  public Command runDebuggingVoltageUpCommand() {
    return startEnd(() -> setGoal(Goal.DEBUGGING_VOLT_UP), () -> setGoal(Goal.IDLE))
        .withName("Hood Debug Voltage Up");
  }

  public Command runDebuggingVoltageDownCommand() {
    return startEnd(() -> setGoal(Goal.DEBUGGING_VOLT_DOWN), () -> setGoal(Goal.IDLE))
        .withName("Hood Debug Voltage Down");
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
    outputs.mode = HoodIOOutputMode.BRAKE;
  }
}
