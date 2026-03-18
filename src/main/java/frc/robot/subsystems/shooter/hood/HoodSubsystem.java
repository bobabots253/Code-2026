package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.highCurrentThreshold;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputMode;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import frc.robot.util.FullSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends FullSubsystem {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  private Alert masterDisconnected;

  /*
   * Defines all possible states for the hood. Each goal state has a DoubleSupplier arguement that updates from Robot State. Note: Only input static choices.
   */
  public enum Goal {
    IDLE,
    JUGGLE,
    DEBUGGING_PID_UP,
    DEBUGGING_PID_DOWN,
    DEBUGGING_VOLT_UP,
    DEBUGGING_VOLT_DOWN,
    LAYUP;
  }

  private double getGoalAsDouble(Goal currentGoal) {
    switch (currentGoal) {
      case IDLE:
        return 0.0;
      case JUGGLE:
        return HoodConstants.jugglingAngle;
      case DEBUGGING_PID_UP:
        return HoodConstants.debuggingAngleUp;
      case DEBUGGING_PID_DOWN:
        return HoodConstants.debuggingAngleDown;
      case DEBUGGING_VOLT_UP:
        return HoodConstants.debuggingVoltageUP;
      case DEBUGGING_VOLT_DOWN:
        return HoodConstants.debuggingVoltageDOWN;
      case LAYUP:
        return HoodConstants.layupAngle;
      default:
        return 0.0;
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
      runVoltage(getGoalAsDouble(currentGoal));
    } else {
      runAngular(getGoalAsDouble(currentGoal));
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

  /**
   * Returns true if the hood is within the angular tolerance. Note: IDLE check catches the
   * exception because I'm lazy
   */
  @AutoLogOutput(key = "Hood/AtGoal")
  public boolean atGoal() {
    return currentGoal == Goal.IDLE
        || Math.abs(getAngle() - getGoalAsDouble(currentGoal))
            <= HoodConstants.closedLoopAngularTolerance;
  }

  /**
   * Update the io for angular closed loop control and applies the new angular setpoint.
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

  public boolean isDrawingHighCurrent() {
    return Math.abs(inputs.masterSupplyCurrentAmps) > highCurrentThreshold;
  }

  // ----------------------------------WIP COMMANDS------------------------------//

  // Working 3/10/26, doesn't respect State-Based but allows for dynamic updates to hood angle from
  // vision
  public Command dynamicUpdatedShootCommand(DoubleSupplier positionRad) {
    return run(() -> runAngular(positionRad.getAsDouble())).withName("Hood Shoot");
  }

  // ----------------------------------LAYUP COMMANDS------------------------------//

  public Command runLayupCommand() {
    return startEnd(() -> setGoal(Goal.LAYUP), () -> setGoal(Goal.IDLE)).withName("Hood Layup");
  }

  // ----------------------------------DEBUGGING COMMANDS------------------------------//

  /*
   * Moves the hood up. NOT INCREASE ANGLE
   */
  public Command runDebuggingVoltageUpCommand() {
    return startEnd(() -> setGoal(Goal.DEBUGGING_VOLT_UP), () -> setGoal(Goal.IDLE))
        .withName("Hood Debug Voltage Up");
  }

  /*
   * Moves the hood down. NOT DECREASE ANGLE
   */
  public Command runDebuggingVoltageDownCommand() {
    return startEnd(() -> setGoal(Goal.DEBUGGING_VOLT_DOWN), () -> setGoal(Goal.IDLE))
        .withName("Hood Debug Voltage Down");
  }

  public Command runDebuggingUpCommand() {
    return startEnd(() -> setGoal(Goal.DEBUGGING_PID_UP), () -> setGoal(Goal.IDLE))
        .withName("Hood Debug");
  }

  public Command runDebuggingDownCommand() {
    return startEnd(() -> setGoal(Goal.DEBUGGING_PID_DOWN), () -> setGoal(Goal.IDLE))
        .withName("Hood Debug");
  }

  // ----------------------------------JUGGLE COMMANDS------------------------------//
  public Command juggleCommand() {
    return startEnd(() -> setGoal(Goal.JUGGLE), () -> setGoal(Goal.IDLE)).withName("Hood Juggle");
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
