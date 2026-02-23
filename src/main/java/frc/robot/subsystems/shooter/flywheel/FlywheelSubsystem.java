package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputMode;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import frc.robot.util.FullSubsystem;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends FullSubsystem {

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  private Alert masterDisconnected;
  private Alert followerDisconnected;

  @RequiredArgsConstructor
  /*
   * Defines all possible states for the flywheel. Each goal state has a DoubleSupplier arguement that updates from Robot State
   */
  // ADD MVP STATE
  public enum Goal {
    // Stop the flywheel and chooses COAST Mode
    IDLE(() -> 0.0),
    // Velocity setpoint calculated by the ShotCalculator for the Hub.
    PREPARE_HUB(() -> RobotState.getInstance().getCustomShotData().correctTargetVelocity()),
    // Currently mirrors PREPARE_HUB but I added it so it can edited/tuned separately
    SHOOT(
        () ->
            RobotState.getInstance()
                .getCustomShotData()
                .correctTargetVelocity()), // Change if Necessary
    // Low-speed state for juggling them balls into our own hopper LOL
    JUGGLE(() -> FlywheelConstants.jugglingVelocity),
    // Static speed state for subsystem testing
    DEBUGGING(() -> FlywheelConstants.debuggingVelocity);

    // Required Arguement for each enum state
    private final DoubleSupplier velocityRadsPerSec;

    /** Returns the current target velocity for this goal state. */
    private double getGoal() {
      return velocityRadsPerSec.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Flywheels/Goal")
  private Goal currentGoal = Goal.IDLE;

  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;

    masterDisconnected = new Alert("Master flywheel disconnected!", Alert.AlertType.kWarning);
    followerDisconnected = new Alert("Follower flywheel disconnected!", Alert.AlertType.kWarning);

    setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Flywheels Idle"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    masterDisconnected.set(Robot.showHardwareAlerts() && (!inputs.masterMotorConnected));
    followerDisconnected.set(Robot.showHardwareAlerts() && (!inputs.followerMotorConnected));

    // Force IDLE state if the robot is disabled so it doesn't jump rpms on enable
    if (DriverStation.isDisabled()) {
      setGoal(Goal.IDLE);
    }

    // Re-poll the supplier every loop to handle new shot calculations
    if (currentGoal == Goal.IDLE) {
      stop();
    } else {
      runVelocity(currentGoal.getGoal());
    }
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Flywheel/Mode", outputs.mode);
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

  @AutoLogOutput(key = "Flywheel/AtGoal")
  /**
   * Returns true if the flywheel is within the velocity tolerance. Note: IDLE check catches the
   * exception because I'm lazy
   */
  public boolean atGoal() {
    return currentGoal == Goal.IDLE
        || Math.abs(getVelocity() - currentGoal.getGoal())
            <= FlywheelConstants.closedLoopVelocityTolerance;
  }

  /**
   * Update the io for velocity closed loop control and applies the new velocity setpoint
   *
   * @param velocityRadsPerSec the new velocity setpoint.
   */
  private void runVelocity(double velocityRadsPerSec) {
    outputs.mode = FlywheelIOOutputMode.VELOCITY_SETPOINT;
    outputs.velocityRadsPerSec = velocityRadsPerSec;
    outputs.measuredVelocityRadPerSec = inputs.masterVelocityRads;
  }

  // private void runFlywheelControlLoop(double velocityRadsPerSec){
  //   double measuredVelocity = inputs.masterVelocityRads;
  //   double setpoint = velocityRadsPerSec;
  //   double error = setpoint - measuredVelocity;

  // }

  public Command shootCommand() {
    return startEnd(() -> setGoal(Goal.SHOOT), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Shoot");
  }

  public Command prepareHubCommand() {
    return startEnd(() -> setGoal(Goal.PREPARE_HUB), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Prepare Hub");
  }

  public Command juggleCommand() {
    return startEnd(() -> setGoal(Goal.JUGGLE), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Juggle");
  }

  public Command runDebuggingCommand() {
    return startEnd(() -> setGoal(Goal.DEBUGGING), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Debug");
  }

  /**
   * Manual override command to run a specific velocity. Note: This bypasses the "state machine".
   */
  public Command runSetVelocityCommand(DoubleSupplier velocity) {
    return runEnd(() -> runVelocity(velocity.getAsDouble()), this::stop);
  }

  /*
   * Sets the IO mode to coast and resets velocity setpoint.
   */
  public Command stopCommand() {
    return runOnce(this::stop);
  }

  @SuppressWarnings("unused")
  private boolean isDrawingHighCurrent() {
    return Math.abs(inputs.masterSupplyCurrentAmps) > 50.0
        || Math.abs(inputs.followerSupplyCurrentAmps) > 50.0;
  }

  @AutoLogOutput(key = "Flywheel/MeasuredVelocity")
  public double getVelocity() {
    return inputs.masterVelocityRads;
  }

  private void stop() {
    outputs.mode = FlywheelIOOutputMode.COAST;
    outputs.velocityRadsPerSec = 0.0;
  }
}
