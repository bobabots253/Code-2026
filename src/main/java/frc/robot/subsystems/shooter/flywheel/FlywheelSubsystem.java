package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.intake.pivot.PivotConstants.highCurrentAmps;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kShotTolerance;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterSubsystemParameters;
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

  private double lastMeasuredVelocity;
  private double lastTime;
  private double measuredVelocity;
  private double setpoint;

  private double currentTime;
  private double deltaTime;

  double acceleration;
  static boolean isWarm;

  /*
   * Defines all possible states for the flywheel. Each goal state has a DoubleSupplier arguement that updates from Robot State. Note: Only input static choices.
   */
  // ADD MVP STATE
  public enum Goal {
    IDLE,
    JUGGLE,
    DEBUGGING_VELOCITY,
    CURRENT,
    LAYUP
  }

  private double getGoalAsDouble(Goal currentGoal) {
    switch (currentGoal) {
      case IDLE:
        return (!isWarm) ? 0.0 : Units.rotationsPerMinuteToRadiansPerSecond(1500);
      case JUGGLE:
        return FlywheelConstants.jugglingVelocity;
      case DEBUGGING_VELOCITY:
        return FlywheelConstants.debuggingVelocity;
      case CURRENT:
        return FlywheelConstants.debuggingCurrent;
      case LAYUP:
        return FlywheelConstants.layupVelocity;
      default:
        return 0.0;
    }
  }

  @AutoLogOutput(key = "Flywheels/Goal")
  private Goal currentGoal = Goal.IDLE;

  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;

    masterDisconnected = new Alert("Master flywheel disconnected!", Alert.AlertType.kWarning);
    followerDisconnected = new Alert("Follower flywheel disconnected!", Alert.AlertType.kWarning);

    setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Flywheels Idle"));

    lastMeasuredVelocity = 0.0;
    lastTime = Timer.getFPGATimestamp();
    ShooterSubsystemParameters newCoordinator =
        new ShooterSubsystemParameters(0, setpoint, kShotTolerance);
    RobotState.getInstance()
        .setShotCoordinator(newCoordinator); // sets flywheelRPM to kStaticVelocity
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

    isWarm = outputs.isWarm;

    // Re-poll the supplier every loop to handle new shot calculations
    if (currentGoal == Goal.IDLE && !isWarm) {
      stop();
    } else if (currentGoal == Goal.IDLE && isWarm) {
      runVelocityPID(getGoalAsDouble(currentGoal));
    } else if (currentGoal == Goal.CURRENT) {
      runCurrent(getGoalAsDouble(currentGoal));
    } else if (currentGoal == Goal.JUGGLE) {
      runVelocityPID(getGoalAsDouble(currentGoal));
    } else {
      runVelocityPID(getGoalAsDouble(currentGoal));
    }

    // Update Acceleration
    measuredVelocity = inputs.masterVelocityRads;
    setpoint = outputs.velocityRadsPerSec;
    ShooterSubsystemParameters newCoordinator =
        new ShooterSubsystemParameters(measuredVelocity, setpoint, kShotTolerance);
    RobotState.getInstance()
        .setShotCoordinator(newCoordinator); // sets flywheelRPM to kStaticVelocity

    currentTime = Timer.getFPGATimestamp();
    deltaTime = currentTime - lastTime;

    acceleration = (measuredVelocity - lastMeasuredVelocity) / deltaTime;

    lastMeasuredVelocity = measuredVelocity;
    lastTime = currentTime;
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Flywheel/Mode", outputs.mode);
    Logger.recordOutput("Flywheel/isWarm", isWarm);
    Logger.recordOutput("Flywheel/getAsADouble", getGoalAsDouble(currentGoal));
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
        || Math.abs(getVelocity() - getGoalAsDouble(currentGoal))
            <= FlywheelConstants.closedLoopVelocityTolerance;
  }

  private void runVelocityBangBangPseudoTorqueCurrent(double velocityRadsPerSec) {
    outputs.mode = FlywheelIOOutputMode.VELOCITY_BANG_BANG_TORQUE;
    outputs.velocityRadsPerSec = velocityRadsPerSec;
    outputs.measuredVelocityRadPerSec = inputs.masterVelocityRads;
    outputs.calculatedAcceleration = acceleration;
  }

  private void runVoltage(double volts) {
    outputs.mode = FlywheelIOOutputMode.VOLTAGE;
    outputs.voltage = volts;
  }

  private void runCurrent(double current) {
    outputs.mode = FlywheelIOOutputMode.CURRENT;
    outputs.current = current;
  }

  private void runVelocityPID(double velocityRadsPerSec) {
    outputs.mode = FlywheelIOOutputMode.VELOCITY_PID;
    outputs.velocityRadsPerSec = velocityRadsPerSec;
  }

  // ----------------------------------WIP COMMANDS------------------------------//

  public Command dynamicUpdatedShootCommand(DoubleSupplier velocityRadPerSec) {
    return run(() -> runVelocityPID(velocityRadPerSec.getAsDouble()))
        .withName("Flywheels Dyanmic Updated Shoot");
  }

  // ----------------------------------LAYUP COMMANDS------------------------------//

  public Command runLayupCommand() {
    return startEnd(() -> setGoal(Goal.LAYUP), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Layup");
  }

  // ----------------------------------DEBUGGING COMMANDS------------------------------//

  public Command runDebuggingVelocityCommand() {
    return startEnd(() -> setGoal(Goal.DEBUGGING_VELOCITY), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Debug");
  }

  public Command runCurentCommand() {
    return run(() -> setGoal(Goal.CURRENT)).withName("Flywheels Current");
  }

  // ----------------------------------JUGGLE COMMANDS------------------------------//

  public Command juggleCommand() {
    return startEnd(() -> setGoal(Goal.JUGGLE), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Juggle");
  }

  // ----------------------------------WARMUP COMMANDS------------------------------//

  private void toggleWarmup() {
    if (isWarm) {
      outputs.isWarm = false;
    } else {
      outputs.isWarm = true;
    }
  }

  public Command toggleWarm() {
    return runOnce(() -> toggleWarmup());
  }

  /**
   * Manual override command to run a specific velocity. Note: This bypasses the "state machine".
   */
  public Command runSetVelocityCommand(DoubleSupplier velocity) {
    return runEnd(() -> runVelocityPID(velocity.getAsDouble()), this::stop);
  }

  /*
   * Sets the IO mode to coast and resets velocity setpoint.
   */
  public Command stopCommand() {
    return runOnce(this::stop);
  }

  @SuppressWarnings("unused")
  private boolean isDrawingHighCurrent() {
    return Math.abs(inputs.masterSupplyCurrentAmps) > highCurrentAmps
        || Math.abs(inputs.followerSupplyCurrentAmps) > highCurrentAmps;
  }

  @AutoLogOutput(key = "Flywheel/MeasuredVelocity")
  public double getVelocity() {
    return inputs.masterVelocityRads;
  }

  @AutoLogOutput(key = "Flywheel/CalculatedAcceleration")
  public double getAcceleration() {
    return acceleration;
  }

  private void stop() {
    outputs.mode = FlywheelIOOutputMode.COAST;
    outputs.velocityRadsPerSec = 0.0;
  }

  /*
   * Sets the IO mode to coast and resets velocity setpoint.
   */
  public Command stopCommand() {
    return runOnce(this::stop);
  }

  @SuppressWarnings("unused")
  private boolean isDrawingHighCurrent() {
    return Math.abs(inputs.masterSupplyCurrentAmps) > highCurrentAmps
        || Math.abs(inputs.followerSupplyCurrentAmps) > highCurrentAmps;
  }

  @AutoLogOutput(key = "Flywheel/MeasuredVelocity")
  public double getVelocity() {
    return inputs.masterVelocityRads;
  }

  @AutoLogOutput(key = "Flywheel/CalculatedAcceleration")
  public double getAcceleration() {
    return acceleration;
  }

  private void stop() {
    outputs.mode = FlywheelIOOutputMode.COAST;
    outputs.velocityRadsPerSec = 0.0;
  }

  /*
   * Set isWarm to True
   */
  // public void toggleWarmup() {
  //   if (isWarm) {
  //     isWarm = false;
  //   } else {
  //     isWarm = true;
  //   }
  // }
}
