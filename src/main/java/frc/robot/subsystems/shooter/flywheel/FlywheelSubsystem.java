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
import frc.robot.RobotState.ShotCoordinator;
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

  @RequiredArgsConstructor
  /*
   * Defines all possible states for the flywheel. Each goal state has a DoubleSupplier arguement that updates from Robot State
   */
  // ADD MVP STATE
  public enum Goal {
    IDLE,
    SHOOT,
    JUGGLE,
    DEBUGGING,
    CURRENT,
    DEBUGGING_VELOCITY,
    STATIC,
    PREPARE_HUB
    // Stop the flywheel and chooses COAST Mode
    // IDLE(() -> isWarm ? 0.0 : Units.rotationsPerMinuteToRadiansPerSecond(500)),
    // // Use a helper functions to
    // // Velocity setpoint calculated by the ShotCalculator for the Hub.
    // PREPARE_HUB(() -> RobotState.getInstance().getCustomShotData().correctTargetVelocity()),
    // // Currently mirrors PREPARE_HUB but I added it so it can edited/tuned separately
    // SHOOT(
    //     () ->
    //         RobotState.getInstance()
    //             .getCustomShotData()
    //             .correctTargetVelocity()), // Change if Necessary
    // // Low-speed state for juggling them balls into our own hopper LOL
    // JUGGLE(() -> FlywheelConstants.jugglingVelocity),
    // // Static speed state for subsystem testing
    // DEBUGGING(() -> FlywheelConstants.kDebuggingVoltage), // VOLTAGE
    // CURRENT(() -> FlywheelConstants.kDebuggingCurrent),
    // DEBUGGING_VELOCITY(() -> FlywheelConstants.kDebuggingVelocity),
    // STATIC(() -> FlywheelConstants.kStaticVelocity);

    // // Required Arguement for each enum state
    // private final DoubleSupplier velocityRadsPerSec;

    // /** Returns the current target velocity for this goal state. */
    // private double getGoal() {
    //   return velocityRadsPerSec.getAsDouble();
    // }

    // private double getAsDouble() {
    //   switch this {
    //     case JUGGLE:
    //       return FlywheelConstants.jugglingVelocity;
    //   }
    // }
  }

  private double getAsDouble(Goal currentGoal) {
    switch (currentGoal) {
      case IDLE:
        return (!isWarm) ? 0.0 : Units.rotationsPerMinuteToRadiansPerSecond(1500);
      case SHOOT:
        return RobotState.getInstance().getCustomShotData().correctTargetVelocity();
      case JUGGLE:
        return FlywheelConstants.jugglingVelocity;
      case DEBUGGING:
        return FlywheelConstants.jugglingVelocity;
      case DEBUGGING_VELOCITY:
        return FlywheelConstants.kDebuggingVelocity;
      case CURRENT:
        return FlywheelConstants.kDebuggingCurrent;
      case STATIC:
        return FlywheelConstants.kStaticVelocity;
      case PREPARE_HUB:
        return RobotState.getInstance().getCustomShotData().correctTargetVelocity();
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
    ShotCoordinator newCoordinator = new ShotCoordinator(0, setpoint, kShotTolerance);
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
      runVelocity(getAsDouble(currentGoal));
    } else if (currentGoal == Goal.DEBUGGING) {
      runVoltage(getAsDouble(currentGoal));
    } else if (currentGoal == Goal.CURRENT) {
      runCurrent(getAsDouble(currentGoal));
    } else {
      runVelocity(getAsDouble(currentGoal));
    }

    // Update Acceleration
    measuredVelocity = inputs.masterVelocityRads;
    ShotCoordinator newCoordinator = new ShotCoordinator(measuredVelocity, setpoint, kShotTolerance);
    RobotState.getInstance()
        .setShotCoordinator(newCoordinator); // sets flywheelRPM to kStaticVelocity
    setpoint = outputs.velocityRadsPerSec;

    currentTime = Timer.getFPGATimestamp();
    deltaTime = currentTime - lastTime;

    acceleration = (measuredVelocity - lastMeasuredVelocity) / deltaTime;

    lastMeasuredVelocity = measuredVelocity;
    lastTime = currentTime;
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Flywheel/Mode", outputs.mode);
    Logger.recordOutput("Flywheel/localIsWarm", isWarm);
    Logger.recordOutput("Flywheel/getAsADouble", getAsDouble(currentGoal));
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
        || Math.abs(getVelocity() - getAsDouble(currentGoal))
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

  private void toggleWarmup() {
    if (isWarm) {
      outputs.isWarm = false;
    } else {
      outputs.isWarm = true;
    }
  }

  // private void runFlywheelControlLoop(double velocityRadsPerSec){
  //   double measuredVelocity = inputs.masterVelocityRads;
  //   double setpoint = velocityRadsPerSec;
  //   double error = setpoint - measuredVelocity;

  // }

  public Command shootCommand() {
    return run(() ->
            runVelocity(RobotState.getInstance().getCustomShotData().correctTargetVelocity()))
        .withName("Flywheels Shoot");
  }

  public Command toggleWarm() {
    return runOnce(() -> toggleWarmup());
  }

  public Command dynamicUpdatedShootCommand(DoubleSupplier velocityRadPerSec) {
    return run(() -> runVelocity(velocityRadPerSec.getAsDouble()))
        .withName("Flywheels Dyanmic Updated Shoot");
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

  public Command runDebuggingVelocityCommand() {
    return startEnd(() -> setGoal(Goal.DEBUGGING_VELOCITY), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Debug");
  }

  public Command runStaticVelocitCommand() {
    return startEnd(() -> setGoal(Goal.STATIC), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Static");
  }

  public Command runCurentCommand() {
    return run(() -> setGoal(Goal.CURRENT)).withName("Flywheels Current");
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
