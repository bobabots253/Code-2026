package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.flywheelTolerance;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelOutputMode;
import frc.robot.util.FullSubsystem;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends FullSubsystem {
  private final FlywheelIO io; // initilializes the flywheel io.
  private final Alert
      flywheelMasterDisconnectedAlert; // initializes alerts for if a motor disconnects
  private final Alert flywheelFollowerDisconnectedAlert;
  private final FlywheelIOInputsAutoLogged inputs =
      new FlywheelIOInputsAutoLogged(); // creates new inputs and outputs, which are logged.
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  @RequiredArgsConstructor
  public enum Goal {
    IDLE(() -> 0.0),
    PREPARE_HUB(() -> RobotState.getInstance().getCustomShotData().correctTargetVelocity()),
    SHOOT(() -> RobotState.getInstance().getCustomShotData().correctTargetVelocity()),
    JUGGLING(() -> FlywheelConstants.jugglingVelocity),
    DEBUGGING(() -> FlywheelConstants.debuggingVelocity);

    private final DoubleSupplier velocityRadsPerSec;

    private double getGoal() {
      return velocityRadsPerSec.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Flywheels/Goal")
  private Goal currentGoal = Goal.IDLE;

  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io; // creates the actual io, which will be FlywheelIO io (above)
    flywheelMasterDisconnectedAlert =
        new Alert(
            "Disconnected Master motor in flywheel",
            AlertType.kError); // gives the alerts text to help identify them
    flywheelFollowerDisconnectedAlert =
        new Alert("Disconnected Follower motor in flywheel", AlertType.kError);

    setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Flywheel's Idle"));
  }

  @Override
  public void periodic() { // things here are called every 20ms

    if (DriverStation.isDisabled()) {
      setGoal(Goal.IDLE);
    }

    if (currentGoal == Goal.IDLE) {
      stop();
    } else {
      runVelocity(currentGoal.getGoal());
    }
    io.updateInputs(inputs); // runs the io's updateInputs function based on our autologged inputs.

    Logger.processInputs("Flywheel / Inputs", inputs); // puts the inputs on advantage scope.
    flywheelMasterDisconnectedAlert.set( // sets the parameters needed for our alerts to trigger
        Robot.showHardwareAlerts()
            && !(inputs
                .flywheelMasterConnected)); // the alerts trigger if our flywheelMasterConnected
    // input is false for more than 0.2 sec
    flywheelFollowerDisconnectedAlert.set(
        Robot.showHardwareAlerts() && !(inputs.flywheelFollowerConnected));
  }

  @Override
  public void
      periodicAfterScheduler() { // this method runs after periodic, and is used to apply outputs
    Logger.recordOutput("Flywheel / Outputs", outputs.mode); // puts our outputs in advantageScope.
    io.applyOutputs(outputs); // runs the io's applyOutputs function with our outputs from above.
  }

  private void setGoal(Goal desiredGoal) {
    if (desiredGoal == Goal.IDLE) {
      outputs.mode = FlywheelOutputMode.COAST;
      return;

    } else if (desiredGoal == Goal.DEBUGGING) {
      outputs.mode = FlywheelOutputMode.VOLTAGE;

    } else {
      this.currentGoal = desiredGoal;
      runVelocity(currentGoal.getGoal());
    }
  }

  @AutoLogOutput(key = "Flywheel/AtGoal")
  public boolean atGoal() {
    return currentGoal == Goal.IDLE
        || Math.abs(getSpeed() - currentGoal.getGoal()) <= FlywheelConstants.flywheelTolerance;
  }

  private void runVolts(double volts) { // method to change the flywheel's mode to VOLTAGE.
    outputs.mode = FlywheelOutputMode.VOLTAGE;
    outputs.volts =
        volts; // sets the target voltage (output.volts) to the double volts passed to the method
  }

  private void runVelocity(
      double velocityRadsPerSec) { // method to change the flywheel's mode to BANG_BANG
    outputs.mode = FlywheelOutputMode.BANG_BANG;
    outputs.velocityRadsPerSec =
        velocityRadsPerSec; // sets the target velocity (output.velocityRadPerSec) to the double
    // velocityRadPerSec passed to the method
    outputs.measuredVelocityRadPerSec = inputs.flywheelMasterVelocityRad;
  }

  public double
      getSpeed() { // method which returns the speed of the flywheel, useful for the shot calculator
    return inputs.flywheelMasterVelocityRad;
  }

  public Boolean
      atSpeed() { // method which checks is the flywheel is within 1 rad/sec of the target speed.
    if (Math.abs(inputs.flywheelMasterVelocityRad - outputs.velocityRadsPerSec)
        <= flywheelTolerance) {
      return true;
    } else {
      return false;
    }
  }

  private void stop() { // method to set the flywheel's mode to COAST, which will stop it.
    outputs.mode = FlywheelOutputMode.COAST;
    outputs.velocityRadsPerSec =
        0.0; // sets velocity and volt setpoints to 0 to ensure the motor stops.
    outputs.volts = 0.0;
  }

  public Command shootCommand() {
    return startEnd(() -> setGoal(Goal.SHOOT), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Shoot");
  }

  public Command prepareHubCommand() {
    return startEnd(() -> setGoal(Goal.PREPARE_HUB), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Shoot");
  }

  public Command jugglingCommand() {
    return startEnd(() -> setGoal(Goal.JUGGLING), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Shoot");
  }

  public Command debuggingCommand() {
    return startEnd(() -> setGoal(Goal.DEBUGGING), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Shoot");
  }

  public Command stopCommand() { // command that runs our stop method to stop the flywheel.
    return runOnce(this::stop);
  }

  public Command runFixedVelocity(
      DoubleSupplier
          velocity) { // command that runs our runVelocity method, and runs the stop method if
    // interrupted.
    return runEnd(() -> runVelocity(velocity.getAsDouble()), this::stop);
  }

  public Command runVoltage(
      DoubleSupplier volts) { // command that runs our runVolts method, and runs the stop method if
    // interrupted.
    return runEnd(() -> runVolts(volts.getAsDouble()), this::stop);
  }
}
