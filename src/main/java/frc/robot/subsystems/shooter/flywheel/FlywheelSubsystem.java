package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.fieldSetup;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOInputs;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelOutputMode;
import frc.robot.util.FullSubsystem;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.flywheelTolerance;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.opencv.ml.RTrees;

public class FlywheelSubsystem extends FullSubsystem {
  private final FlywheelIO io;
  private final Alert flywheelMasterDisconnectedAlert;
  private final Alert flywheelFollowerDisconnectedAlert;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();
  private final Debouncer flywheelDebouncer = new Debouncer(0.2,DebounceType.kFalling);

  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;
    flywheelMasterDisconnectedAlert = new Alert("Disconnected Master motor in flywheel", AlertType.kError);
    flywheelFollowerDisconnectedAlert = new Alert("Disconnected Follower motor in flywheel", AlertType.kError);

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel / Inputs", inputs);
    flywheelMasterDisconnectedAlert.set(
    Robot.showHardwareAlerts() && flywheelDebouncer.calculate(inputs.flywheelMasterConnected));
    flywheelFollowerDisconnectedAlert.set(
    Robot.showHardwareAlerts() && flywheelDebouncer.calculate(inputs.flywheelFollowerConnected));
  }

  @Override
  public void periodicAfterScheduler(){
    Logger.recordOutput("Flywheel / Outputs", outputs.mode);
    io.applyOutputs(outputs);

  }

private void runVolts(double volts){
  outputs.mode = FlywheelOutputMode.VOLTAGE;
}

private void runVelocity(double velocityRadsPerSec){
  outputs.mode = FlywheelOutputMode.BANG_BANG;
  outputs.velocityRadsPerSec = velocityRadsPerSec;
}

   public double getSpeed(){
    return inputs.flywheelMasterVelocityRad;
  }

  public Boolean atSpeed(){
    if (Math.abs(inputs.flywheelMasterVelocityRad - outputs.velocityRadsPerSec) <= flywheelTolerance){
        return true; 
    } else {
        return false;
    }
  }

  private void stop(){
    outputs.mode = FlywheelOutputMode.COAST;
    outputs.velocityRadsPerSec = 0.0;
  }

    public Command stopCommand(){
    return runOnce(this::stop);
  }

  public Command runFixedVelocity(DoubleSupplier velocity){
    return runEnd(() -> runVelocity(velocity.getAsDouble()),this::stop);
  }

   public Command runVolts(DoubleSupplier volts){
    return runEnd(() -> runVelocity(volts.getAsDouble()),this::stop);
  }

  // public Command runCalculatedVelocity(){
  //   return runEnd (() -> runVelocity(), this::stop);
  // }
}
